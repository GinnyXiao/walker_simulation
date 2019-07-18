////////////////////////////////////
//    Athour: Andrew Dornbush     //
////////////////////////////////////

#include <assert.h>
#include <atomic>
#include <limits>
#include <ros/ros.h>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <boost/circular_buffer.hpp>

#include <Eigen/Dense>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using moveit::planning_interface::MoveGroup;

using FollowJointTrajectoryActionClient = actionlib::SimpleActionClient<
        control_msgs::FollowJointTrajectoryAction>;

double ADJUST = 0.02;

// PrepareGripper -> WaitForGoal
// WaitForGoal -> Finished
// WaitForGoal -> ExecutePickup
// ExecutePickup -> GraspObject
// ExecutePickup -> WaitForGoal (failure case)
// GraspObject -> CloseGripper
// CloserGripper -> PlanDropoff
// PlanDropoff -> (TODO: failure case...what do we do if we can't move back?)
// PlanDropoff -> ExecuteDropoff
// ExecuteDropoff -> WaitForGoal
// ExecuteDropoff -> (TODO: failure case...what do we do if we can't move back?)

enum struct PickState
{
    Finished = -1,
    PrepareGripper = 0,
    WaitForGoal,
    ExecutePickup,
    PlanPickup,
    GraspObject,
    CloseGripper,
    // distinguish between planning and execution for dropoff so that we can
    // plan for an idle arm while the busy arm is moving (moveit only lets us
    // plan for one group at a time)
    PlanDropoff,
    ExecuteDropoff,
    OpenGripper,
    MoveToHome,
    Count
};

struct PickMachine;

struct PickMachState
{
    using EnterFn = void (*)(PickMachine* mach, PickState);
    using PumpFn = PickState (*)(PickMachine* mach);
    using ExitFn = void (*)(PickMachine* mach, PickState);

    EnterFn enter = NULL;
    PumpFn pump = NULL;
    ExitFn exit = NULL;
};

struct PickMachine
{
    PickState prev_state;
    PickState curr_state;
    PickState next_state;

    PickMachState* states;

    std::unique_ptr<MoveGroup> move_group;
    // std::unique_ptr<GripperCommandActionClient> gripper_client;
    std::unique_ptr<FollowJointTrajectoryActionClient> follow_joint_trajectory_client;

    std::atomic<bool> goal_ready;

    ros::Time time_at_execute;

    double time_to_reach_grasp = 10; //4.2;

    // NOTE: workspace boundaries are w.r.t. the tool frame
    double min_workspace_y;
    double max_workspace_y;

    double conveyor_speed;
    double time_to_grasp;
    uint32_t claimed_id = std::numeric_limits<uint32_t>::max();

    std::vector<double> dropoff_position;
    std::vector<double> home_position;

    MoveGroup::Plan pickup_plan;
    MoveGroup::Plan dropoff_plan;

    geometry_msgs::PoseStamped grasp_pose_goal_base_footprint;
    geometry_msgs::PoseStamped grasp_pose_goal;
};

struct WorldObject
{
    bool claimed;
    uint32_t id;
    boost::circular_buffer<ar_track_alvar_msgs::AlvarMarker> pose_estimates;
};

struct WorldState
{
    std::mutex objects_mutex;
    std::vector<WorldObject> objects;
};

// truly global
std::unique_ptr<tf::TransformBroadcaster> g_broadcaster;
std::unique_ptr<tf::TransformListener> g_listener;
const char* g_planning_frame = "odom_combined"; // TODO: take this from the MoveGroupInterface
const char* g_robot_frame = "base_footprint";
std::atomic<bool> g_move_group_busy(false);

// an upper bound on the time to plan and execute the motion to a grasp location
WorldState g_world_state;

void WaitForMoveGroup()
{
    while (g_move_group_busy) { ros::Duration(0.02).sleep(); }
}

bool TryHardTransformVector(
    const std::string& frame_id,
    const geometry_msgs::Vector3Stamped& vector_in,
    geometry_msgs::Vector3Stamped& vector_out)
{
    while (true) {
        try {
            if (!ros::ok()) return false;
            g_listener->transformVector(frame_id, vector_in, vector_out);
            return true;
        } catch (...) {

        }
        ros::Duration(0.01).sleep();
    }
}

bool TryHardTransformPose(
    const std::string& frame_id,
    const geometry_msgs::PoseStamped& pose_in,
    geometry_msgs::PoseStamped& pose_out)
{
    while (true) {
        try {
            if (!ros::ok()) return false;
            g_listener->transformPose(frame_id, pose_in, pose_out);
            return true;
        } catch (...) {
            ROS_INFO("Transform not available from '%s' to '%s'", pose_in.header.frame_id.c_str(), frame_id.c_str());
        }
        ros::Duration(0.01).sleep();
    }
}

void LockWorldState(WorldState* state)
{
    state->objects_mutex.lock();
}

void UnlockWorldState(WorldState* state)
{
    state->objects_mutex.unlock();
}

void UpdateWorldState(
    WorldState* state,
    const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    for (auto& marker : msg.markers) {
        bool found = false;
        for (auto& object : state->objects) {
            if (object.id == marker.id) {
                found = true;
                object.pose_estimates.push_back(marker);
                break;
            }
        }

        if (!found) {
            WorldObject object;
            object.id = marker.id;
            object.pose_estimates.set_capacity(30);
            object.pose_estimates.push_back(marker);
            object.pose_estimates.back().header.frame_id = "base_footprint";
            object.pose_estimates.back().pose.header.frame_id = "base_footprint";
            object.claimed = false;
            state->objects.push_back(object);
        }
    }
}

void RemoveObject(WorldState* state, uint32_t id)
{
    auto rit = std::remove_if(begin(state->objects), end(state->objects),
            [&](const WorldObject& object)
            {
                return object.id == id;
            });
    state->objects.erase(rit, end(state->objects));
}

// possible criteria for ensuring a good estimate
// 0. recent timestamps
// 1. sufficient number of samples
// 2. sufficient history in time
// 3. no significant gaps in history
// 4. no significant pose jumps in history
// 5. resample/extrapolate the intermediate poses
// 6. downsample/filter samples for noise
// 7. more weight to the recent samples
bool EstimateObjectVelocity(
    const ros::Time& now,
    const WorldObject* object,
    const std::string& frame_id,
    double span,
    int min_samples,
    geometry_msgs::Vector3Stamped* vel)
{
    ROS_DEBUG("Estimate velocity for object %u", object->id);

    if (object->pose_estimates.size() < 2) {
        ROS_DEBUG(" -> Insufficient pose estimates");
        return false;
    }

    vel->vector.x = 0.0;
    vel->vector.y = 0.0;
    vel->vector.z = 0.0;

    // use at least 1 (current) velocity sample...if we have more than one...drop all that are too old
    auto span_begin = now - ros::Duration(span);

    // check for sufficient samples within time span
    auto samples = 0;
    for (size_t i = object->pose_estimates.size() - 1; i >= 1; --i) {
        auto& curr_pose = object->pose_estimates[i].pose;
        auto& prev_pose = object->pose_estimates[i - 1].pose;

        if (curr_pose.header.stamp < now - ros::Duration(1.0) ||
            prev_pose.header.stamp < now - ros::Duration(1.0))
        {
            ROS_DEBUG(" -> Sample pair too old!");
            continue;
        }

        // sufficient samples and remaining data is too old to care about
        if (samples >= 1 && object->pose_estimates[i].header.stamp < span_begin) {
            ROS_DEBUG(" -> Got enough samples!");
            break;
        }

        auto dx = (curr_pose.pose.position.x - prev_pose.pose.position.x);
        auto dy = (curr_pose.pose.position.y - prev_pose.pose.position.y);
        auto dz = (curr_pose.pose.position.z - prev_pose.pose.position.z);
        auto dt = curr_pose.header.stamp.toSec() - prev_pose.header.stamp.toSec();
        vel->vector.x += dx / dt;
        vel->vector.y += dy / dt;
        vel->vector.z += dz / dt;
        ROS_DEBUG("  -> Sample v = (%f, %f, %f) / %f", dx, dy, dz, dt);

        ++samples;
    }

    if (samples == 0) {
        ROS_DEBUG(" -> No recent velocity samples");
        return false;
    }

    vel->vector.x /= (double)samples;
    vel->vector.y /= (double)samples;
    vel->vector.z /= (double)samples;

    vel->header = object->pose_estimates.back().pose.header;
    return true;

#if 0
    // no samples at all...
    if (object->pose_estimates.empty()) return false;

    // not enough samples in time
    auto& most_recent_time = now; //object->pose_estimates.back().header.stamp;
    auto span_begin = most_recent_time - ros::Duration(span);
    if (object->pose_estimates.front().header.stamp > span_begin) return false;

    // check for sufficient samples within time span
    auto samples = 0;
    for (size_t i = object->pose_estimates.size() - 1; i >= 0; --i) {
        if (object->pose_estimates[i].header.stamp < span_begin) break;
        ++samples;
    }

    if (samples < min_samples) return false;

    vel->vector.x = 0;
    vel->vector.y = 0;
    vel->vector.z = 0;
    for (auto i = 0; i < samples - 1; ++i) {
        auto& curr_pose = object->pose_estimates[object->pose_estimates.size() - 1 - i].pose;
        auto& prev_pose = object->pose_estimates[object->pose_estimates.size() - 1 - i - 1].pose;
        auto dt = curr_pose.header.stamp.toSec() - prev_pose.header.stamp.toSec();
        vel->vector.x += (curr_pose.pose.position.x - prev_pose.pose.position.x) / dt;
        vel->vector.y += (curr_pose.pose.position.y - prev_pose.pose.position.y) / dt;
        vel->vector.z += (curr_pose.pose.position.z - prev_pose.pose.position.z) / dt;
    }
    vel->vector.x /= (samples - 1);
    vel->vector.y /= (samples - 1);
    vel->vector.z /= (samples - 1);

    vel->header = object->pose_estimates.back().pose.header;

    return true;
#endif
}

///////////////////////////
// OLD PER-OBJECT GRASPS //
///////////////////////////

std::unordered_map<int, geometry_msgs::PoseStamped> g_id_to_pose;
void fillGraspPoses()
{
    // hardcoded grasp poses defined in marker frames (not using for now)

    geometry_msgs::PoseStamped p;
    p.header.frame_id = "ar_marker_7";
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.707;
    p.pose.orientation.w = 0.707;
    p.pose.position.x = -0.005;
    p.pose.position.y = 0.196;
    p.pose.position.z = -0.036;
    g_id_to_pose.insert(std::pair<int, geometry_msgs::PoseStamped> (7, p));
}

/////////////////////////
// OLD MARKER TRACKING //
/////////////////////////

ar_track_alvar_msgs::AlvarMarkers g_markers;
std::unordered_map<uint32_t, int> g_marker_counts;
std::mutex g_marker_mutex;

void ARPoseCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    g_markers = msg;
    LockWorldState(&g_world_state);
    UpdateWorldState(&g_world_state, msg);
    UnlockWorldState(&g_world_state);
}

auto to_cstring(PickState state) -> const char*
{
    switch (state) {
    case PickState::PrepareGripper:     return "PrepareGripper";
    case PickState::WaitForGoal:        return "WaitForGoal";
    case PickState::PlanPickup:         return "PlanPickup";
    case PickState::ExecutePickup:      return "ExecutePickup";
    case PickState::GraspObject:        return "GraspObject";
    case PickState::CloseGripper:       return "CloseGripper";
    case PickState::PlanDropoff:        return "PlanDropoff";
    case PickState::ExecuteDropoff:     return "ExecuteDropoff";
    case PickState::OpenGripper:        return "OpenGripper";
    case PickState::MoveToHome:         return "MoveToHome";
    default:                            return "<Unknown>";
    }
};

void DoLocalizeConveyor()
{
    // TODO: we want to estimate the speed of the conveyor once on startup here
    // ...position would be nice too but I won't cry about it
}

bool ResetArms(PickMachine* l_mach, PickMachine* r_mach)
{
    {
        auto v = l_mach->home_position;
        // for (auto& value : v) {
        //     value *= M_PI / 180.0;
        // }

        l_mach->move_group->setJointValueTarget(v);

        {
            moveit::planning_interface::MoveGroup::Plan plan;
            auto err = l_mach->move_group->plan(plan);
            if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                ROS_WARN("Failed to plan to home position");
                return false;
            }

            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory = plan.trajectory_.joint_trajectory;

            auto state = l_mach->follow_joint_trajectory_client->sendGoalAndWait(goal);
            if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_ERROR("Failed to execute trajectory to home position");
                return false;
            }
        }
    }

    {
        auto v = r_mach->home_position;
        for (auto& value : v) {
            value *= M_PI / 180.0;
        }

        r_mach->move_group->setJointValueTarget(v);

        {
            moveit::planning_interface::MoveGroup::Plan plan;
            auto err = r_mach->move_group->plan(plan);
            if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                ROS_WARN("Failed to plan to home position");
                return false;
            }

            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory = plan.trajectory_.joint_trajectory;

            auto state = r_mach->follow_joint_trajectory_client->sendGoalAndWait(goal);
            if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_ERROR("Failed to move to execute trajectory to home position");
                return false;
            }
        }
    }

    return true;
}

bool ResetArm(PickMachine* mach)
{
    auto v = mach->home_position;
    // for (auto& value : v) {
    //     value *= M_PI / 180.0;
    // }

    mach->move_group->setJointValueTarget(v);

    // moveit::planning_interface::MoveGroup::Plan plan;
    // auto err = mach->move_group->plan(plan);
    auto err = mach->move_group->move();
    if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_WARN("Failed to plan to home position");
        return false;
    }

    // control_msgs::FollowJointTrajectoryGoal goal;
    // goal.trajectory = plan.trajectory_.joint_trajectory;

    // auto state = mach->follow_joint_trajectory_client->sendGoalAndWait(goal);
    // if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
    //     ROS_ERROR("Failed to move to execute trajectory to home position");
    //     return false;
    // }

    // auto err = mach->move_group->execute(plan);

    // if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    //     ROS_ERROR("Failed to move arm to home position");
    //     mach->goal_ready = false;
    //     return PickState::WaitForGoal;
    // }

    return true;
}

PickState DoPrepareGripper(PickMachine* mach)
{
    return PickState::WaitForGoal;
}

PickState DoWaitForGoal(PickMachine* mach)
{
    if (mach->claimed_id != std::numeric_limits<uint32_t>::max()) {
        LockWorldState(&g_world_state);
        for (auto& object : g_world_state.objects) {
            if (object.id == mach->claimed_id) {
                object.claimed = false;
                break;
            }
        }
        UnlockWorldState(&g_world_state);
    }
    while (ros::ok()) {
        if (mach->goal_ready) {
            // mach->goal_ready = false; // consume goal
            return PickState::PlanPickup;
        }
        ros::Duration(1.0).sleep();
    }

    return PickState::Finished;
}

bool SendGoal(
    PickMachine* mach,
    const geometry_msgs::PoseStamped& grasp_pose_goal,
    double conveyor_speed,
    double time_to_grasp,
    uint32_t claimed_id) // time for object to reach goal position
{
    if (mach->goal_ready) return false; // we're busy

    geometry_msgs::PoseStamped grasp_pose_goal_global;

    g_listener->waitForTransform(g_planning_frame, g_robot_frame, ros::Time(0), ros::Duration(10.0));
    TryHardTransformPose(g_planning_frame, grasp_pose_goal, grasp_pose_goal_global);

    mach->grasp_pose_goal_base_footprint = grasp_pose_goal;
    mach->grasp_pose_goal = grasp_pose_goal_global;

    mach->time_at_execute = ros::Time::now();
    mach->conveyor_speed = conveyor_speed;
    mach->time_to_grasp = time_to_grasp;
    mach->claimed_id = claimed_id;

    // TODO: GUARANTEE THAT STATE MACHINE IS IN EXECUTE_PICKUP STAGE BEFORE EXITING
    mach->goal_ready = true; 
    return true;
}

bool ComputeGraspGoal(
    PickMachine* mach,
    const geometry_msgs::PoseStamped& object_pose,
    const geometry_msgs::Vector3Stamped& object_vel,
    geometry_msgs::PoseStamped* grasp_pose_goal,
    double* conveyor_speed,
    double* time_to_grasp)
{
    *conveyor_speed = object_vel.vector.y;
    // auto conveyor_speed = mach->conveyor_speed;
    ROS_INFO("Velocity of object is: (%f, %f, %f)", object_vel.vector.x, object_vel.vector.y, object_vel.vector.z);

    ////////////////////////////
    // Construct gripper goal //
    ////////////////////////////

    ROS_INFO("Construct gripper goal");
    ROS_INFO("  Object position = (%f, %f, %f)", object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z);

    // how far the object will have moved during the time it takes to close the
    // gripper
    auto grasp_offset = mach->time_to_reach_grasp * (*conveyor_speed);
    ROS_INFO("Pick at the offset: %f ", grasp_offset);

    // NOTE: object poses assumed to be stored in the robot's local frame (base_footprint)
    grasp_pose_goal->header.frame_id = g_robot_frame;

    // final pose of the object after speed estimation + minor adjustment
    grasp_pose_goal->pose.position.x = object_pose.pose.position.x; // + 0.01;

    // final pose of object after speed estimation + distance object will travel
    // during arm execution
    grasp_pose_goal->pose.position.y = object_pose.pose.position.y + grasp_offset;

    // hardcoded :)
    // grasp_pose_goal->pose.position.z = 0.73 + ADJUST - 0.01; //+= 0.05;
    // grasp_pose_goal->pose.orientation.x = grasp_pose_goal->pose.orientation.y = 0.0;
    // grasp_pose_goal->pose.orientation.z = grasp_pose_goal->pose.orientation.w = 0.5 * sqrt(2.0);
    grasp_pose_goal->pose.position.z = 0.8;
    grasp_pose_goal->pose.orientation.x = 0.1472033;
    grasp_pose_goal->pose.orientation.y = 0.2944066;
    grasp_pose_goal->pose.orientation.z = 0.0;
    grasp_pose_goal->pose.orientation.w = 0.9442754;

    // limit the grasp goal to be reachable (not too far ahead of the robot)
    grasp_pose_goal->pose.position.y = std::min(grasp_pose_goal->pose.position.y, mach->max_workspace_y);

    ROS_INFO("Picking at x: %f ", grasp_pose_goal->pose.position.x);
    ROS_INFO("Picking at y: %f ", grasp_pose_goal->pose.position.y);
    ROS_INFO("Picking at z: %f ", grasp_pose_goal->pose.position.z);

    *time_to_grasp = std::fabs((object_pose.pose.position.y - grasp_pose_goal->pose.position.y) / (*conveyor_speed));

    // check if the object will be unreachable by the time we get there
    ROS_INFO("Picking at y: %f ", grasp_pose_goal->pose.position.y);
    if (grasp_pose_goal->pose.position.y < mach->min_workspace_y) {
        // TODO: add this to criteria for selecting object
        ROS_INFO(" -> Sorry! that was too fast");
        return false;
    }

    // TRANSFORM tool frame goal to wrist frame goal
    // grasp_pose_goal->pose.position.y -= 0.18; 
    return true;
}

#if 0
// TODO:
// select an object which is untargeted by another machine, we have a good
// estimate of its position and velocity, and we're able to grasp it
bool TryPickObject(PickMachine* mach, bool left)
{
    ROS_INFO("Try to pick object");

    geometry_msgs::PoseStamped object_pose;
    geometry_msgs::Vector3Stamped object_vel;
    bool found = false;

    LockWorldState(&g_world_state);
    // select an object
    for (auto& object : g_world_state.objects) {
        // skip claimed objects
        if (object.claimed){
            ROS_INFO(" -> Skip claimed object '%u'", object.id);
            continue;
        }

        geometry_msgs::Vector3Stamped vel;
        // have a good velocity estimate
        if (EstimateObjectVelocity(&object, "base_footprint", 1.0, 2/*6*/, &vel)) {
            ROS_INFO("  Select object %u", object.id);
            auto& pose = object.pose_estimates.back().pose;
            ROS_INFO("  Transform pose from '%s' to 'base_footprint'", pose.header.frame_id.c_str());
            TryHardTransformPose("base_footprint", pose, object_pose);
            ROS_INFO("  Transform velocity from '%s' to 'base_footprint'", vel.header.frame_id.c_str());
            TryHardTransformVector("base_footprint", vel, object_vel);
            found = true;
            break;
        }
    }
    UnlockWorldState(&g_world_state);

    if (!found) {
        ROS_INFO("  No reasonable estimates of object velocity");
        return false; // no object with a reasonable estimate
    }

    // OLD FUNCTIONALITY: get 6 samples, evenly spaced every 0.2 seconds and
    // compute the average velocity at any point in the path

    //////////////////////////////////////////////
    // Estimate the conveyor speed from samples //
    //////////////////////////////////////////////

    double conveyor_speed = object_vel.vector.y;
    ROS_INFO("Velocity of object is: (%f, %f, %f)", object_vel.vector.x, object_vel.vector.y, object_vel.vector.z);

    ////////////////////////////
    // Construct gripper goal //
    ////////////////////////////

    ROS_INFO("Construct gripper goal");
    ROS_INFO("  Object position = (%f, %f, %f)", object_pose.pose.position.x, object_pose.pose.position.y, object_pose.pose.position.z);

    // how far the object will have moved during the time it takes to close the
    // gripper
    auto g_grasp_offset = g_time_to_reach_grasp * conveyor_speed;
    ROS_INFO("Pick at the offset: %f ", g_grasp_offset);

    geometry_msgs::PoseStamped grasp_pose_goal_local;

    grasp_pose_goal_local.header.frame_id = "base_footprint";

    // final pose of the object after speed estimation + minor adjustment
    grasp_pose_goal_local.pose.position.x = object_pose.pose.position.x; // + 0.01;

    // final pose of object after speed estimation + distance object will travel
    // during arm execution
    grasp_pose_goal_local.pose.position.y = object_pose.pose.position.y + g_grasp_offset;

    // hardcoded :)
    grasp_pose_goal_local.pose.position.z = 0.8; //+= 0.05;
    // grasp_pose_goal_local.pose.orientation.x = grasp_pose_goal_local.pose.orientation.y = 0.0;
    // grasp_pose_goal_local.pose.orientation.z = grasp_pose_goal_local.pose.orientation.w = 0.5 * sqrt(2.0);
    grasp_pose_goal_local.pose.orientation.x = 0.1472033;
    grasp_pose_goal_local.pose.orientation.y = 0.2944066;
    grasp_pose_goal_local.pose.orientation.z = 0.0;
    grasp_pose_goal_local.pose.orientation.w = 0.9442754;

    // limit the grasp goal to be reachable (not too far ahead of the robot)
    if (!left) {
        grasp_pose_goal_local.pose.position.y = std::min(grasp_pose_goal_local.pose.position.y, -0.4);
    } else {
        grasp_pose_goal_local.pose.position.y = std::min(grasp_pose_goal_local.pose.position.y, 0.35);
    }

    // check if the object will be unreachable by the time we get there
    ROS_INFO("Picking at x: %f ", grasp_pose_goal_local.pose.position.x);
    ROS_INFO("Picking at y: %f ", grasp_pose_goal_local.pose.position.y);
    ROS_INFO("Picking at z: %f ", grasp_pose_goal_local.pose.position.z);
    if (!left) {
        if (grasp_pose_goal_local.pose.position.y < -0.6) {
            // TODO: add this to criteria for selecting object
            ROS_INFO(" -> Sorry! that was too fast");
            return false;
        }
    } else {
        if (grasp_pose_goal_local.pose.position.y < 0.05) {
            ROS_INFO(" -> Sorry! that was too fast");
            return false;
        }
    }
    // grasp_pose_goal_local.pose.position.y = std::max(grasp_pose_goal_local.pose.position.y, -0.35);

    return SendGoal(mach, grasp_pose_goal_local, conveyor_speed);
}
#endif

bool TryPickObject(PickMachine* mach, bool lock = true)
{
    ROS_INFO("Try to pick object");

    geometry_msgs::PoseStamped object_pose;
    geometry_msgs::Vector3Stamped object_vel;
    uint32_t claimed_id = 0;
    bool found = false;

    if (lock) LockWorldState(&g_world_state);
    // select an object
    for (auto& object : g_world_state.objects) {
        // skip claimed objects
        if (object.claimed) {
            ROS_INFO(" -> Skip claimed object '%u'", object.id);
            continue;
        }

        auto now = ros::Time::now();

        geometry_msgs::Vector3Stamped vel;
        // have a good velocity estimate
        if (EstimateObjectVelocity(now, &object, g_robot_frame, 1.0, 2/*6*/, &vel)) {
            ROS_INFO("  Select object %u", object.id);
            auto& pose = object.pose_estimates.back().pose;
            ROS_INFO("  Transform pose from '%s' to '%s'", pose.header.frame_id.c_str(), g_robot_frame);
            TryHardTransformPose(g_robot_frame, pose, object_pose);
            ROS_INFO("  Transform velocity from '%s' to '%s'", vel.header.frame_id.c_str(), g_robot_frame);
            TryHardTransformVector(g_robot_frame, vel, object_vel);
            object.claimed = true;
            claimed_id = object.id;
            found = true;
            break;
        }
    }
    if (lock) UnlockWorldState(&g_world_state);

    if (!found) {
        ROS_INFO("  No reasonable estimates of object velocity");
        return false; // no object with a reasonable estimate
    }

    auto unclaim_object = [&]()
    {
        if (lock) LockWorldState(&g_world_state);
        for (auto& object : g_world_state.objects) {
            if (object.id == claimed_id) {
                object.claimed = false;
                break;
            }
        }
        if (lock) UnlockWorldState(&g_world_state);
    };

    geometry_msgs::PoseStamped grasp_pose_goal_local;
    double conveyor_speed;
    double time_to_grasp;
    if (!ComputeGraspGoal(mach, object_pose, object_vel, &grasp_pose_goal_local, &conveyor_speed, &time_to_grasp)) {
        unclaim_object();
        return false;
    }

    if (!SendGoal(mach, grasp_pose_goal_local, conveyor_speed, time_to_grasp, claimed_id)) {
        unclaim_object();
        return false;
    } else {
        return true;
    }
}

PickState DoPlanPickup(PickMachine* mach)
{
    ROS_INFO("Plan link '%s' to grasp pose", mach->move_group->getEndEffectorLink().c_str());
    WaitForMoveGroup();

    g_move_group_busy = true;
    mach->move_group->setPoseTarget(mach->grasp_pose_goal.pose);
    auto err = mach->move_group->plan(mach->pickup_plan);
    g_move_group_busy = false;

    if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_ERROR("Failed to plan to grasp pose");
        mach->goal_ready = false;
        return PickState::WaitForGoal;
    }

    return PickState::ExecutePickup;
}

PickState DoExecutePickup(PickMachine* mach)
{
    // control_msgs::FollowJointTrajectoryGoal goal;
    // goal.trajectory = mach->pickup_plan.trajectory_.joint_trajectory;
    // auto state = mach->follow_joint_trajectory_client->sendGoalAndWait(goal);
    // if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
    //     ROS_ERROR("maybe should move to home?!");
    // }

    g_move_group_busy = true;
    auto err = mach->move_group->execute(mach->pickup_plan);
    g_move_group_busy = false;

    if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_ERROR("Failed to move arm to grasp pose");
        mach->goal_ready = false;
        return PickState::WaitForGoal;
    }

    return PickState::GraspObject;
}

#if 0
PickState DoMoveToGrasp(PickMachine* mach)
{
    ROS_INFO("Move link '%s' to grasp pose", mach->move_group->getEndEffectorLink().c_str());

    // geometry_msgs::Pose tip_pose;
    // tip_pose.orientation.x = 0.1472033;
    // tip_pose.orientation.y = 0.2944066;
    // tip_pose.orientation.z = 0.0;
    // tip_pose.orientation.w = 0.9442754;
    // tip_pose.position.x = 0.25;
    // tip_pose.position.y = -0.4;
    // tip_pose.position.z = 0.8;

    // g_move_group->setPoseTarget(tip_pose);
    // std::string pose_reference_frame = "base_footprint";
    // g_move_group->setPoseReferenceFrame(pose_reference_frame);

    mach->move_group->setPoseTarget(mach->grasp_pose_goal.pose);
    auto err = mach->move_group->move();
    if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_ERROR("Failed to move arm to grasp pose");
        mach->goal_ready = false;
        return PickState::WaitForGoal;
    } 
    
    ROS_INFO("Successfully move arm to grasp pose!");
    return PickState::GraspObject;
}
#endif

PickState DoGraspObject(PickMachine* mach)
{
    double buffer_time = 0.05 / std::fabs(mach->conveyor_speed);

    ROS_INFO("Wait an additional %f", buffer_time);

    // how long it actually took to plan and execute the arm motion
    ros::Duration execution_duration = ros::Time::now() - mach->time_at_execute;

    double wait_time = std::max(/*g_time_to_reach_grasp*/ mach->time_to_reach_grasp - execution_duration.toSec(), 0.0);
    wait_time += buffer_time;

    if (wait_time < 0.0) {
        ROS_WARN("It actually took us %f seconds to reach the grasp pose", execution_duration.toSec());
        wait_time = std::max(wait_time, 0.0);
    }

    wait_time = std::min(wait_time, 10.0); // hurp durp, take this from conveyor length / conveyor_speed

    ROS_INFO("Waiting before grasp for %f secs", wait_time);
    ros::Duration(wait_time).sleep();

    return PickState::CloseGripper;
}

PickState DoCloseGripper(PickMachine* mach)
{
    return PickState::PlanDropoff;
}

PickState DoPlanDropoff(PickMachine* mach)
{
    WaitForMoveGroup();
    g_move_group_busy = true;

    auto v = mach->dropoff_position;
    for (auto& value : v) {
        value *= M_PI / 180.0;
    }

    mach->move_group->setJointValueTarget(v);

    auto err = mach->move_group->plan(mach->dropoff_plan);
    if (err != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        // You got us in here, did you have a plan for getting out?
        // we should literally execute the pickup plan in reverse ...probably...
        ROS_ERROR("PRETTY BAD! GOING NOWHERE!");
    }

    g_move_group_busy = false;

    return PickState::ExecuteDropoff;
}

PickState DoExecuteDropoff(PickMachine* mach)
{
    // control_msgs::FollowJointTrajectoryGoal goal;
    // goal.trajectory = mach->dropoff_plan.trajectory_.joint_trajectory;
    // auto state = mach->follow_joint_trajectory_client->sendGoalAndWait(goal);
    // if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
    //     ROS_ERROR("ALSO PRETTY BAD!");
    // }
    g_move_group_busy = true;
    auto err = mach->move_group->execute(mach->dropoff_plan);
    g_move_group_busy = false;

    if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_ERROR("Failed to move arm to dropoff pose");
        return PickState::WaitForGoal;
    }

    return PickState::OpenGripper;
}

PickState DoOpenGripper(PickMachine* mach)
{
    mach->goal_ready = false;
    return PickState ::MoveToHome;
}

PickState DoMoveToHome(PickMachine* mach)
{
    WaitForMoveGroup();
    g_move_group_busy = true;

    auto v = mach->home_position;
    // for (auto& value : v) {
    //     value *= M_PI / 180.0;
    // }

    mach->move_group->setJointValueTarget(v);

    auto err = mach->move_group->move();

    // MoveGroup::Plan home_plan;
    // auto err = mach->move_group->plan(home_plan);
    if (err != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        // You got us in here, did you have a plan for getting out?
        // we should literally execute the pickup plan in reverse ...probably...
        ROS_ERROR("PRETTY BAD! GOING NOWHERE!");
    }

    // auto err = mach->move_group->execute(plan);

    // if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    //     ROS_ERROR("Failed to move arm to home position");
    //     mach->goal_ready = false;
    //     return PickState::WaitForGoal;
    // }

    g_move_group_busy = false;

    // control_msgs::FollowJointTrajectoryGoal goal;
    // goal.trajectory = home_plan.trajectory_.joint_trajectory;
    // auto state = mach->follow_joint_trajectory_client->sendGoalAndWait(goal);
    // if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
    //     ROS_ERROR("ALSO PRETTY BAD!");
    // }

    return PickState::WaitForGoal;
}

auto MakeConveyorCollisionObject() -> moveit_msgs::CollisionObject
{
     moveit_msgs::CollisionObject conveyor;

    double height = 0.7; //0.64 + ADJUST;

    geometry_msgs::PoseStamped p;
    p.header.frame_id = g_robot_frame;
    p.header.stamp = ros::Time(0); //ros::Time::now();
    p.pose.position.x = 0.25;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.5 * height;
    p.pose.orientation.w = 1.0;

    geometry_msgs::PoseStamped p_out;
    ROS_INFO("Transform conveyor object to planning frame");
    TryHardTransformPose(g_planning_frame, p, p_out);
    ROS_INFO("...done");

    conveyor.header.frame_id = g_planning_frame;
    conveyor.header.stamp = ros::Time::now();

    conveyor.id = "conveyor";

    shape_msgs::SolidPrimitive conveyor_shape;
    conveyor_shape.type = shape_msgs::SolidPrimitive::BOX;
    conveyor_shape.dimensions.resize(3);
    conveyor_shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.26;
    conveyor_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.14;
    conveyor_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height;
    conveyor.primitives.push_back(conveyor_shape);

    geometry_msgs::Pose conveyor_pose;
    conveyor_pose.position.x = p_out.pose.position.x;
    conveyor_pose.position.y = p_out.pose.position.y;
    conveyor_pose.position.z = p_out.pose.position.z;
    conveyor_pose.orientation.w = p_out.pose.orientation.w;
    conveyor_pose.orientation.x = p_out.pose.orientation.x;
    conveyor_pose.orientation.y = p_out.pose.orientation.y;
    conveyor_pose.orientation.z = p_out.pose.orientation.z;
    conveyor.primitive_poses.push_back(conveyor_pose);

    conveyor.operation = moveit_msgs::CollisionObject::ADD;

    return conveyor;
}

void RunStateMachine(PickMachine* mach)
{
    while (ros::ok()) {
        if (mach->prev_state != mach->curr_state) {
            ROS_INFO("Enter state '%s' -> '%s'", to_cstring(mach->prev_state), to_cstring(mach->curr_state));
            if (mach->states[(int)mach->curr_state].enter) {
                mach->states[(int)mach->curr_state].enter(mach, mach->prev_state);
            }
            mach->prev_state = mach->curr_state;
        }

        // assert(mach->states[(int)mach->curr_state] != NULL);
        mach->next_state = mach->states[(int)mach->curr_state].pump(mach);
        if (mach->next_state == PickState::Finished) {
            break;
        }

        if (mach->next_state != mach->curr_state) {
            ROS_INFO("Exit state '%s' -> '%s'", to_cstring(mach->curr_state), to_cstring(mach->next_state));
            if (mach->states[(int)mach->curr_state].exit) {
                mach->states[(int)mach->curr_state].exit(mach, mach->curr_state);
            }
            mach->curr_state = mach->next_state;
        }
    }
}

// 0. TODO
//
// ...move both arms simultaneously
// ...single-shot conveyor velocity estimation
// ...better policy for choosing which object to grasp
// .....quick kinematics checks
// .....quick planning queries
// .....choose best of available objects
// ...planning with time to choose earliest time
//
// 1. new
//
// pose = localize_conveyor()
// open_grippers()
//
// loop:
//   // find an object to attempt picking
//   // considerations:
//   // * do we have a free arm?
//   // * is the object being attempted by the other arm?
//   // * is the other arm in the way
//   select untargeted object
//
// 2. old
//
// pose = localize_conveyor()
// open_gripper()
// loop:
//   o, pose = find_object
//   grasps = plan_grasps(o, pose)
//   traj = plan_arm_motion(grasps)
//   move_arm(traj)
//   wait_for_grasp()
//   close_gripper()
//   traj = plan_arm_motion(dropoff)
//   open_gripper()

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "walker_simulation");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    // std::string l_arm_action_name;
    // std::string r_arm_action_name;
    // ph.param<std::string>("l_arm_action_name", l_arm_action_name, "l_arm_controller");
    // ph.param<std::string>("r_arm_action_name", r_arm_action_name, "r_arm_controller");

    double allowed_planning_time;
    ph.param("allowed_planning_time", allowed_planning_time, 30.0);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    g_broadcaster.reset(new tf::TransformBroadcaster);
    g_listener.reset(new tf::TransformListener);

    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // planning_scene_interface.applyCollisionObject(MakeConveyorCollisionObject());

    ros::Subscriber pose_sub = nh.subscribe("ar_pose_marker", 1000, ARPoseCallback);
    fillGraspPoses();

    PickMachState states[(int)PickState::Count];
    states[(int)PickState::PrepareGripper].pump = DoPrepareGripper;
    states[(int)PickState::WaitForGoal].pump = DoWaitForGoal;
    states[(int)PickState::PlanPickup].pump = DoPlanPickup;
    states[(int)PickState::ExecutePickup].pump = DoExecutePickup;
    states[(int)PickState::GraspObject].pump = DoGraspObject;
    states[(int)PickState::CloseGripper].pump = DoCloseGripper;
    states[(int)PickState::PlanDropoff].pump = DoPlanDropoff;
    states[(int)PickState::ExecuteDropoff].pump = DoExecuteDropoff;
    states[(int)PickState::OpenGripper].pump = DoOpenGripper;
    states[(int)PickState::MoveToHome].pump = DoMoveToHome;

//     ROS_INFO("Initialize left picking machine");
//     PickMachine left_machine;
//     left_machine.prev_state = PickState::PrepareGripper;
//     left_machine.curr_state = PickState::PrepareGripper;
//     left_machine.next_state = PickState::PrepareGripper;
//     left_machine.goal_ready = false;
//     left_machine.move_group.reset(new MoveGroup(
//                 "left_arm", boost::shared_ptr<tf::Transformer>(), ros::WallDuration(25.0)));
//     left_machine.move_group->setPlanningTime(allowed_planning_time);
//     left_machine.move_group->setPlannerId("left_arm[arastar_bfs_manip]");
//     left_machine.move_group->setWorkspace(-0.4, -1.2, 0.0, 1.10, 1.2, 2.0);
//     left_machine.move_group->startStateMonitor();
//     left_machine.min_workspace_y = 0.05;
//     left_machine.max_workspace_y = 0.4;
//     left_machine.home_position = {
// //        64.72, 5.16, 160.66, -89.70, 106.84, -110.93, 1.00
//         49.38, 6.75, 167.66, -104.50, 137.01, -114.57, 0.0
//     };
//     left_machine.dropoff_position = {
// //        86.99, 20.40, 73.15, -110.59, 141.89, -28.94, 0.0
//         94.72, 5.16, 160.66, -89.70, 106.84, -110.93, 1.00
//     };

//     ROS_INFO("Create Left Gripper Command Action Client");
//     left_machine.gripper_command_client.reset(new GripperCommandActionClient(
//                 "l_gripper_controller/gripper_action"));
//     if (!left_machine.gripper_command_client->waitForServer(ros::Duration(10.0))) {
//         ROS_ERROR("Gripper Action client not available");
//         return 1;
//     }

//     ROS_INFO("Create Left Arm Follow Joint Trajectory Action Client");
//     left_machine.follow_joint_trajectory_client.reset(new FollowJointTrajectoryActionClient(
//                 l_arm_action_name + "/follow_joint_trajectory"));
//     if (!left_machine.follow_joint_trajectory_client->waitForServer(ros::Duration(10.0))) {
//         ROS_ERROR("Follow Joint Trajectory client not available");
//         return 1;
//     }
//     left_machine.states = states;

    ROS_INFO("Initialize right picking machine");
    PickMachine right_machine;
    right_machine.prev_state = PickState::PrepareGripper;
    right_machine.curr_state = PickState::PrepareGripper;
    right_machine.next_state = PickState::PrepareGripper;
    right_machine.goal_ready = false;
    right_machine.move_group.reset(new MoveGroup(
                "right_arm", boost::shared_ptr<tf::Transformer>(), ros::WallDuration(25.0)));
    // right_machine.gripper_command_client.reset(new GripperCommandActionClient(
    //             "r_gripper_controller/gripper_action"));
    right_machine.move_group->setPlanningTime(allowed_planning_time);
    right_machine.move_group->setPlannerId("right_arm[arastar_bfs_manip]");
    right_machine.move_group->setWorkspace(-0.4, -1.2, 0.0, 1.10, 1.2, 2.0);
    right_machine.move_group->startStateMonitor();
    right_machine.min_workspace_y = -0.50;
    right_machine.max_workspace_y = -0.4;
    right_machine.home_position = {
        // -79.38, 15.53, -68.79, -95.13, 359.0, -66.94, 79.95
        -0.736, -1.052, 0.243, -0.807, 0.2405, 0.017, 0.133
    };
    // right_machine.dropoff_position = {
    //     //-94.13, 19.62, -68.78, -102.20, 359.0, -114.55, 359.00
    //     -79.38, 15.53, -68.79, -95.13, 359.0, -66.94, 79.95
    // };

    // ROS_INFO("Create Right Gripper Command Action Client");
    // right_machine.gripper_command_client.reset(new GripperCommandActionClient(
    //             "r_gripper_controller/gripper_action"));
    // if (!right_machine.gripper_command_client->waitForServer(ros::Duration(10.0))) {
    //     ROS_ERROR("Gripper Action Client not available");
    //     return 1;
    // }

    // ROS_INFO("Create Right Arm Follow Joint Trajectory Action Client");
    // right_machine.follow_joint_trajectory_client.reset(new FollowJointTrajectoryActionClient(
    //             r_arm_action_name + "/follow_joint_trajectory"));
    // if (!right_machine.follow_joint_trajectory_client->waitForServer(ros::Duration(10.0))) {
    //     ROS_ERROR("Follow Joint Trajectory client not available");
    //     return 1;
    // }
    right_machine.states = states;

#if 0
    auto l_mach_thread = std::thread(RunStateMachine, &left_machine);
    auto r_mach_thread = std::thread(RunStateMachine, &right_machine);

    DoLocalizeConveyor();

    ros::Rate loop_rate(1.0);
    while (ros::ok) {
        // update objects

        // preconditions
        //    some machine is waiting for a goal
        //    all other machines waiting for goals or moving to dropoff
        PickMachine* idle_machine = NULL;
        PickMachine* other_machine = NULL;
        if (right_machine.curr_state == PickState::WaitForGoal) {
            idle_machine = &right_machine;
            other_machine = &left_machine;
        } else if (left_machine.curr_state == PickState::WaitForGoal) {
            idle_machine = &left_machine;
            other_machine = &right_machine;
        }

        if (idle_machine != NULL &&
            (
                other_machine->curr_state == PickState::WaitForGoal ||
                other_machine->curr_state == PickState::ExecuteDropoff ||
                other_machine->curr_state == PickState::OpenGripper
            ))
        {
            // select a target object
            TryPickObject(idle_machine, idle_machine == &left_machine);
        }

        loop_rate.sleep();
    }

    ros::waitForShutdown();

    l_mach_thread.join();
    r_mach_thread.join();
#endif

    // only enable right arm 
    auto r_mach_thread = std::thread(RunStateMachine, &right_machine);

    DoLocalizeConveyor();
    // ResetArms(&left_machine, &right_machine);
    ResetArm(&right_machine);

    ros::Rate loop_rate(1.0);   
    while (ros::ok) {
        if (right_machine.goal_ready){
            tf::StampedTransform transform;
            transform.frame_id_ = "base_footprint";
            transform.child_frame_id_ = "predicted_pose";
            tf::poseMsgToTF(right_machine.grasp_pose_goal.pose, transform);
            g_broadcaster->sendTransform(transform);
        }
        if (right_machine.curr_state == PickState::WaitForGoal){
            TryPickObject(&right_machine, false);
        }
        loop_rate.sleep();
    }

    ros::waitForShutdown();
    r_mach_thread.join();
    return 0;
}