#include <atomic>
#include <ros/ros.h>
#include <mutex>
#include <thread>

// #include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <walker_simulation/Path1.h>
#include <walker_simulation/GraspPose.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using moveit::planning_interface::MoveGroupInterface;

// WaitForGoal -> Finished
// WaitForGoal -> PlanToGoal
// PlanToGoal -> PublishPath
// PublishPath -> WaitForGoal

enum struct PickState
{
    Finished = -1,
    WaitForGoal = 0,
    PlanToGoal,
    PublishPath,
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

	std::unique_ptr<MoveGroupInterface> move_group;

    std::atomic<bool> goal_ready;

	std::vector<double> dropoff_position;
	std::vector<double> home_position;

	MoveGroupInterface::Plan previous_plan;
	MoveGroupInterface::Plan current_plan;

	geometry_msgs::PoseStamped grasp_pose_goal;
};

struct WorldState
{
    std::mutex grasp_mutex;
    std::vector<geometry_msgs::PoseStamped> grasp_pose_buffer;
};

// global variables
ros::Publisher g_path_publisher;
std::unique_ptr<tf::TransformBroadcaster> g_broadcaster;
std::unique_ptr<tf::TransformListener> g_listener;
const char *g_planning_frame = "odom_combined"; // TODO: take this from the MoveGroupInterface
const char *g_robot_frame = "base_footprint";
std::atomic<bool> g_move_group_busy(false);
WorldState g_world_state;
std::vector<geometry_msgs::PoseStamped> g_grasp_pose_buffer;

void LockWorldState(WorldState* state)
{
    state->grasp_mutex.lock();
}

void UnlockWorldState(WorldState* state)
{
    state->grasp_mutex.unlock();
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

void WaitForMoveGroup()
{
    while (g_move_group_busy) { ros::Duration(0.02).sleep(); }
}

void GraspPoseCallback(const geometry_msgs::PoseStamped& grasp)
{
	g_grasp_pose_buffer.push_back(grasp);
    ROS_INFO("buffer size: %d", g_grasp_pose_buffer.size());

}

PickState DoWaitForGoal(PickMachine* mach)
{
    while (ros::ok()) {
        if (mach->goal_ready) {
            mach->goal_ready = false; // consume goal
            return PickState::PlanToGoal;
        }
        ros::Duration(1.0).sleep();
    }

    return PickState::Finished;
}

bool PlanFromCurrentState(PickMachine *mach)
{
	g_move_group_busy = true;
	mach->move_group->setStartStateToCurrentState();
    mach->move_group->setPoseTarget(mach->grasp_pose_goal.pose);
    auto err = mach->move_group->plan(mach->current_plan);
    g_move_group_busy = false;

    if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_ERROR("Failed to plan to pose (from current robot state)!");
		return false;
    }
	return true;
}

/* 
struct Plan{
	moveit_msgs::RobotState start_state_;
	moveit_msgs::RobotTrajectory trajectory_;
	double planning_time_;
};*/

void GetLastRobotState(PickMachine *mach, moveit_msgs::RobotState& state)
{
	state.joint_state.name = mach->previous_plan.trajectory_.joint_trajectory.joint_names;
	int joint_traj_len = mach->previous_plan.trajectory_.joint_trajectory.points.size();
	state.joint_state.position = mach->previous_plan.trajectory_.joint_trajectory.points[joint_traj_len-1].positions;
}

bool PlanFromLastState(PickMachine *mach)
{
	moveit_msgs::RobotState last_robot_state;
	GetLastRobotState(mach, last_robot_state);

	g_move_group_busy = true;
	mach->move_group->setStartState(last_robot_state);
    mach->move_group->setPoseTarget(mach->grasp_pose_goal.pose);
    auto err = mach->move_group->plan(mach->current_plan);
    g_move_group_busy = false;

    if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_ERROR("Failed to plan to pose (from last planned state)! ");
		return false;
    }
	return true;
}

PickState DoPlanToGoal(PickMachine* mach)
{
	bool plan_succeed;
	plan_succeed = PlanFromCurrentState(mach);

    g_grasp_pose_buffer.erase(g_grasp_pose_buffer.begin());
	ros::param::set("/walker_planner_done", 1);

	if (plan_succeed) {
        ROS_WARN("Successfully planned to goal pose!");
        mach->previous_plan = mach->current_plan;
		return PickState::PublishPath;
	} else {
		mach->current_plan = mach->previous_plan;
        return PickState::WaitForGoal;
	}
}

PickState DoPublishPath(PickMachine* mach)
{
    int iteration = 0; 
    int iterations = 1000;
    ros::Rate r(10);
    while (ros::ok && iteration < iterations) {
        walker_simulation::Path1 path;
        for (int i = 0; i < mach->current_plan.trajectory_.joint_trajectory.points.size(); i++) {
            walker_simulation::GraspPose robot_state;
            for (int j = 0; j < mach->current_plan.trajectory_.joint_trajectory.points[i].positions.size(); j++) {
                robot_state.a.push_back(mach->current_plan.trajectory_.joint_trajectory.points[i].positions[j]);
            }
            path.path.push_back(robot_state);
        }
        iteration++;
        g_path_publisher.publish(path);
        r.sleep();
    }
    return PickState::WaitForGoal;
}

auto MakeConveyorCollisionObject() -> moveit_msgs::CollisionObject
{
     moveit_msgs::CollisionObject conveyor;

    double height = 0.70; //0.64 + ADJUST;

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

auto to_cstring(PickState state) -> const char*
{
    switch (state) {
    case PickState::WaitForGoal:        return "WaitForGoal";
    case PickState::PlanToGoal:         return "PlanToGoal";
    case PickState::PublishPath:        return "PublishPath";
    default:                            return "<Unknown>";
    }
};

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

bool PlanForNextGoal(PickMachine* mach)
{
    ROS_INFO("Trying to plan to next goal...");
    
    if (g_grasp_pose_buffer.empty()) {
		ROS_WARN("No grasp goals available!");
		return false;
	}

    if (mach->goal_ready) return false; // we're busy

    int req;
    ros::param::get("/walker_planner_request", req);
    if (req)
    {
        ROS_INFO("Planning request received!");
        ros::param::set("/walker_planner_request", 0);
        WaitForMoveGroup();
        mach->grasp_pose_goal = g_grasp_pose_buffer[0];
        mach->goal_ready = true;
        return true;
    }
    return false;
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "multi_step_planner");
	ros::NodeHandle nh;
	ros::NodeHandle ph("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

	double allowed_planning_time;
	ph.param("allowed_planning_time", allowed_planning_time, 3.0);
    ros::param::set("/walker_planner_request", 0);
    ros::param::set("/walker_planner_done", 1);
    ros::param::set("/from_current_state", 1);

	g_broadcaster.reset(new tf::TransformBroadcaster);
    g_listener.reset(new tf::TransformListener);

	ros::Subscriber pose_sub = nh.subscribe("/Grasp", 1000, GraspPoseCallback);
	g_path_publisher = nh.advertise<walker_simulation::Path1>("Robot_path", 1000);

    // WaitForGoal -> Finished
    // WaitForGoal -> PlanToGoal
    // PlanToGoal -> PublishPath
    // PublishPath -> WaitForGoal

    PickMachState states[(int)PickState::Count];
    states[(int)PickState::WaitForGoal].pump = DoWaitForGoal;
    states[(int)PickState::PlanToGoal].pump = DoPlanToGoal;
    states[(int)PickState::PublishPath].pump = DoPublishPath;

	ROS_INFO("Initialize right picking machine");
    PickMachine right_machine;
    right_machine.prev_state = PickState::WaitForGoal;
    right_machine.curr_state = PickState::WaitForGoal;
    right_machine.next_state = PickState::WaitForGoal;
    right_machine.goal_ready = false;
    right_machine.move_group.reset(new MoveGroupInterface(
                "right_arm", boost::shared_ptr<tf::Transformer>(), ros::WallDuration(25.0)));
    right_machine.move_group->setPlanningTime(allowed_planning_time);
    right_machine.move_group->setPlannerId("right_arm[arastar_bfs_manip]");
    right_machine.move_group->setWorkspace(-0.4, -1.2, 0.0, 1.10, 1.2, 2.0);
    right_machine.move_group->startStateMonitor();
    right_machine.states = states;

	// moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	// planning_scene_interface.applyCollisionObject(MakeConveyorCollisionObject());

    auto r_mach_thread = std::thread(RunStateMachine, &right_machine);

    ros::Rate loop_rate(10);
	while (ros::ok())
	{
        if (right_machine.curr_state == PickState::WaitForGoal){
            PlanForNextGoal(&right_machine);
        }
        loop_rate.sleep();
	}

    ros::waitForShutdown();
    r_mach_thread.join();

	return 0;
}