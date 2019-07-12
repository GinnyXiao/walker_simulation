#include <assert.h>
#include <ros/ros.h>
#include <memory>
#include <unordered_map>

#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using moveit::planning_interface::MoveGroup;

const char* g_planning_group = "right_arm";
std::unique_ptr<MoveGroup> g_move_group;
std::unordered_map<int, geometry_msgs::PoseStamped> g_id_to_pose;
ar_track_alvar_msgs::AlvarMarkers g_markers;
geometry_msgs::PoseStamped g_grasp_pose;

void fillGraspPoses()
{
    // hardcoded grasp poses defined in marker frames
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "ar_marker_6";
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.0;
    p.pose.orientation.w = 1.0;
    p.pose.position.x = 0.0;
    p.pose.position.y = 0.0;
    p.pose.position.z = 0.0;
    g_id_to_pose.insert(std::pair<int, geometry_msgs::PoseStamped> (6, p));
}

void ARPoseCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    g_markers = msg;
}

enum struct State {
    LocalizeConveyor = 0,
    // PrepareGripper,
    FindObject,
    ExecutePickup,
    GraspObject,
    // CloseGripper,
    ExecuteDropoff,
    // OpenGripper,
    Count
};

auto to_cstring(State state) -> const char*
{
    switch (state) {
    case State::LocalizeConveyor:   return "LocalizeConveyor";
    // case State::PrepareGripper:     return "PrepareGripper";
    case State::FindObject:         return "FindObject";
    case State::ExecutePickup:      return "ExecutePickup";
    case State::GraspObject:        return "GraspObject";
    // case State::CloseGripper:       return "CloseGripper";
    case State::ExecuteDropoff:     return "ExecuteDropoff";
    // case State::OpenGripper:        return "OpenGripper";
    default:                        return "<Unknown>";
    }
};

struct MachState {
    using EnterFn = void (*)(State);
    using PumpFn = State (*)();
    using ExitFn = void (*)(State);

    EnterFn enter = NULL;
    PumpFn pump = NULL;
    ExitFn exit = NULL;
};

State DoLocalizeConveyor()
{
    // return State::PrepareGripper;
    return State::FindObject;
}

// State DoPrepareGripper()
// {
//     return State::FindObject;
// }

State DoFindObject()
{
    // tf::TransformListener listener;

    // assuming we see only one object at a time
    // auto marker = g_markers.markers[0];
    // int id = marker.id;
    // auto pose = marker.pose;
    // std::string marker_frame = "ar_marker_" + std::to_string(id);

    // auto it = g_id_to_pose.find(id); //geometry_msgs::PoseStamped
    // if (it == g_id_to_pose.end()) {
    //     ROS_ERROR("Could not find object in the library");
    //     return State::FindObject;
    // }

    // auto g_tf = it->second;
    // listener.waitForTransform("base_footprint", marker_frame,
    //                           ros::Time(0), ros::Duration(10.0));
    // listener.transformPose(marker_frame, g_tf, g_grasp_pose);
    return State::ExecutePickup;
}

State DoMoveToGrasp()
{
    ROS_INFO("Move link '%s' to grasp pose", g_move_group->getEndEffectorLink().c_str());
#if 1
    geometry_msgs::Pose tip_pose;
    tip_pose.orientation.x = 0.1472033;
    tip_pose.orientation.y = 0.2944066;
    tip_pose.orientation.z = 0.0;
    tip_pose.orientation.w = 0.9442754;
    tip_pose.position.x = 0.25;
    tip_pose.position.y = -0.4;
    tip_pose.position.z = 0.8;

    g_move_group->setPoseTarget(tip_pose);
    // g_move_group->setPoseTarget(g_grasp_pose.pose);
    std::string pose_reference_frame = "base_footprint";
    g_move_group->setPoseReferenceFrame(pose_reference_frame);

    auto err = g_move_group->move();
    if (err.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_ERROR("Failed to move arm to grasp pose");
    }
#endif
    return State::GraspObject;
}

State DoGraspObject()
{
    // return State::CloseGripper;
    return State::ExecuteDropoff;
}

// State DoCloseGripper()
// {
//     return State::ExecuteDropoff;
// }

State DoExecuteDropoff()
{
    return State::FindObject;
}

// State DoOpenGripper()
// {
//     return State::FindObject;
// }

auto MakeConveyorCollisionObject() -> moveit_msgs::CollisionObject
{
    moveit_msgs::CollisionObject conveyor;
    conveyor.header.frame_id = "base_footprint";
    conveyor.header.stamp = ros::Time::now();

    conveyor.id = "conveyor";

    double height = 0.64;

    shape_msgs::SolidPrimitive conveyor_shape;
    conveyor_shape.type = shape_msgs::SolidPrimitive::BOX;
    conveyor_shape.dimensions.resize(3);
    conveyor_shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.26;
    conveyor_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.14;
    conveyor_shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height;
    conveyor.primitives.push_back(conveyor_shape);

    geometry_msgs::Pose conveyor_pose;
    conveyor_pose.position.x = 0.60; //0.62;
    conveyor_pose.position.y = 0.0;
    conveyor_pose.position.z = 0.5 * height;
    conveyor_pose.orientation.w = 1.0;
    conveyor.primitive_poses.push_back(conveyor_pose);

    conveyor.operation = moveit_msgs::CollisionObject::ADD;

    return conveyor;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "walker_simulation");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Subscriber pose_sub = nh.subscribe("ar_pose_marker", 1000, ARPoseCallback);
    fillGraspPoses();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(MakeConveyorCollisionObject());

    ROS_INFO("Create Move Group");
//    MoveGroup::Options ops(g_planning_group);
    g_move_group.reset(new MoveGroup(g_planning_group, 
        boost::shared_ptr<tf::Transformer>(), ros::WallDuration(5.0)));

    g_move_group->setPlanningTime(30.0);
    g_move_group->setPlannerId("right_arm[arastar_bfs_manip]");
    g_move_group->setWorkspace(-0.4, -1.2, 0.0, 1.10, 1.2, 2.0); // change this later maybe

    MachState states[(int)State::Count];
    states[(int)State::LocalizeConveyor].pump = DoLocalizeConveyor;
    // states[(int)State::PrepareGripper].pump = DoPrepareGripper;
    states[(int)State::FindObject].pump = DoFindObject;
    states[(int)State::ExecutePickup].pump = DoMoveToGrasp;
    states[(int)State::GraspObject].pump = DoGraspObject;
    // states[(int)State::CloseGripper].pump = DoCloseGripper;
    states[(int)State::ExecuteDropoff].pump = DoExecuteDropoff;
    // states[(int)State::OpenGripper].pump = DoOpenGripper;

    ros::Rate loop_rate(5.0);

    State prev_state = State::LocalizeConveyor;
    State curr_state = State::LocalizeConveyor;
    State next_state = State::LocalizeConveyor;

    while (ros::ok()) {
        ros::spinOnce();

        if (prev_state != curr_state) {
            ROS_INFO("Enter state '%s' -> '%s'", to_cstring(prev_state), to_cstring(curr_state));
            if (states[(int)curr_state].enter) {
                states[(int)curr_state].enter(prev_state);
            }
        }

        // assert(states[(int)curr_state] != NULL);
        next_state = states[(int)curr_state].pump();

        if (next_state != curr_state) {
            ROS_INFO("Exit state '%s' -> '%s'", to_cstring(curr_state), to_cstring(next_state));
            if (states[(int)curr_state].exit) {
                states[(int)curr_state].exit(curr_state);
            }
        }

        curr_state = next_state;
        loop_rate.sleep();
    }

    return 0;
}