#include <assert.h>
#include <ros/ros.h>
#include <memory>
#include <actionlib/client/simple_action_client.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf/transform_broadcaster.h>
// #include <geometry_msgs/Pose.h>

using moveit::planning_interface::MoveGroup;

enum struct State {
    LocalizeConveyor = 0,
    // PrepareGripper,
    FindObject,
    ExecutePickup,
    GraspObject,
    // CloseGripper,
    // ExecuteDropoff,
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
    // case State::ExecuteDropoff:     return "ExecuteDropoff";
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

const char* g_planning_group = "right_arm";
std::unique_ptr<MoveGroup> g_move_group;

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
    return State::FindObject;
}

// State DoCloseGripper()
// {
//     return State::ExecuteDropoff;
// }

// State DoExecuteDropoff()
// {
//     return State::OpenGripper;
// }

// State DoOpenGripper()
// {
//     return State::FindObject;
// }

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "walker_simulation");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO("Create Move Group");
//    MoveGroup::Options ops(g_planning_group);
    g_move_group.reset(new MoveGroup(g_planning_group, 
        boost::shared_ptr<tf::Transformer>(), ros::WallDuration(5.0)));

    g_move_group->setPlanningTime(30.0);
    g_move_group->setPlannerId("right_arm[arastar_bfs_manip]");
    g_move_group->setWorkspace(-0.4, -1.2, 0.0, 1.10, 1.2, 2.0);

    MachState states[(int)State::Count];
    states[(int)State::LocalizeConveyor].pump = DoLocalizeConveyor;
    // states[(int)State::PrepareGripper].pump = DoPrepareGripper;
    states[(int)State::FindObject].pump = DoFindObject;
    states[(int)State::ExecutePickup].pump = DoMoveToGrasp;
    states[(int)State::GraspObject].pump = DoGraspObject;
    // states[(int)State::CloseGripper].pump = DoCloseGripper;
    // states[(int)State::ExecuteDropoff].pump = DoExecuteDropoff;
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