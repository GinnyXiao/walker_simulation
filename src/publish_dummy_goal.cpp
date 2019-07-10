#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "goal_publisher");
  ros::NodeHandle nh;
  ros::Rate loop_rate(30.0);

  geometry_msgs::Pose tip_pose;
  tip_pose.orientation.x = 0.1472033;
  tip_pose.orientation.y = 0.2944066;
  tip_pose.orientation.z = 0.0;
  tip_pose.orientation.w = 0.9442754;
  tip_pose.position.x = 0.25;
  tip_pose.position.y = -0.4;
  tip_pose.position.z = 0.8;

  // tip_pose.orientation.x = 0.174;
  // tip_pose.orientation.y = 0.0;
  // tip_pose.orientation.z = -0.1727;
  // tip_pose.orientation.w = 0.97012;
  // tip_pose.position.x = 0.1;
  // tip_pose.position.y = -0.1;
  // tip_pose.position.z = 0.1;

  // tip_pose.orientation.x = 0.726;
  // tip_pose.orientation.y = 0.687;
  // tip_pose.orientation.z = 0.015;
  // tip_pose.orientation.w = 0.018;
  // tip_pose.position.x = 0.502;
  // tip_pose.position.y = -0.503;
  // tip_pose.position.z = 1.107;

  while (ros::ok()) {
    // publish goal pose
    tf::TransformBroadcaster broadcaster;
    tf::StampedTransform transform;
    transform.frame_id_ = "base_footprint";
    transform.child_frame_id_ = "goal";
    tf::poseMsgToTF(tip_pose, transform);
    broadcaster.sendTransform(transform);

    loop_rate.sleep();
  }

  return 0;
}
