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

  tf::TransformBroadcaster broadcaster;
  tf::StampedTransform transform;
  transform.frame_id_ = "base_footprint";
  transform.child_frame_id_ = "goal";
  tf::poseMsgToTF(tip_pose, transform);

  while (ros::ok()) {
    // publish goal pose
    broadcaster.sendTransform(transform);
    loop_rate.sleep();
  }

  return 0;
}
