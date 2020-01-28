#include <ros/ros.h>
#include <publisher_subscriber.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

template<>
void PublisherSubscriber<geometry_msgs::PoseStamped, sensor_msgs::camera>::subscriberCallback(const nav_msgs::Path::ConstPtr& receivedMsg)
{
  ROS_INFO("Sending the received message on 'pose' topic");
  geometry_msgs::PoseStamped pose_msg;
  pose_msg = receivedMsg->poses.front();  // poses are received in a vector, only the first element is needed, other elements are copies of the first one
  publisherObject.publish(pose_msg);
}

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, "publish_pose_stamped_node");
  PublisherSubscriber<geometry_msgs::PoseStamped, nav_msgs::Path> parrot("pose", "rtabmap/mapPath", 1);
  ros::spin();
}
