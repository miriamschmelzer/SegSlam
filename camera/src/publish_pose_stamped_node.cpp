#include <ros/ros.h>
#include <publisher_subscriber.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

template<>
void PublisherSubscriber<geometry_msgs::PoseStamped, nav_msgs::Path>::subscriberCallback(const nav_msgs::Path::ConstPtr& receivedMsg)
{
  ROS_INFO("Sending the received message on 'pose' topic");
  geometry_msgs::PoseStamped pose_msg;
  pose_msg = receivedMsg->poses.front();  // poses are received in a vector, only the first element is needed, other elements are copies of the first one
  publisherObject.publish(pose_msg);
}

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, "publisher_subscriber_node");
  PublisherSubscriber<geometry_msgs::PoseStamped, nav_msgs::Path> parrot("pose", "rtabmap/mapPath", 1);
  ros::spin();
}
