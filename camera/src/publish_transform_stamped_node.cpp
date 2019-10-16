#include <ros/ros.h>
#include <string>
#include <publisher_subscriber.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>


template<>
void PublisherSubscriber<geometry_msgs::TransformStamped, nav_msgs::Path>::subscriberCallback(const nav_msgs::Path::ConstPtr& receivedMsg)
{
  ROS_INFO("Sending the received message on 'transform' topic");
  geometry_msgs::TransformStamped transform_msg;
  geometry_msgs::PoseStamped received_pose = receivedMsg->poses.front(); // poses are received in a vector, only the first element is needed, other elements are copies of the first one
  transform_msg.header = received_pose.header;
  transform_msg.child_frame_id = "";
  transform_msg.transform.translation.x = received_pose.pose.position.x;
  transform_msg.transform.translation.y = received_pose.pose.position.y;
  transform_msg.transform.translation.z = received_pose.pose.position.z;
  transform_msg.transform.rotation = received_pose.pose.orientation;
  publisherObject.publish(transform_msg);

}

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, "publish_transform_stamped_node");
  PublisherSubscriber<geometry_msgs::TransformStamped, nav_msgs::Path> parrot("transform", "rtabmap/mapPath", 1);
  ros::spin();
}
