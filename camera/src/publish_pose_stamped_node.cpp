#include <ros/ros.h>
#include <publisher_subscriber.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>


// TUM ground truth
/*template<>
void PublisherSubscriber<geometry_msgs::PoseStamped, visualization_msgs::MarkerArray>::subscriberCallback(const visualization_msgs::MarkerArray::ConstPtr& receivedMsg)
{
  ROS_INFO("Sending the received message on 'pose_imu' topic");

  for(int i = 0; i<=receivedMsg->markers.size(); i++)
  {
    if(receivedMsg->markers[i].ns == "kinect")
    {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header = receivedMsg->markers[0].header;
      pose_msg.pose = receivedMsg->markers[0].pose;
      publisherObject.publish(pose_msg);
    }
  }
//  geometry_msgs::PoseStamped pose_msg;
//  pose_msg.header = receivedMsg->markers[0].header;
//  pose_msg.pose = receivedMsg->markers[0].pose;
//  publisherObject.publish(pose_msg);
}

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, "publish_pose_stamped_node");
  PublisherSubscriber<geometry_msgs::PoseStamped, visualization_msgs::MarkerArray> parrot("pose_imu", "/cortex_marker_array", 1);
  ros::spin();
}
*/
// TUM Pioneer
//template<>
//void PublisherSubscriber<geometry_msgs::PoseStamped, nav_msgs::Odometry>::subscriberCallback(const nav_msgs::Odometry::ConstPtr& receivedMsg)
//{
//  ROS_INFO("Sending the received message on 'pose' topic");
//  geometry_msgs::PoseStamped pose_msg;
//  pose_msg.header = receivedMsg->header;
//  pose_msg.pose = receivedMsg->pose.pose;
//  publisherObject.publish(pose_msg);
//}

//int main(int argc, char **argv)
//{
//  // Set up ROS
//  ros::init(argc, argv, "publish_pose_stamped_node");
//  PublisherSubscriber<geometry_msgs::PoseStamped, nav_msgs::Odometry> parrot("pose_imu", "/pose", 1);
//  ros::spin();
//}

// RS D435i
template<>
void PublisherSubscriber<geometry_msgs::PoseStamped, nav_msgs::Odometry>::subscriberCallback(const nav_msgs::Odometry::ConstPtr& receivedMsg)
{
  ROS_INFO("Sending the received message on 'pose_imu' topic");
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = receivedMsg->header;
  pose_msg.pose = receivedMsg->pose.pose;
  publisherObject.publish(pose_msg);
}

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, "publish_pose_stamped_node");
  PublisherSubscriber<geometry_msgs::PoseStamped, nav_msgs::Odometry> parrot("pose_imu", "/rtabmap/odom", 1);
  ros::spin();
}

// D435
/*
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
  ros::init(argc, argv, "publish_pose_stamped_node");
  PublisherSubscriber<geometry_msgs::PoseStamped, nav_msgs::Path> parrot("pose", "rtabmap/mapPath", 1);
  ros::spin();
}
*/
