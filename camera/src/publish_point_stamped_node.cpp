#include <ros/ros.h>
#include <publisher_subscriber.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

// TUM Pioneer
template<>
void PublisherSubscriber<geometry_msgs::PointStamped, nav_msgs::Odometry>::subscriberCallback(const nav_msgs::Odometry::ConstPtr& receivedMsg)
{
  ROS_INFO("Sending the received message on 'odom_points' topic");
  geometry_msgs::PointStamped point_msg;
  point_msg.header = receivedMsg->header;

  // point_msg.point = receivedMsg->pose.pose.position;

  if (receivedMsg->pose.pose.position.x==0.0 &&  receivedMsg->pose.pose.position.y==0.0 && receivedMsg->pose.pose.position.z==0.0)
  {
    point_msg.point.x = x_prev;
    point_msg.point.y = y_prev;
    point_msg.point.z = z_prev;
  }
   else {
    point_msg.point = receivedMsg->pose.pose.position;
    x_prev = point_msg.point.x;
    y_prev = point_msg.point.y;
    z_prev = point_msg.point.z;
  }

  publisherObject.publish(point_msg);
}

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, "publish_point_stamped_node");
  PublisherSubscriber<geometry_msgs::PointStamped, nav_msgs::Odometry> parrot("odom_points", "/rtabmap/odom", 1);
  ros::spin();
}
