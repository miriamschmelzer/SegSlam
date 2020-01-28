#include <ros/ros.h>
#include <string>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>


// TUM

void callback(const geometry_msgs::TransformStamped::ConstPtr& receivedMsg)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transform;
  transform.header = receivedMsg->header;
  transform.child_frame_id = receivedMsg->child_frame_id;
  transform.transform = receivedMsg->transform;
  br.sendTransform(transform);
}

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, "publish_transform_stamped_to_tf_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("transforms", 1, callback);
  ros::spin();
}
