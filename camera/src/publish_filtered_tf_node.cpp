#include <ros/ros.h>
#include <string>
#include <publisher_subscriber.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>


// TUM
int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_tf_to_transform_stamped_node");

  ros::NodeHandle node;

  ros::Publisher transform_stamped = node.advertise<geometry_msgs::TransformStamped>("tf_filtered", 100);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStampedTmp;
    try {
      transformStampedTmp = tfBuffer.lookupTransform("world", "kinect", ros::Time(0));
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header = transformStampedTmp.header;
      transformStamped.child_frame_id = "";
      transformStamped.transform = transformStampedTmp.transform;
      transform_stamped.publish(transformStamped);

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
    }
    rate.sleep();
  }
  return 0;
}
