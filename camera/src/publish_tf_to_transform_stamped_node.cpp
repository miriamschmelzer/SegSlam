#include <ros/ros.h>
#include <string>
#include <publisher_subscriber.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>


// TUM
int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_tf_to_transform_stamped_node");

  ros::NodeHandle node;

  ros::Publisher transform_stamped = node.advertise<geometry_msgs::TransformStamped>("transform_imu", 100);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStampedTmp;
    try {
      transformStampedTmp = tfBuffer.lookupTransform("world", "kinect", ros::Time(0));
      //transformStampedTmp = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
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


/*template<>
void PublisherSubscriber<geometry_msgs::TransformStamped, tf2_msgs::TFMessage>::subscriberCallback(const tf2_msgs::TFMessage::ConstPtr& receivedMsg)
{


  /*for(int i = 0; i <= sizeof(receivedMsg->transforms); i++)
  {
    if(receivedMsg->transforms[i].header.frame_id.compare("world") == 0)
      publisherObject.publish(receivedMsg->transforms[i]);
  }
  //it = receivedMsg->transforms.begin();
  /*'geometry_msgs::TransformStamped transform_msg = receivedMsg->transforms[1];
  it = find(receivedMsg->transforms.begin(), receivedMsg->transforms.end(), *it.header.frame_id=="world");
  if (it != receivedMsg->transforms.end())
  {
    publisherObject.publish(*it);
  }
  //geometry_msgs::TransformStamped received_transform = receivedMsg->transforms.front(); // transforms are received in a vector, only the first element is needed, other elements are copies of the first one
  /*transform_msg.header = received_pose.header;
  transform_msg.child_frame_id = "";
  transform_msg.transform.translation.x = received_pose.pose.position.x;
  transform_msg.transform.translation.y = received_pose.pose.position.y;
  transform_msg.transform.translation.z = received_pose.pose.position.z;
  transform_msg.transform.rotation = received_pose.pose.orientation;

  publisherObject.publish(transform_msg);
  */

//}
/*
int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, "publish_tf_to_transform_stamped_node");
  PublisherSubscriber<geometry_msgs::TransformStamped, tf2_msgs::TFMessage> parrot("transform", "tf", 1);
  ros::spin();
}
*/
