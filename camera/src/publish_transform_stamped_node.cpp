#include <ros/ros.h>
#include <string>
#include <publisher_subscriber.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_msgs/TFMessage.h>

template<>
void PublisherSubscriber<geometry_msgs::TransformStamped, tf2_msgs::TFMessage>::subscriberCallback(const tf2_msgs::TFMessage::ConstPtr& receivedMsg)
{
  ROS_INFO("Sending the received message on 'transform' topic");

  ros::Rate r(10); // 10 hz
  while (ros::ok())
  {
    for(int i=0;i<receivedMsg->transforms.size();i++)
    {
      geometry_msgs::TransformStamped transform_msg;
      transform_msg = receivedMsg->transforms[i];
      publisherObject.publish(transform_msg);
      /*if(receivedMsg->transforms[i].child_frame_id.compare("camera_color_optical_frame") == 0)
        {
          geometry_msgs::TransformStamped transform_msg;
          transform_msg = receivedMsg->transforms[i];
          publisherObject.publish(transform_msg);
        }
        */
    }
    r.sleep();
  }


  //geometry_msgs::TransformStamped transform_msg;
  //geometry_msgs::TransformStamped test = receivedMsg->transforms;
  /*geometry_msgs::PoseStamped received_pose = receivedMsg->poses.front(); // poses are received in a vector, only the first element is needed, other elements are copies of the first one
  transform_msg.header = received_pose.header;
  transform_msg.child_frame_id = "";
  transform_msg.transform.translation.x = received_pose.pose.position.x;
  transform_msg.transform.translation.y = received_pose.pose.position.y;
  transform_msg.transform.translation.z = received_pose.pose.position.z;
  transform_msg.transform.rotation = received_pose.pose.orientation;
  */
  //geometry_msgs::TransformStamped received_transform = receivedMsg->transforms.front();
  /*if(receivedMsg.transforms.child_frame_id.compare("camera_accel_optical_frame"))
  {
    transform_msg = receivedMsg.transforms;
    publisherObject.publish(transform_msg);
  }
  */

}

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, "publish_transform_stamped_node");
  PublisherSubscriber<geometry_msgs::TransformStamped, tf2_msgs::TFMessage> parrot("transform", "tf_static", 1);
  ros::spin();
}
