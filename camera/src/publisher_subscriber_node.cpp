#include <ros/ros.h>
#include <publisher_subscriber.h>
#include <std_msgs/String.h>

template<>
void PublisherSubscriber<std_msgs::String, std_msgs::String>::subscriberCallback(const std_msgs::String::ConstPtr& receivedMsg)
{
  ROS_INFO("I received the following: %s", receivedMsg->data.c_str());
  ROS_INFO("Sending the received message on 'echo' topic");
  std_msgs::String echo_msg;
  echo_msg.data = receivedMsg->data;
  publisherObject.publish(echo_msg);
}

int main(int argc, char **argv)
{
  // Set up ROS
  ros::init(argc, argv, "publisher_subscriber_node");
  PublisherSubscriber<std_msgs::String, std_msgs::String> parrot("echo", "chatter", 1);
  ros::spin();
}
