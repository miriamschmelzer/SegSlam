#include <ros/ros.h>
#include <string>
#include <publisher_subscriber.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rosbag/bag.h>

// RS D435i
int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_tf_regularly_node");

  ros::NodeHandle node;

  //ros::Publisher transform_stamped = node.advertise<std::vector< geometry_msgs::TransformStamped >>("transforms", 100);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  //rosbag::Bag my_bag;
  ros::Rate rate(10.0);
  while (node.ok()){
    static tf2_ros::TransformBroadcaster br;
    //geometry_msgs::TransformStamped transform;
    std::vector< geometry_msgs::TransformStamped > transforms;
    try {
      transforms.push_back(tfBuffer.lookupTransform("odom", "camera_link", ros::Time(0)));
      //transform_stamped.publish(transform);
      //br.sendTransform(transform);
      //continue;

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    try {
      transforms.push_back(tfBuffer.lookupTransform("camera_link", "camera_depth_frame", ros::Time(0)));
      //transform_stamped.publish(transform);
      //br.sendTransform(transform);
      //continue;

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    try {
      transforms.push_back(tfBuffer.lookupTransform("camera_depth_frame", "camera_depth_optical_frame", ros::Time(0)));
      //transform_stamped.publish(transform);
      //br.sendTransform(transform);
      //continue;

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    br.sendTransform(transforms);
    rate.sleep();
  }
  return 0;
}


/*
// TUM
int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_tf_regularly_node");

  ros::NodeHandle node;

  //ros::Publisher transform_stamped = node.advertise<std::vector< geometry_msgs::TransformStamped >>("transforms", 100);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  //rosbag::Bag my_bag;
  ros::Rate rate(10.0);
  while (node.ok()){
    static tf2_ros::TransformBroadcaster br;
    //geometry_msgs::TransformStamped transform;
    std::vector< geometry_msgs::TransformStamped > transforms;
    try {
      transforms.push_back(tfBuffer.lookupTransform("world", "kinect", ros::Time(0)));
      //transform_stamped.publish(transform);
      //br.sendTransform(transform);
      //continue;

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    try {
      transforms.push_back(tfBuffer.lookupTransform("kinect", "openni_camera", ros::Time(0)));
      //transform_stamped.publish(transform);
      //br.sendTransform(transform);
      //continue;

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

//    try {
//      transforms.push_back(tfBuffer.lookupTransform("laser", "openni_camera", ros::Time(0)));
//      //transform_stamped.publish(transform);
//      //dTransform(transform);
//      //continue;

//    } catch (tf2::TransformException &ex) {
//        ROS_WARN("%s",ex.what());
//        ros::Duration(1.0).sleep();
//    }

    try {
      transforms.push_back(tfBuffer.lookupTransform("openni_camera", "openni_rgb_frame", ros::Time(0)));
      //transform_stamped.publish(transform);
      //br.sendTransform(transform);
      //continue;

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    int main(int argc, char** argv)
    {
      ros::init(argc, argv, "publish_tf_regularly_node");

      ros::NodeHandle node;

      //ros::Publisher transform_stamped = node.advertise<std::vector< geometry_msgs::TransformStamped >>("transforms", 100);

      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener(tfBuffer);

      //rosbag::Bag my_bag;
      ros::Rate rate(10.0);
      while (node.ok()){
        static tf2_ros::TransformBroadcaster br;
        //geometry_msgs::TransformStamped transform;
        std::vector< geometry_msgs::TransformStamped > transforms;
        try {
          transforms.push_back(tfBuffer.lookupTransform("world", "kinect", ros::Time(0)));
          //transform_stamped.publish(transform);
          //br.sendTransform(transform);
          //continue;

        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        try {
          transforms.push_back(tfBuffer.lookupTransform("kinect", "openni_camera", ros::Time(0)));
          //transform_stamped.publish(transform);
          //br.sendTransform(transform);
          //continue;

        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

    //    try {
    //      transforms.push_back(tfBuffer.lookupTransform("laser", "openni_camera", ros::Time(0)));
    //      //transform_stamped.publish(transform);
    //      //dTransform(transform);
    //      //continue;

    //    } catch (tf2::TransformException &ex) {
    //        ROS_WARN("%s",ex.what());
    //        ros::Duration(1.0).sleep();
    //    }

        try {
          transforms.push_back(tfBuffer.lookupTransform("openni_camera", "openni_rgb_frame", ros::Time(0)));
          //transform_stamped.publish(transform);
          //br.sendTransform(transform);
          //continue;

        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        try {
          transforms.push_back(tfBuffer.lookupTransform("openni_rgb_frame", "openni_rgb_optical_frame", ros::Time(0)));
          //transform_stamped.publish(transform);
          //br.sendTransform(transform);
          //continue;

        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        //transform_stamped.publish(transforms);
    //    tf::tfMessage tfmsg;
    //    tfmsg.set_transforms_vec(transforms);
    //    my_bag.write("/tf", ros::Time::now(), tfmsg);
        br.sendTransform(transforms);
        rate.sleep();
      }
      return 0;
    }

    try {
      transforms.push_back(tfBuffer.lookupTransform("openni_rgb_frame", "openni_rgb_optical_frame", ros::Time(0)));
      //transform_stamped.publish(transform);
      //br.sendTransform(transform);
      //continue;

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    //transform_stamped.publish(transforms);
//    tf::tfMessage tfmsg;
//    tfmsg.set_transforms_vec(transforms);
//    my_bag.write("/tf", ros::Time::now(), tfmsg);
    br.sendTransform(transforms);
    rate.sleep();
  }
  return 0;
} */
