#include <ros/ros.h>
#include <string>
#include <publisher_subscriber.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rosbag/bag.h>

// Odometrie + RS D435i

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publish_tf_regularly_node");

  ros::NodeHandle node;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transform;
  geometry_msgs::TransformStamped transform_odom;
  geometry_msgs::TransformStamped transform_depth;
  geometry_msgs::TransformStamped transform_depth_optical;

  ros::Publisher pose_publisher = node.advertise<geometry_msgs::PoseStamped>("pose_imu", 100);
  geometry_msgs::PoseStamped pose_imu;
  ros::Publisher transform_publisher = node.advertise<geometry_msgs::TransformStamped>("transform_imu", 100);
  geometry_msgs::TransformStamped transform_imu;

  ros::Time stamp;
  ros::Rate rate(10.0);
  while (node.ok()){
    std::vector< geometry_msgs::TransformStamped > transforms;

//    try {
//      transform_depth = tfBuffer.lookupTransform("camera_link", "camera_depth_frame", ros::Time(0));
//    } catch (tf2::TransformException &ex) {
//      ROS_WARN("%s",ex.what());
//    }

//    try {
//      transform_depth_optical = tfBuffer.lookupTransform("camera_depth_frame", "camera_depth_optical_frame", ros::Time(0));
//    } catch (tf2::TransformException &ex) {
//      ROS_WARN("%s",ex.what());
//    }

    try {
      transform_odom = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));

      stamp =  transform_odom.header.stamp;

      transforms.push_back(transform_odom);

      transform.header.stamp = stamp;
      transform.header.frame_id = "base_link";
      transform.child_frame_id = "camera_link";
      transform.transform.translation.x = 0.0;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = 0.85;
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
      transform.transform.rotation.w = 1.0;

      transforms.push_back(transform);

      transform.header.stamp = stamp;
      transform.header.frame_id = "camera_link";
      transform.child_frame_id = "camera_depth_frame";
      transform.transform.translation.x = 0.0;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
      transform.transform.rotation.w = 1.0;

      transforms.push_back(transform);

      transform.header.stamp = stamp;
      transform.header.frame_id = "camera_depth_frame";
      transform.child_frame_id = "camera_depth_optical_frame";
      transform.transform.translation.x = 0.0;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation.x = -0.5;
      transform.transform.rotation.y = 0.5;
      transform.transform.rotation.z = -0.5;
      transform.transform.rotation.w = 0.5;

      transforms.push_back(transform);

      pose_imu.header = transform_odom.header;
      pose_imu.pose.position.x = transform_odom.transform.translation.x;
      pose_imu.pose.position.y = transform_odom.transform.translation.y;
      pose_imu.pose.position.z = transform_odom.transform.translation.z;
      pose_imu.pose.orientation = transform_odom.transform.rotation;
      pose_publisher.publish(pose_imu);

      transform_imu = transform_odom;
      transform_imu.child_frame_id = "";
//      transform_imu.header = transform_odom.header;
//      transform_imu.child_frame_id = "";
//      transform_imu.transform = transform_odom.transform;
      transform_publisher.publish(transform_imu);

//      // transform = tfBuffer.lookupTransform("camera_link", "camera_depth_frame", ros::Time(0));
//      transform_depth.header.stamp = stamp;
//      transforms.push_back(transform_depth);

//      // transform = tfBuffer.lookupTransform("camera_depth_frame", "camera_depth_optical_frame", ros::Time(0));
//      transform_depth_optical.header.stamp = stamp;
//      transforms.push_back(transform_depth_optical);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    br.sendTransform(transforms);
    rate.sleep();
  }
  return 0;
}



// RS D435i

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "publish_tf_regularly_node");

//  ros::NodeHandle node;

//  //ros::Publisher transform_stamped = node.advertise<std::vector< geometry_msgs::TransformStamped >>("transforms", 100);

//  tf2_ros::Buffer tfBuffer;
//  tf2_ros::TransformListener tfListener(tfBuffer);

//  static tf2_ros::TransformBroadcaster br;
//  geometry_msgs::TransformStamped transform;
//  geometry_msgs::TransformStamped transform_depth;
//  geometry_msgs::TransformStamped transform_depth_optical;

//  ros::Time stamp;
//  ros::Rate rate(10.0);
//  while (node.ok()){
//    std::vector< geometry_msgs::TransformStamped > transforms;
//    try {
//      transform_depth = tfBuffer.lookupTransform("camera_link", "camera_depth_frame", ros::Time(0));
//      stamp =  transform_depth.header.stamp; // ros::Time::now();

//      transform.header.stamp = stamp;
//      transform.header.frame_id = "odom";
//      transform.child_frame_id = "base_link";
//      transform.transform.translation.x = 0.0;
//      transform.transform.translation.y = 0.0;
//      transform.transform.translation.z = 0.0;
//      transform.transform.rotation.x = 0.0;
//      transform.transform.rotation.y = 0.0;
//      transform.transform.rotation.z = 0.0;
//      transform.transform.rotation.w = 1.0;

//      transforms.push_back(transform);

//      transform.header.stamp = stamp;
//      transform.header.frame_id = "base_link";
//      transform.child_frame_id = "camera_link";
//      transform.transform.translation.x = 0.0;
//      transform.transform.translation.y = 0.0;
//      transform.transform.translation.z = 0.88;
//      transform.transform.rotation.x = 0.0;
//      transform.transform.rotation.y = 0.0;
//      transform.transform.rotation.z = 0.0;
//      transform.transform.rotation.w = 1.0;

//      transforms.push_back(transform);

//      transforms.push_back(transform_depth);

//      transform_depth_optical = tfBuffer.lookupTransform("camera_depth_frame", "camera_depth_optical_frame", ros::Time(0));
//      transform_depth_optical.header.stamp = stamp;
//      transforms.push_back(transform_depth_optical);
//    } catch (tf2::TransformException &ex) {
////      ROS_WARN("%s",ex.what());
//      ROS_WARN("%s",ex.what());
//      ros::Duration(1.0).sleep();
//    }
////    transform_depth_optical.header.stamp = stamp;
////    transforms.push_back(transform_depth_optical);

//    br.sendTransform(transforms);
//    rate.sleep();
//  }
//  return 0;
//}

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "publish_tf_regularly_node");

//  ros::NodeHandle node;

//  //ros::Publisher transform_stamped = node.advertise<std::vector< geometry_msgs::TransformStamped >>("transforms", 100);

//  tf2_ros::Buffer tfBuffer;
//  tf2_ros::TransformListener tfListener(tfBuffer);

//  //rosbag::Bag my_bag;
//  ros::Rate rate(10.0);
//  while (node.ok()){
//    static tf2_ros::TransformBroadcaster br;
//    //geometry_msgs::TransformStamped transform;
//    std::vector< geometry_msgs::TransformStamped > transforms;
//    try {
//      transforms.push_back(tfBuffer.lookupTransform("odom", "camera_link", ros::Time(0)));
//      //transform_stamped.publish(transform);
//      //br.sendTransform(transform);
//      //continue;

//    } catch (tf2::TransformException &ex) {
//        ROS_WARN("%s",ex.what());
//        ros::Duration(1.0).sleep();
//    }

//    try {
//      transforms.push_back(tfBuffer.lookupTransform("camera_link", "camera_depth_frame", ros::Time(0)));
//      //transform_stamped.publish(transform);
//      //br.sendTransform(transform);
//      //continue;

//    } catch (tf2::TransformException &ex) {
//        ROS_WARN("%s",ex.what());
//        ros::Duration(1.0).sleep();
//    }

//    try {
//      transforms.push_back(tfBuffer.lookupTransform("camera_depth_frame", "camera_depth_optical_frame", ros::Time(0)));
//      //transform_stamped.publish(transform);
//      //br.sendTransform(transform);
//      //continue;

//    } catch (tf2::TransformException &ex) {
//        ROS_WARN("%s",ex.what());
//        ros::Duration(1.0).sleep();
//    }

//    br.sendTransform(transforms);
//    rate.sleep();
//  }
//  return 0;
//}


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
