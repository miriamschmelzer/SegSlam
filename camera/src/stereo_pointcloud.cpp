#include <stereo_pointcloud.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


StereoPointcloud::StereoPointcloud()
{}

void StereoPointcloud::scanACallback(const sensor_msgs::PointCloud2& cloud_msg_in) {
  if (tf_listener_.waitForTransform("odom", "camera_depth_optical_frame",
                                    cloud_msg_in.header.stamp, ros::Duration(kTimeout_s))) {
    // Get the tf transform.
    tf::StampedTransform tf_transform;
    tf_listener_.lookupTransform("odom", "camera_depth_optical_frame",
                                 cloud_msg_in.header.stamp, tf_transform);

    // Convert input cloud to laser scan.
//    LaserScan new_scan;
//    new_scan.scan = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg_in);
//    new_scan.time_ns = rosTimeToCurveTime(cloud_msg_in.header.stamp.toNSec());


//    // Get the last cloud in world frame.
//    DataPoints new_fixed_cloud;
//    laser_track_->getLocalCloudInWorldFrame(laser_track_->getMaxTime(), &new_fixed_cloud);

  }
}

void StereoPointcloud::scanBCallback(const sensor_msgs::PointCloud2& cloud_msg_in) {
  if (tf_listener_.waitForTransform("odom", "camera_depth_optical_frame",
                                    cloud_msg_in.header.stamp, ros::Duration(kTimeout_s))) {
    // Get the tf transform.
    tf::StampedTransform tf_transform;
    tf_listener_.lookupTransform("odom", "camera_depth_optical_frame",
                                 cloud_msg_in.header.stamp, tf_transform);

    // Convert input cloud to laser scan.
//    LaserScan new_scan;
//    new_scan.scan = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg_in);
//    new_scan.time_ns = rosTimeToCurveTime(cloud_msg_in.header.stamp.toNSec());


//    // Get the last cloud in world frame.
//    DataPoints new_fixed_cloud;
//    laser_track_->getLocalCloudInWorldFrame(laser_track_->getMaxTime(), &new_fixed_cloud);

  }
}




int main(int argc, char **argv) {
  ros::init(argc, argv, "StereoPointcloud");
  ros::NodeHandle node_handle("~");

  StereoPointcloud stereo_pointcloud(node_handle);

  ros::Subscriber scan_a_sub = node_handle.subscribe("/cam_1/depth/color/points", StereoPointcloud::kScanSubscriberMessageQueueSize,
                           &StereoPointcloud::scanACallback);

  ros::Subscriber scan_b_sub = node_handle.subscribe("/cam_2/depth/color/points", StereoPointcloud::kScanSubscriberMessageQueueSize,
                           &StereoPointcloud::scanBCallback);
  try {
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception");
    return 1;
  }

  return 0;
}
