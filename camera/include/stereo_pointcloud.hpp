#ifndef STEREO_POINTCLOUD_HPP
#define STEREO_POINTCLOUD_HPP

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>



class StereoPointcloud
{
public:
  StereoPointcloud();

  void scanACallback(const sensor_msgs::PointCloud2& cloud_msg_in);
  void scanBCallback(const sensor_msgs::PointCloud2& cloud_msg_in);

  tf::TransformListener tf_listener_;

  static constexpr double kTimeout_s = 0.2;
  static constexpr unsigned int kScanSubscriberMessageQueueSize = 1u;

};

#endif // STEREO_POINTCLOUD_HPP
