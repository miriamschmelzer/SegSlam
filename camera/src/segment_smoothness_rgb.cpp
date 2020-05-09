#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>

#include <iostream>
#include <thread>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::fromROSMsg (*input, *cloud);

  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud <pcl::PointXYZRGB>);
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  float voxel_size = 0.04;
  sor.setLeafSize (voxel_size, voxel_size, voxel_size);
  sor.setMinimumPointsNumberPerVoxel (2);
  sor.filter (*cloud_filtered);

  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud_filtered);
  normal_estimator.setRadiusSearch (0.05); //.setKSearch (50);
  //normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 5.0);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setInputCloud (cloud_filtered);
  reg.setInputNormals (normals);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (0.06);
  //reg.setNumberOfNeighbours (30);
  reg.setPointColorThreshold (6);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setRegionColorThreshold (5);
  reg.setCurvatureThreshold (1.0); // 0.05
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (15000);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);


  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

  sensor_msgs::PointCloud2 output;

  pcl::toROSMsg(*colored_cloud,output);
  output.header.frame_id = input->header.frame_id;

  // Publish the data
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "segment_smoothness_rgb");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output_smooth", 1);

  // Spin
  ros::spin ();
}
