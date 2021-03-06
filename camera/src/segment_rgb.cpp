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


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  pcl::fromROSMsg (*input, *cloud);


  std::cout << "test: " << cloud->points[4]  << std::endl;
  std::cout << "test2: " << cloud->points[4].rgb << cloud->points[4].r << cloud->points[4].g << cloud->points[4].b << std::endl;

  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud <pcl::PointXYZRGB>);
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  float voxel_size = 0.04;
  sor.setMinimumPointsNumberPerVoxel(3);
  sor.setLeafSize (voxel_size, voxel_size, voxel_size);
  sor.filter (*cloud_filtered);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 5.0);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud_filtered);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (0.06);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (2);
  reg.setMinClusterSize (60);
  reg.setMaxClusterSize (5000);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

  sensor_msgs::PointCloud2 output;

  pcl::toROSMsg(*colored_cloud,output);
  output.header.frame_id = input->header.frame_id;

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "segment_rgb");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output_rgb", 1);

  // Spin
  ros::spin ();
}



/*
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
//  BOOST_FOREACH (cloud, msg->points);

//  pcl::visualization::CloudViewer viewer ("Cluster viewer");
//  viewer.showCloud (cloud);

//  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

//  pcl::IndicesPtr indices (new std::vector <int>);
//  pcl::PassThrough<pcl::PointXYZRGB> pass;
//  pass.setInputCloud (cloud);
//  pass.setFilterFieldName ("z");
//  pass.setFilterLimits (0.0, 5.0);
//  pass.filter (*indices);

//  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
//  reg.setInputCloud (cloud);
//  reg.setIndices (indices);
//  reg.setSearchMethod (tree);
//  reg.setDistanceThreshold (10);
//  reg.setPointColorThreshold (6);
//  reg.setRegionColorThreshold (5);
//  reg.setMinClusterSize (600);

//  std::vector <pcl::PointIndices> clusters;
//  reg.extract (clusters);

//  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//  pcl::visualization::CloudViewer viewer ("Cluster viewer");
//  viewer.showCloud (colored_cloud);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 1, callback);
  ros::spin();
}
*/
