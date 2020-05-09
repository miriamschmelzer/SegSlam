#include <octomap_world/octomap_manager.h>
#include <ros/ros.h>

int main(int argc, char** argv) {

  // Init ROS things.
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  volumetric_mapping::OctomapManager manager(nh, nh_private);

}
