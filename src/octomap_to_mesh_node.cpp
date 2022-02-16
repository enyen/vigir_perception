#include "ros/ros.h"
#include <vigir_point_cloud_proc/octomap_to_mesh_ros.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "octomap_to_mesh");
  vigir_point_cloud_proc::OctomapToMeshRos<pcl::PointXYZ> conv;

  ros::spin();
  return 0;
}
