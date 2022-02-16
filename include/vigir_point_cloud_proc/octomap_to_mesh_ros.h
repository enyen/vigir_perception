#ifndef OCTOMAP_TO_MESH_ROS_H_
#define OCTOMAP_TO_MESH_ROS_H_

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vigir_point_cloud_proc/mesh_conversions.h>
#include <vigir_point_cloud_proc/cloud_to_mesh.h>
#include <vigir_point_cloud_proc/mesh_conversions.h>
#include <vigir_point_cloud_proc/StringInput.h>

#include <octomap/octomap.h>


using namespace std;
using namespace octomap;

namespace vigir_point_cloud_proc
{

template <typename PointT>
class OctomapToMeshRos
{
public:
  OctomapToMeshRos()
  {
    ros::NodeHandle pnh("~");
    convert_srv = pnh.advertiseService("octomap_to_mesh", &OctomapToMeshRos::srvCallback, this);
    cloud_to_mesh_.setVoxelFilterSize(0.025);
  }

  bool srvCallback(StringInput::Request  &req, StringInput::Response &res) {
    boost::shared_ptr<pcl::PointCloud<PointT> > cloud (new pcl::PointCloud<PointT>());
    bt2pcl(cloud, req.data);
    cloud_to_mesh_.setInput(cloud);
    string filename = req.data + ".ply";
    if (cloud_to_mesh_.computeMesh()){
      meshToPly_nocolor(cloud_to_mesh_.getMesh() ,filename);
      ROS_INFO("Mesh %s saved.", filename.c_str());
    }

    return true;
  }

  bool bt2pcl(boost::shared_ptr<pcl::PointCloud<PointT> > cloud, string filename)
  {
    OcTree* tree = new OcTree(filename);
    double mapres = tree->getResolution();

    for(OcTree::leaf_iterator it = tree->begin(), end=tree->end(); it!= end; ++it) {
      if(tree->isNodeOccupied(*it)){
        cloud->push_back(PointT(it.getX(), it.getY(), it.getZ()));
//        cloud->push_back(PointT(it.getX() + mapres / 2,
//                                it.getY() + mapres / 2,
//                                it.getZ() + mapres / 2));
//        cloud->push_back(PointT(it.getX() + mapres / 2,
//                                it.getY() - mapres / 2,
//                                it.getZ() + mapres / 2));
//        cloud->push_back(PointT(it.getX() - mapres / 2,
//                                it.getY() - mapres / 2,
//                                it.getZ() + mapres / 2));
//        cloud->push_back(PointT(it.getX() - mapres / 2,
//                                it.getY() + mapres / 2,
//                                it.getZ() + mapres / 2));
//        cloud->push_back(PointT(it.getX() + mapres / 2,
//                                it.getY() + mapres / 2,
//                                it.getZ() - mapres / 2));
//        cloud->push_back(PointT(it.getX() + mapres / 2,
//                                it.getY() - mapres / 2,
//                                it.getZ() - mapres / 2));
//        cloud->push_back(PointT(it.getX() - mapres / 2,
//                                it.getY() - mapres / 2,
//                                it.getZ() - mapres / 2));
//        cloud->push_back(PointT(it.getX() - mapres / 2,
//                                it.getY() + mapres / 2,
//                                it.getZ() - mapres / 2));
      }
    }
    delete tree;
    return true;
  }


private:
  ros::ServiceServer convert_srv;
  CloudToMesh<PointT, pcl::PointNormal> cloud_to_mesh_;
};

}
#endif
