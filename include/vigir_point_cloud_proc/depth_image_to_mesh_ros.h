//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_DEPTH_IMAGE_TO_MESH_ROS_H_
#define VIGIR_DEPTH_IMAGE_TO_MESH_ROS_H_

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vigir_point_cloud_proc/depth_image_to_mesh.h>
#include <vigir_point_cloud_proc/mesh_conversions.h>
#include <depth_image_proc/depth_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace vigir_point_cloud_proc
{

/**
 * @brief The DepthImageToMeshRos class provides
 * a ROS(topic) interface for converting point cloud
 * messages to mesh representations.
 */
template <typename PointT>
class DepthImageToMeshRos
{
public:
  DepthImageToMeshRos():
    pnh_("~"),
    sub_depth_(pnh_, "depth_registered_rect", 1),
    sub_color_(pnh_, "color_rect", 1),
    sub_camera_info_(pnh_, "camera_info", 1),
    sync(rgbdPolicy(10), sub_depth_, sub_color_, sub_camera_info_)
  {
    pnh_.param("max_publish_rate", p_max_rate_hz_, 0.0);
    pnh_.param("ply_dir", ply_dir, std::string(""));
    pnh_.param("target_frame", p_target_frame_, std::string(""));

    if (!p_target_frame_.empty()){
      tfl_.reset(new tf::TransformListener());
      ROS_INFO("DepthImageToMeshRos using target_frame: %s", p_target_frame_.c_str());
    }else{
      ROS_INFO("DepthImageToMeshRos using empty target_frame, not transforming mesh.");
    }

    marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("mesh_marker", 1, true);
    generate_mesh_srv_ = pnh_.advertiseService("generate_mesh", &DepthImageToMeshRos::generateMeshCb, this);
    sync.registerCallback(boost::bind(&DepthImageToMeshRos::rgbdCallback, this, _1, _2, _3));

    srv_requested = false;
    ply_count = 0;
    last_img_pub_stamp_ = ros::Time::now();
  }

  bool generateMeshCb(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    srv_requested = true;
    return true; 
  }

  void rgbdCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& color_msg,
                    const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if (!srv_requested)
      return;
    //If max rate is set, check if we should proceed with computation
    if (p_max_rate_hz_ > 0.0){
     if (depth_msg->header.stamp <= last_img_pub_stamp_ + ros::Duration(1.0/p_max_rate_hz_))
       return;
    }

    // convert depth image to point cloud
    model_.fromCameraInfo(info_msg);
    boost::shared_ptr<pcl::PointCloud<PointT> > cloud (new pcl::PointCloud<PointT>());
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      convertToPcl<uint16_t>(depth_msg, color_msg, cloud, model_, 1.5);
    else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      convertToPcl<float>(depth_msg, color_msg, cloud, model_, 1.5);
    depth_image_to_mesh_.setInput(cloud);

    // convert point cloud to mesh
    if (depth_image_to_mesh_.computeMesh())
    {
      last_img_pub_stamp_ = depth_msg->header.stamp;

      // publish visualization marker
      if (marker_pub_.getNumSubscribers() > 0){
        visualization_msgs::Marker mesh_marker;
        meshToMarkerMsg(depth_image_to_mesh_.getMesh() ,mesh_marker);
        mesh_marker.header.frame_id = depth_msg->header.frame_id;
        marker_pub_.publish(mesh_marker);
      }
      if (srv_requested){
        srv_requested = false;
        ply_count += 1;
        std::string filename = ply_dir + "/" + std::to_string(ply_count) + ".ply";
        if (tfl_->waitForTransform(p_target_frame_, depth_msg->header.frame_id, depth_msg->header.stamp, ros::Duration(0.5))){
          tf::StampedTransform transform;
          tfl_->lookupTransform(p_target_frame_, depth_msg->header.frame_id, depth_msg->header.stamp, transform);

          Eigen::Affine3d transform_eigen;
          tf::transformTFToEigen(transform, transform_eigen);

          meshToPly(*depth_image_to_mesh_.getMeshTransformed(transform_eigen) ,filename);
          ROS_INFO("Mesh %s saved.", filename.c_str());
        }else{
          ROS_WARN("Timeout while transforming from %s to %s, not publishing depth image mesh!",
                    depth_msg->header.frame_id.c_str(),
                    p_target_frame_.c_str());
        }
      }
    }else{
      ROS_WARN("Could not generate mesh for depth image!");
    }
  }

  template<typename T>
  bool convertToPcl(const sensor_msgs::ImageConstPtr& depth_msg,
                    const sensor_msgs::ImageConstPtr& color_msg,
                    boost::shared_ptr<pcl::PointCloud<PointT> > cloud,
                    const image_geometry::PinholeCameraModel& model,
                    double max_range = 1000.0)
  {
    cloud->width  = depth_msg->width;
    cloud->height = depth_msg->height;
    cloud->resize(cloud->width * cloud->height);

    float center_x = model.cx();
    float center_y = model.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = depth_image_proc::DepthTraits<T>::toMeters( T(1) );
    float constant_x = unit_scaling / model.fx();
    float constant_y = unit_scaling / model.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
    int depth_row_step = depth_msg->step / sizeof(T);
    const uint8_t* color_row = &color_msg->data[0];
    int color_row_step = color_msg->step / sizeof(uint8_t);
    for (int v = 0; v < (int)cloud->height; ++v, depth_row += depth_row_step)
    {
      for (int u = 0; u < (int)cloud->width; ++u) //++iter_x, ++iter_y, ++iter_z)
      {
        T depth = depth_row[u];
        PointT& point = (*cloud)(u,v);

        // Missing points denoted by NaNs
        if (!depth_image_proc::DepthTraits<T>::valid(depth))
        {
            point.x = bad_point;
            point.y = bad_point;
            point.z = bad_point;
            continue;
        }

        if (depth_image_proc::DepthTraits<T>::toMeters(depth) > max_range)
        {
          point.x = bad_point;
          point.y = bad_point;
          point.z = bad_point;
        }else{
          point.x = (u - center_x) * depth * constant_x;
          point.y = (v - center_y) * depth * constant_y;
          point.z = depth_image_proc::DepthTraits<T>::toMeters(depth);
          uint32_t rgb = ((uint32_t)color_row[(u*3)] << 16 | (uint32_t)color_row[(u*3) + 1] << 8 | (uint32_t)color_row[(u*3) + 2]);
          point.rgb = *reinterpret_cast<float*>(&rgb);
        }
      }
      color_row += color_row_step;
    }

    return true;
  }

private:
  ros::NodeHandle pnh_;
  message_filters::Subscriber<sensor_msgs::Image> sub_depth_;
  message_filters::Subscriber<sensor_msgs::Image> sub_color_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> rgbdPolicy;
  message_filters::Synchronizer<rgbdPolicy> sync;

  image_geometry::PinholeCameraModel model_;

  ros::Publisher marker_pub_;
  ros::Publisher shape_pub_;
  ros::ServiceServer generate_mesh_srv_;

  sensor_msgs::PointCloud2 cloud_out_;
  sensor_msgs::PointCloud2 cloud_self_filtered_out;

  int p_img_queue_size_;
  double p_max_rate_hz_;
  std::string p_target_frame_;
  bool srv_requested;
  std::string ply_dir;
  int ply_count;

  DepthImageToMesh<PointT> depth_image_to_mesh_;

  boost::shared_ptr<tf::TransformListener> tfl_;

  ros::Time last_img_pub_stamp_;

};

}
#endif
