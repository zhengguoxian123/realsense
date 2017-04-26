/*
Purpose of node: subscribe to a point cloud, use a VoxelGrid filter on it with a setting that
clobbers voxels with fewer than a threshold of points.
*/

#pragma once
#ifndef REALSENSE_CAMERA_VOXEL_GRID_FILTER_NODELET_H
#define REALSENSE_CAMERA_VOXEL_GRID_FILTER_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ PointXYZ;

namespace realsense_camera
{

class FilterAndPublish : public nodelet::Nodelet
{
  public:
    virtual void onInit();
    void pc_callback(const PointCloud::ConstPtr &msg);

    private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    int thresh;
};
} // namespace realsense_camera
#endif // REALSENSE_CAMERA_VOXEL_GRID_FILTER_NODELET_H