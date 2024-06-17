/**
 * @file icp_matching.h
 * @author Toshiki Nakamura
 * @brief ICP matching class
 * @copyright Copyright (c) 2024
 */

#ifndef ICP_MATCHING_ICP_MATCHING_H
#define ICP_MATCHING_ICP_MATCHING_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ros/ros.h>
#include <string>

class ICPMatching
{
public:
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

  ICPMatching(void);
  void read_pcd(const std::string &file_name, PointCloudT::Ptr cloud);
  void voxel_grid_filter(const PointCloudT::Ptr cloud, const float leaf_size, PointCloudT::Ptr cloud_filtered);
  void align(
      const PointCloudT::Ptr cloud_src, const PointCloudT::Ptr cloud_target, PointCloudT &cloud_src_registered,
      const int iterations = 10);
  bool has_converged(void) { return has_converged_; }
  float get_fitness_score(void) { return fitness_score_; }
  Eigen::Matrix4f get_transformation(void) { return transformation_; }

private:
  bool has_converged_;
  float fitness_score_;
  Eigen::Matrix4f transformation_;
};

#endif  // ICP_MATCHING_ICP_MATCHING_H
