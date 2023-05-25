/*
 * Created on Wed Sep 22 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#pragma once

#include <octomap/OcTree.h>
#include <octomap/math/Vector3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <utility>
#include <thread>
#include <mutex>
#include <string>
#include <memory>
#include <list>
#include <vector>
#include <algorithm>
#include <opencv2/core/core.hpp>

#include "active_mapping/world_representation/world_representation.h"
#include "active_mapping/world_representation/ig_tree.h"
#include "active_mapping/save_octomap.h"
#include "active_mapping/utils/octomap_utils.h"
#include "active_mapping/utils/bounding_box.h"
#include "active_mapping/utils/pointxyzirt.h"


namespace active_mapping {
class WorldRepresentation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief Load parameters and publish some topic
   * 
   */
  WorldRepresentation();
  using Ptr = std::shared_ptr<WorldRepresentation>;
  octomap::IgTree *getOctreePtr() { return octree_ptr_.get(); }
  std::shared_ptr<octomap::IgTree> getOcTreeSharedPtr() { return octree_ptr_; }
  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool loadParameters();

  /**
   * @brief RANSAC method to remove groud points
   *
   * @param cloud_in input cloud 
   * @param cloud_out output cloud without ground points
   */
  void removeGround(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out);

  /**
   * @brief This method remove points out of ROI.
   * 
   * @param cloud_in 
   * @param cloud_out 
   */
  void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> *cloud_out);

  /**
   * @brief inset point cloud to octotree.
   * 
   * @param [in] origin sensor position in map frame.
   * @param [in] cloud point cloud to be inserted.
   * @param [in] filter_pointcloud boolean value. If true, filterPointcloud method will be called.
   * @param [in] remove_ground boolean value. If true, removeGround method will be called.
   */
  void insertPointCloud(const Eigen::Vector3d &origin, const pcl::PointCloud<pcl::PointXYZ> &cloud,
                        bool filter_pointcloud = false, bool remove_ground = false);

  /**
   * @brief pull out point cloud from octotree.
   * 
   * @param [in] origin sensor position in map frame.
   * @param [in] cloud point cloud to be inserted.
   * @param [in] filter_pointcloud boolean value. If true, filterPointcloud method will be called.
   */
  void pullOutPointCloud(const Eigen::Vector3d &origin, const pcl::PointCloud<pcl::PointXYZ> &cloud,
                         bool filter_pointcloud = false);


  /**
  * @brief Get the Input Pointcloud Bbox object
  * 
  * @return octomap::Boundingbox 
  */
  octomap::Boundingbox getInputPointcloudBbox() { return input_pc_bbox_; }

  /**
   * @brief reset the input pc bounding box.
   * 
   */
  void resetInputPointcloudBbox() { input_pc_bbox_.reset(); }

  /**
   * @brief Set the Building Root. It is the part of disjoint set.
   * 
   * @param key 
   */
  void setBuildingRoot(const octomap::OcTreeKey &key) { octree_ptr_->setBuildingRoot(key); }

  /**
   * @brief Get the Building Root
   * 
   * @return const octomap::OcTreeKey 
   */
  const octomap::OcTreeKey getBuildingRoot() { return octree_ptr_->getBuildingRoot(); }

  octomap::Boundingbox getROI() const { return roi_; }

  octomap::Boundingbox getOccupiedVoxelBoundingbox() const { return occ_bbox_; }

  pcl::PointCloud<pcl::PointXYZ> getOccupiedVoxelPointCloud() const;

  
  /**
   * @brief Judge that whether a position is drivable
   * 
   * @param x 
   * @param y 
   * @return true 
   * @return false 
   */
  bool isDrivable(const double x, const double y);

  std::mutex &octomapMutex() const { return octree_mutex_; }

  /**
   * @brief Judge if a point is in FOV.
   *
   * @param pt
   * @param view
   * @return true
   * @return false
   */
  bool inFieldOfView(const octomap::point3d &pt, const Eigen::Matrix4d &pose) const;

  /**
   * @brief dialate Free space from occupied voxels.
   * 
   * @param sensor_pose 
   * @param update_box 
   */
  void
  dilateFreeSpaceVoxelsFromOccupiedVoxels(const Eigen::Isometry3d &sensor_pose,
                                          const octomap::Boundingbox &update_box = octomap::Boundingbox::infinityMax());

  /**
   * @brief dialate free space from free voxels. It aims to avoid the robot keep scanning the same position.
   * 
   * @param sensor_pose 
   * @param free_voxels 
   */
  void dilateFreeSpaceVoxelsFromFreeVoxels(const Eigen::Isometry3d &sensor_pose,
                                           const std::vector<octomap::point3d> free_voxels);

  /**
   * @brief sub-function of dilateFreeSpaceVoxelsFromOccupiedVoxels and dilateFreeSpaceVoxelsFromFreeVoxels
   * 
   * @param sensor_pose 
   * @param populate_voxels 
   * @return int 
   */
  int dilateFreeSpaceVoxels(const Eigen::Isometry3d &sensor_pose, const octomap::KeySet &populate_voxels);

  /**
   * @brief Get the number of occupied voxels of building
   * 
   * @return int 
   */
  int getBuildingVoxelsNum() { return octree_ptr_->getBuildingVoxelsNum(); }

  bool savePoints(std::string file_name);


  /**
   * @brief Set the truncted length of octomap. Limit the free voxels.
   * 
   * @param length 
   */
  void setTrunctedLength(double length) { octree_ptr_->setTrunctedLength(length); }

	pcl::PointCloud<pcl::PointXYZ> getBuildingPointCloud();
	void updateBuildingBoundingbox();
	octomap::Boundingbox getBuildingBoundingbox() {	return buliding_bbox_;  }
	octomap::Boundingbox setBuildingBoundingbox(octomap::Boundingbox new_bbox) { buliding_bbox_ = new_bbox;  }
	void startUpdating();
	void stopUpdating();
	octomap::Boundingbox buliding_bbox_;
	ros::Publisher building_bbox_publisher_; 
	ros::Publisher filtered_pcd_publisher_;
	pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
	void saveOctomap(std::string file_path);
	int frame_count_ = 0;
	bool curr_submap_ = false;


 private:
  ros::Publisher octomap_publisher_;
  ros::Publisher roi_publisher_;
  ros::Publisher update_bbox_publisher_;
  
  ros::Subscriber cost_map_subscriber_;
  ros::ServiceServer save_octomap_server_;
  std::shared_ptr<octomap::IgTree> octree_ptr_;
  std::list<std::pair<octomap::Pointcloud, octomap::point3d>> cloud_list_;
  std::thread octomap_pub_thread_;
  std::thread insert_cloud_thread_;
  std::thread roi_pub_thread_;
  mutable std::mutex octree_mutex_;
  std::mutex cloud_list_mutex_;

  std::string octree_frame_;
  bool publish_octree_;
  bool use_disjoint_set_;
  bool visualize_roi_;
  bool visualize_update_bbox_;
  pcl::KdTreeFLANN<pcl::PointXYZ> eval_kdtree_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr eval_pc_;
  std::ofstream eval_log_file_;
  double publish_frequency_;
  double octree_resolution_;

  float min_sensor_dist_;
  float max_sensor_dist_;
  double fov_horizon_;
  double fov_vertical_;

  octomap::Boundingbox roi_;
  octomap::Boundingbox input_pc_bbox_;
  octomap::Boundingbox occ_bbox_;

  nav_msgs::OccupancyGrid cost_map_;

  void publishOcTreeThread();

  void publishROIThread();

  void publishUpdateBBoxThread();

  void insertCloudThread();

  void costmapCallback(const nav_msgs::OccupancyGridConstPtr &costmap_msg);

  bool saveOctomapServerCallback(save_octomap::Request &request, save_octomap::Response &response);

};
}  // namespace active_mapping
