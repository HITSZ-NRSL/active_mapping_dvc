/*
 * Created on Thu Sep 23 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#pragma once

#include <octomap/octomap_types.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <vector>
#include <list>
#include <nav_msgs/Path.h>

#include "active_mapping/view_space/frontier.h"
#include "active_mapping/view_space/view_space_finder.h"
#include "active_mapping/world_representation/world_representation.h"

#include "active_mapping/view_space/sample_viewpoints.h"	
namespace active_mapping {

class FrontierViewSpaceFinder : public ViewSpaceFinder {
 public:
  FrontierViewSpaceFinder();
  /**
   * @brief set ROI of the object to be detected.
   *
   * @param bbox
   */
  void setROIBbox(const octomap::Boundingbox &bbox);

  /**
   * @brief Set the Frontier Color
   *
   * @param frontier_id frontier_id
   * @param r red part
   * @param g green part
   * @param b blue part
   */
  void setFrontierColor(int frontier_id, uint8_t r, uint8_t g, uint8_t b);

  /**
   * @brief Set the Frontier Color object
   *
   * @param frt Frontier to set
   * @param r
   * @param g
   * @param b
   */
  void setFrontierColor(const Frontier &frt, uint8_t r, uint8_t g, uint8_t b);

  /**
   * @brief Give world representation ptr to frontier_view_space_finder 
   * 
   * @param wr 
   */
  void setWorldRepresentation(WorldRepresentation::Ptr wr) override;


  /**
   * @brief Return Frontier set's const inference
   * 
   * @return const std::unordered_map<int, Frontier>& 
   */
  const std::unordered_map<int, Frontier> &getFrontiers() const;

  /**
   * @brief Return Frontier's const inference
   * 
   * @param id 
   * @return const Frontier& 
   */
  const Frontier &getFrontier(int id) const;

  /**
   * @brief Get active frontiers' copy
   *
   * @return std::vector<Frontier>
   */
  std::vector<Frontier> getActiveFrontiers();

geometry_msgs::Pose calcFrontierNormal(active_mapping::Frontier frt);	
nav_msgs::Path getFrontiersAndViewpoints(const Eigen::Isometry3d &sensor_pose,
											const octomap::Boundingbox &update_bbox);
int side_id_ = 0;
std::vector<Eigen::Vector3f> history_frontiers;

 private:

  /**
   * @brief Load ros paramters
   *
   * @return true
   * @return false
   */
  bool loadParameters();

  /**
   * @brief Calculate frontiers
   *
   * @return octomap::KeyBoolMap Map which contains frontiers' keys.
   */
  std::vector<Frontier>
  frontierCalculation(const octomap::point3d &sensor_origin,
                      const octomap::Boundingbox &update_bbox = octomap::Boundingbox::infinityMax());

  /**
   * @brief split frontier clusters which size is over threshold
   *
   * @param frontier_clusters input frontier clusters.
   * @return std::vector<Frontier>  output frontiers.
   */
  std::vector<Frontier> splitClusters(const std::vector<Frontier> &frontier_clusters);

  /**
   * @brief Utility function used by splitClusters. Use PCA to obtain the size
   *  of cluster, and decide which dimenson to split.
   *
   * @param frt input frontier.
   * @param split_frts output split frontiers.
   * @return true if successfually split.
   * @return false if don't need to be split.
   */
  bool splitFrontierPCA(const Frontier &frt, std::vector<Frontier> *split_frts);

  /**
   * @brief Compute viewpoint from frts.
   *
   * @param frts input frontiers
   * @return ViewSpace
   */
  ViewSpace computeViewSpace(const octomap::point3d &sensor_origin, std::vector<Frontier> *frts);

  /**
   * @brief Delete the frontiers in the bounding box.
   *
   * @param bbox
   */
  void deleteFrontiersInBbox(const octomap::Boundingbox &bbox);

  /**
   * @brief Judge that if a voxel is a frontier.
   *
   * @param key
   * @param neigh_unknown_voxel_keys
   * @return true
   * @return false
   */
  bool isVoxelFrontier(const octomap::point3d *sensor_origin, const octomap::OcTreeKey &key,
                       std::vector<octomap::OcTreeKey> *neigh_unknown_voxel_keys = nullptr);


  octomap::IgTree *octree_ptr_;
  WorldRepresentation::Ptr world_representation_;

  std::unordered_map<int, Frontier> frontiers_;
  Frontier last_visit_frontier_;
  ViewSpace view_space_;
  octomap::Boundingbox bbox_roi_;
  bool display_frontier_voxels_flag_;
  bool use_disjoint_set_;
  int min_cluster_size_;
  double max_cluster_xyz_;
  double view_min_dist_from_object_ = 4;
  double yaw_step_;
  double radius_step_;
  double radius_min_;
  double radius_max_;
  double max_pitch_;
  double min_pitch_;

  std::string world_frame_;		
  std::string tsp_path_topic; 	
  nav_msgs::Path frontiers_pose_;
  nav_msgs::Path frontiers_color_;
  nav_msgs::Path current_vp_path_;
  ros::NodeHandle nh;
  SamplingRegion sr;
	
};
}  // namespace active_mapping
