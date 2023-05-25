/*
 * Created on Thu Sep 23 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */
#include "active_mapping/view_space/frontier_view_space_finder.h"

#include <queue>
#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <lkh_tsp_solver/lkh_interface.h>
#include <tf/tf.h>

#include "active_mapping/utils/octomap_utils.h"
#include "active_mapping/utils/tictoc.hpp"
#include "active_mapping/utils/common_utils.h"
#include "active_mapping/utils/math_utils.h"

namespace active_mapping {
FrontierViewSpaceFinder::FrontierViewSpaceFinder() { loadParameters(); }

/**
 * @brief set ROI of the object to be detected.
 *
 * @param bbox
 */
void FrontierViewSpaceFinder::setROIBbox(const octomap::Boundingbox &bbox) { bbox_roi_ = bbox; }

/**
 * @brief Get the Frontiers
 *
 * @return const std::unordered_map<int, Frontier>&
 */
const std::unordered_map<int, Frontier> &FrontierViewSpaceFinder::getFrontiers() const { return frontiers_; }

/**
 * @brief Get a specific frontier
 *
 * @param id frontier id
 * @return const Frontier&
 */
const Frontier &FrontierViewSpaceFinder::getFrontier(int id) const { return frontiers_.at(id); }

std::vector<Frontier> FrontierViewSpaceFinder::getActiveFrontiers() {
  std::vector<Frontier> res;
  for (auto frt : frontiers_) {
    if (frt.second.isActive())
      res.push_back(frt.second);
  }
  return res;
}

bool FrontierViewSpaceFinder::loadParameters() {
  ros::NodeHandle nh_local("/active_mapping/frontier_view_space_finder");
  ros::NodeHandle nh_ac("/active_mapping");
  nh_local.getParam("display_frontier_voxels", display_frontier_voxels_flag_);
  nh_local.getParam("max_cluster_xyz", max_cluster_xyz_);
  nh_local.getParam("min_cluster_size", min_cluster_size_);
  nh_local.getParam("roi_min_z", bbox_roi_.bottom_right_front_pt.z());
  nh_local.getParam("view_min_dist_from_object", view_min_dist_from_object_);
  nh_local.getParam("yaw_step", yaw_step_);
  nh_local.getParam("radius_step", radius_step_);
  nh_local.getParam("radius_min", radius_min_);
  nh_local.getParam("radius_max", radius_max_);
  nh_local.getParam("max_pitch", max_pitch_);
  nh_local.getParam("min_pitch", min_pitch_);
  nh_ac.getParam("use_disjoint_set", use_disjoint_set_);
  nh_ac.getParam("world_frame", world_frame_);
  return true;
}

void FrontierViewSpaceFinder::setFrontierColor(int frontier_id, uint8_t r, uint8_t g, uint8_t b) {
  const octomap::IgTree *ig_tree = reinterpret_cast<const octomap::IgTree *>(octree_ptr_);
  for (auto &voxel : frontiers_[frontier_id].voxels()) {
    octomap::IgTreeNode *node = octree_ptr_->search(voxel.getKey());
    node->setColor(r, g, b);
  }
}

void FrontierViewSpaceFinder::setFrontierColor(const Frontier &frt, uint8_t r, uint8_t g, uint8_t b) {
  for (auto &voxel : frt.voxels()) {
    octomap::IgTreeNode *node = octree_ptr_->search(voxel.getKey());
    node->setColor(r, g, b);
  }
}

void FrontierViewSpaceFinder::setWorldRepresentation(WorldRepresentation::Ptr wr) {
  world_representation_ = wr;
  octree_ptr_ = wr->getOctreePtr();

  sr.octree_ptr_ = octree_ptr_;
  sr.wr_ = wr;
}

bool FrontierViewSpaceFinder::isVoxelFrontier(const octomap::point3d *sensor_origin, const octomap::OcTreeKey &node_key,
                                              std::vector<octomap::OcTreeKey> *neigh_unknown_voxel_keys) {
  if (neigh_unknown_voxel_keys) {
    neigh_unknown_voxel_keys->clear();
  }

  octomap::IgTreeNode *tree_node = octree_ptr_->search(node_key);
  octomap::point3d pt = octree_ptr_->keyToCoord(node_key);
  if (bbox_roi_.minZ() > pt.z()) {
    return false;
  }
  /* Frontier should be a free voxel. */
  if (tree_node == nullptr || octree_ptr_->isNodeOccupied(tree_node))
    return false;

  if (tree_node->hasMeasurement() == false)
    return false;

  if (nullptr != sensor_origin) {
    octomap::point3d cast_origin = *sensor_origin;
    octomap::point3d cast_dir = pt - cast_origin;
    double cast_range = cast_dir.norm();
    octomap::point3d cast_end;
    bool find_end_pt = octree_ptr_->castRay(cast_origin, cast_dir, cast_end, true, cast_range);
    if (find_end_pt) {
      return false;
    }
  }

  std::vector<octomap::IgTreeNode *> neighs;
  std::vector<octomap::OcTreeKey> neigh_keys;
  octomap::getNeighborSix(*octree_ptr_, node_key, &neighs, &neigh_keys);

  int unknown_voxel_count = 0;
  int occ_voxel_count = 0;
  for (int i = 0; i < neigh_keys.size(); ++i) {
    octomap::point3d pt = octree_ptr_->keyToCoord(neigh_keys[i]);
    if (!world_representation_->getROI().ifContain(pt)) {
      continue;
    }

    if (!neighs[i] || (!neighs[i]->hasMeasurement())) {
      ++unknown_voxel_count;
      if (neigh_unknown_voxel_keys) {
        neigh_unknown_voxel_keys->push_back(neigh_keys[i]);
      }
    } else if (this->octree_ptr_->isNodeOccupied(neighs[i])) {
      if (use_disjoint_set_) {
        octomap::OcTreeKey building_root = octree_ptr_->getBuildingRoot();
        if (octree_ptr_->disjointSet().count(neigh_keys[i]) &&
            octree_ptr_->disjointSet().find(neigh_keys[i]) == building_root) {
          ++occ_voxel_count;
        }
      } else {
        ++occ_voxel_count;
      }
    }
  }

  if (unknown_voxel_count > 0 && occ_voxel_count > 0) {
    tree_node->setFrontier(true);
  } else {
    tree_node->setFrontier(false);
  }
  return tree_node->isFrontier();
}

std::vector<Frontier> FrontierViewSpaceFinder::frontierCalculation(const octomap::point3d &sensor_origin,
                                                                   const octomap::Boundingbox &update_bbox) {
  std::lock_guard<std::mutex> lock(world_representation_->octomapMutex());
  octomap::IgTree::leaf_bbx_iterator iter_begin, iter_end;
  if (update_bbox.isReset()) {
    octomap::OcTreeKey min_key(0, 0, 0), max_key(65535, 65535, 65535);
    iter_begin = octree_ptr_->begin_leafs_bbx(min_key, max_key);
  } else {
    iter_begin = octree_ptr_->begin_leafs_bbx(update_bbox.minPoint(), update_bbox.maxPoint());
  }
  iter_end = octree_ptr_->end_leafs_bbx();

  octomap::KeySet prob_frt_voxels;
  for (auto it = iter_begin; it != iter_end; ++it) {
    const octomap::OcTreeKey &node_key = it.getKey();
    octomap::IgTreeNode *tree_node = this->octree_ptr_->search(it.getKey());
    if (!tree_node || !octree_ptr_->isNodeOccupied(tree_node)) {
      continue;
    }
    std::vector<octomap::OcTreeKey> neigh_keys;
    std::vector<octomap::IgTreeNode *> node_vec;
    octomap::getNeighborSix(*octree_ptr_, node_key, &node_vec, &neigh_keys);
    for (int i = 0; i < neigh_keys.size(); ++i) {
      if (node_vec[i] && !octree_ptr_->isNodeOccupied(node_vec[i])) {
        prob_frt_voxels.insert(neigh_keys[i]);
      }
    }
  }

  std::vector<Frontier> frt_vec;
  octomap::KeySet visited_voxels;

  for (auto it = prob_frt_voxels.begin(); it != prob_frt_voxels.end(); ++it) {
    const octomap::OcTreeKey &node_key = *it;
    if (visited_voxels.find(node_key) != visited_voxels.end())
      continue;

    octomap::IgTreeNode *tree_node = this->octree_ptr_->search(node_key);
    if (!tree_node)
      continue;

    /* Frontier should be a free voxel. */
    if (this->octree_ptr_->isNodeOccupied(tree_node)) {
      continue;
    }
    std::vector<octomap::OcTreeKey> neigh_unknown_keys;
    if (!isVoxelFrontier(nullptr, node_key, &neigh_unknown_keys)) {
      continue;
    }

    octomap::point3d node_coord = this->octree_ptr_->keyToCoord(node_key);
    std::queue<FrontierVoxel> voxel_q;
    Frontier curr_frt;
    octomap::KeySet frt_key_set;
    voxel_q.push(FrontierVoxel(node_key, node_coord, neigh_unknown_keys));
    while (!voxel_q.empty()) {
      FrontierVoxel curr_voxel = voxel_q.front();
      voxel_q.pop();
      if (visited_voxels.find(curr_voxel.getKey()) != visited_voxels.end()) {
        continue;
      }
      curr_frt.insert(curr_voxel);
      visited_voxels.insert(curr_voxel.getKey());
      std::vector<octomap::OcTreeKey> curr_neigh_keys;
      octomap::getNeighborKeys(curr_voxel.getKey(), &curr_neigh_keys);
      for (int i = 0; i < curr_neigh_keys.size(); ++i) {
        if (visited_voxels.find(curr_neigh_keys[i]) != visited_voxels.end()) {
          continue;
        }
        if (isVoxelFrontier(nullptr, curr_neigh_keys[i], &neigh_unknown_keys)) {
          FrontierVoxel curr_neigh_voxel(curr_neigh_keys[i], octree_ptr_->keyToCoord(curr_neigh_keys[i]),
                                         neigh_unknown_keys);
          voxel_q.push(curr_neigh_voxel);
        }
      }
    }
    if (curr_frt.voxels().size() < min_cluster_size_) {
      if (display_frontier_voxels_flag_) {
        for (auto &voxel : curr_frt.voxels()) {
          octomap::IgTreeNode *node = octree_ptr_->search(voxel.getKey());
          node->setFrontier(false);
        }
      }
      continue;
    }
    curr_frt.computeFrontierInfo();
    frt_vec.push_back(curr_frt);
  }

  return frt_vec;
}

bool FrontierViewSpaceFinder::splitFrontierPCA(const Frontier &frt, std::vector<Frontier> *split_frts) {
  split_frts->clear();
  auto voxels = frt.voxels();
  bool need_to_split = false;
  for (auto voxel : voxels) {
    if ((voxel.getCoord() - frt.average()).norm() > max_cluster_xyz_) {
      need_to_split = true;
      break;
    }
  }
  if (!need_to_split) {
    return false;
  }
  Eigen::Matrix3d cov;
  cov.setZero();
  for (auto voxel : frt.voxels()) {
    octomap::point3d diff_octomap = voxel.getCoord() - frt.average();
    Eigen::Vector3d diff(diff_octomap.x(), diff_octomap.y(), diff_octomap.z());
    cov += diff * diff.transpose();
  }
  cov /= static_cast<double>(frt.voxels().size());

  Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int max_idx;
  double max_eigenvalue = -1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] > max_eigenvalue) {
      max_idx = i;
      max_eigenvalue = values[i];
    }
  }

  Eigen::Vector3d first_pc = vectors.col(max_idx);

  Frontier frt1, frt2;
  for (auto voxel : frt.voxels()) {
    octomap::point3d diff_octomap = voxel.getCoord() - frt.average();
    Eigen::Vector3d diff(diff_octomap.x(), diff_octomap.y(), diff_octomap.z());
    if (diff.dot(first_pc) >= 0)
      frt1.insert(voxel);
    else
      frt2.insert(voxel);
  }
  frt1.computeFrontierInfo();
  frt2.computeFrontierInfo();

  std::vector<Frontier> split_frts_new;
  if (splitFrontierPCA(frt1, &split_frts_new)) {
    for (int i = 0; i < split_frts_new.size(); ++i) {
      split_frts->push_back(std::move(split_frts_new[i]));
    }
  } else {
    split_frts->push_back(frt1);
  }

  if (splitFrontierPCA(frt2, &split_frts_new)) {
    for (int i = 0; i < split_frts_new.size(); ++i) {
      split_frts->push_back(std::move(split_frts_new[i]));
    }
  } else {
    split_frts->push_back(frt2);
  }

  return true;
}

std::vector<Frontier> FrontierViewSpaceFinder::splitClusters(const std::vector<Frontier> &frontiers) {
  std::vector<Frontier> res_frts, tmp_frts;

  for (auto frt : frontiers) {
    if (splitFrontierPCA(frt, &tmp_frts)) {
      // res_frts.insert(res_frts.end(), tmp_frts.begin(), tmp_frts.end());
      for (int i = 0; i < tmp_frts.size(); ++i) {
        res_frts.push_back(std::move(tmp_frts[i]));
      }
    } else {
      res_frts.push_back(frt);
    }
  }
  return res_frts;
}

void FrontierViewSpaceFinder::deleteFrontiersInBbox(const octomap::Boundingbox &bbox) {
  std::vector<int> frt_id_to_erase;
  for (auto frt_pair : frontiers_) {
    if (bbox.ifIntersect(frt_pair.second.bbox())) {
      if (display_frontier_voxels_flag_) {
        for (auto voxel : frt_pair.second.voxels()) {
          const octomap::IgTree *ig_tree_ptr = reinterpret_cast<const octomap::IgTree *>(this->octree_ptr_);
          octomap::IgTreeNode *node_ptr = ig_tree_ptr->search(voxel.getCoord());
          node_ptr->setColor(255, 255, 255);
        }
      }
      frt_id_to_erase.push_back(frt_pair.first);
    }
  }
  for (int i = 0; i < frt_id_to_erase.size(); ++i) {
    frontiers_.erase(frt_id_to_erase[i]);
  }
}

geometry_msgs::Pose FrontierViewSpaceFinder::calcFrontierNormal(active_mapping::Frontier frt) {
  octomap::point3d node_coord = frt.average();
  octomap::OcTreeKey node_key = this->octree_ptr_->coordToKey(node_coord);
  std::vector<octomap::OcTreeKey> curr_neigh_keys;
  std::vector<octomap::IgTreeNode *> curr_neighs;
  octomap::getNeighbor(*octree_ptr_, node_key, &curr_neighs, &curr_neigh_keys, 10);

  Eigen::Matrix3d cov;
  cov.setZero();
  for (int i = 0; i < curr_neigh_keys.size(); i++) {
    if (!curr_neighs[i] || !curr_neighs[i]->hasMeasurement() || !this->octree_ptr_->isNodeOccupied(curr_neighs[i]))
      continue;
    octomap::point3d neigh_node_coord = this->octree_ptr_->keyToCoord(curr_neigh_keys[i]);
    octomap::point3d diff_octomap = neigh_node_coord - node_coord;
    Eigen::Vector3d diff(diff_octomap.x(), diff_octomap.y(), diff_octomap.z());
    cov += diff * diff.transpose();
  }
  cov /= static_cast<double>(curr_neigh_keys.size());

  Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
  auto values = es.eigenvalues().real();
  auto vectors = es.eigenvectors().real();
  int min_idx;
  double max_eigenvalue = 1000000;
  for (int i = 0; i < values.rows(); ++i) {
    if (values[i] < max_eigenvalue) {
      min_idx = i;
      max_eigenvalue = values[i];
    }
  }
  Eigen::Vector3d first_pc = vectors.col(min_idx);
  Eigen::Quaterniond quat_pc = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(1, 0, 0), first_pc);

  geometry_msgs::Pose pose;
  pose.orientation.w = quat_pc.w();
  pose.orientation.x = quat_pc.x();
  pose.orientation.y = quat_pc.y();
  pose.orientation.z = quat_pc.z();
  pose.position.x = node_coord.x();
  pose.position.y = node_coord.y();
  pose.position.z = node_coord.z();
  return pose;
}

nav_msgs::Path FrontierViewSpaceFinder::getFrontiersAndViewpoints(const Eigen::Isometry3d &sensor_pose,
                                                                  const octomap::Boundingbox &update_bbox) {
  static View last_view;
  static bool first_flag = true;
  static float inactive_frt_height = 1e9;
  std::vector<octomap::point3d> voxels_to_dilate;
  octomap::Boundingbox modified_update_bbox = update_bbox;
  if (last_visit_frontier_.views().size() != 0) {
    for (auto voxel : last_visit_frontier_.voxels()) {
      modified_update_bbox.insertPoint(voxel.getCoord());
    }
  }

  for (auto &frt : getFrontiers()) {
    if (modified_update_bbox.ifIntersect(frt.second.bbox())) {
      for (const FrontierVoxel &voxel : frt.second.voxels()) {
        voxels_to_dilate.push_back(voxel.getCoord());
      }
    }
  }
  world_representation_->dilateFreeSpaceVoxelsFromFreeVoxels(sensor_pose, voxels_to_dilate);
  deleteFrontiersInBbox(modified_update_bbox);

  octomap::point3d sensor_origin = octomap::toOctomap(sensor_pose.translation());
  world_representation_->dilateFreeSpaceVoxelsFromOccupiedVoxels(sensor_pose, modified_update_bbox);
  std::vector<Frontier> new_frontiers = frontierCalculation(sensor_origin, modified_update_bbox);
  std::vector<Frontier> new_split_clusters = splitClusters(new_frontiers);

  for (int i = 0; i < new_split_clusters.size(); ++i) {
    Frontier &frt = new_split_clusters.at(i);
    frt.setActive(true);
    frontiers_.insert(std::make_pair(frt.id(), frt));
  }
  frontiers_pose_.poses.clear();
  frontiers_color_.poses.clear();
  uint r_seed = 2412352;

  sr.updateBuildingBoundingBox(world_representation_->getBuildingBoundingbox());
  for (auto frt_pair : frontiers_) {
    int r = rand_r(&r_seed) % 200;
    int g = rand_r(&r_seed) % 200;
    int b = rand_r(&r_seed) % 200;

    Eigen::Vector3f curr_point(frt_pair.second.average().x(), frt_pair.second.average().y(),
                               frt_pair.second.average().z());
    int frontier_side = sr.queryCurrentQuadrant(curr_point);
    if (side_id_ != 0 && frontier_side != sr.side_order_[side_id_])  // work
      continue;

    if (frt_pair.second.isActive() == true) {
      for (int i = 0; i < history_frontiers.size(); i++) {
        Eigen::Vector3f frt_h = history_frontiers[i];
        if ((curr_point - frt_h).norm() < 0.4 && frontier_side == sr.queryCurrentQuadrant(frt_h))  // work
        {
          frt_pair.second.setActive(false);
          break;
        }
      }
    }
    history_frontiers.push_back(curr_point);

    if (frt_pair.second.isActive() == false) {
      for (int i = 0; i < frt_pair.second.voxels().size(); ++i) {
        octomap::IgTreeNode *node_ptr = this->octree_ptr_->search(frt_pair.second.voxels()[i].getKey());
        node_ptr->setColor(255, 255, 255);
      }
      continue;
    }

    for (int i = 0; i < frt_pair.second.voxels().size(); ++i) {
      octomap::IgTreeNode *node_ptr = this->octree_ptr_->search(frt_pair.second.voxels()[i].getKey());
      node_ptr->setColor(r + 56, g + 56, b + 56);
    }

    {
      geometry_msgs::PoseStamped pose_t;
      pose_t.pose = calcFrontierNormal(frt_pair.second);
      frontiers_pose_.poses.push_back(pose_t);

      geometry_msgs::PoseStamped color;
      color.pose.position.x = r + 56;
      color.pose.position.y = g + 56;
      color.pose.position.z = b + 56;
      frontiers_color_.poses.push_back(color);
    }

    frt_pair.second.setActive(false);
  }

  {
    geometry_msgs::Pose bbox;
    bbox.position.x = world_representation_->getBuildingBoundingbox().maxX();
    bbox.position.y = world_representation_->getBuildingBoundingbox().maxY();
    bbox.position.z = world_representation_->getBuildingBoundingbox().maxZ();
    bbox.orientation.x = world_representation_->getBuildingBoundingbox().minX();
    bbox.orientation.y = world_representation_->getBuildingBoundingbox().minY();
    bbox.orientation.z = world_representation_->getBuildingBoundingbox().minZ();

    geometry_msgs::Pose robot_pose;
    robot_pose.position.x = sensor_pose.translation()(0);
    robot_pose.position.y = sensor_pose.translation()(1);
    robot_pose.position.z = sensor_pose.translation()(2);

    Eigen::Quaterniond sensor_orientation(sensor_pose.rotation());
    robot_pose.orientation.w = sensor_orientation.w();
    robot_pose.orientation.x = sensor_orientation.x();
    robot_pose.orientation.y = sensor_orientation.y();
    robot_pose.orientation.z = sensor_orientation.z();

    current_vp_path_ = sr.sampleViewpointsFunc(frontiers_pose_, frontiers_color_, robot_pose, bbox, &side_id_);
  }

  return current_vp_path_;
}

}  // namespace active_mapping
