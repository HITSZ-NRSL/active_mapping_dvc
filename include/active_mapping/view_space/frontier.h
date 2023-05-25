/*
 * Created on Wed Sep 22 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#include <octomap/OcTree.h>
#include <octomap/octomap_types.h>
#include <vector>
#include "active_mapping/view_space/view.h"
#include "active_mapping/utils/bounding_box.h"

#pragma once

namespace active_mapping {

class FrontierVoxel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit FrontierVoxel(
      const octomap::OcTreeKey &key, const octomap::point3d &coord,
      const std::vector<octomap::OcTreeKey> neigh_unknown_voxels);

  std::vector<octomap::OcTreeKey> &getNeighUnknownVoxels();

  octomap::OcTreeKey &getKey();
  const octomap::OcTreeKey &getKey() const;
  octomap::point3d &getCoord();
  const octomap::point3d &getCoord() const;

 private:
  octomap::OcTreeKey key_;
  octomap::point3d coord_;
  std::vector<octomap::OcTreeKey> neigh_unknown_voxels_;
};

class Frontier{
 public:
  Frontier();

  void computeFrontierInfo();

  std::vector<FrontierVoxel> &voxels();
  const std::vector<FrontierVoxel> &voxels() const;

  octomap::point3d &average();
  const octomap::point3d &average() const;

  int id();
  const int id() const;

  std::vector<View> &views();
  const std::vector<View> &views() const;

  octomap::Boundingbox &bbox();
  const octomap::Boundingbox &bbox() const;

  void setActive(bool active);

  bool isActive() const;

  octomap::KeySet &getNeighUnknownVoxels();

  const octomap::KeySet &getNeighUnknownVoxels() const;

  void insert(const FrontierVoxel &voxel);

 private:
  std::vector<FrontierVoxel> voxels_;
  octomap::point3d average_;
  octomap::Boundingbox bbox_;
  std::vector<View> views_;
  octomap::KeySet neigh_unknown_voxels_;
  int id_;
  static int id_count_;
  bool active_;
};
}  // namespace active_mapping
