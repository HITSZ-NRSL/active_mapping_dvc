/*
 * Created on Wed Sep 22 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#include "active_mapping/view_space/frontier.h"

namespace active_mapping {

FrontierVoxel::FrontierVoxel(const octomap::OcTreeKey &key, const octomap::point3d &coord,
                          const std::vector<octomap::OcTreeKey> neigh_unknown_voxels)
    : key_(key), coord_(coord), neigh_unknown_voxels_(neigh_unknown_voxels) {}

std::vector<octomap::OcTreeKey> &FrontierVoxel::getNeighUnknownVoxels() { return neigh_unknown_voxels_; }

octomap::OcTreeKey &FrontierVoxel::getKey() { return key_; }

const octomap::OcTreeKey &FrontierVoxel::getKey() const { return key_; }

octomap::point3d &FrontierVoxel::getCoord() { return coord_; }

const octomap::point3d &FrontierVoxel::getCoord() const { return coord_; }

/**************** class Frontier *****************/
Frontier::Frontier() {
  id_ = id_count_;
  ++id_count_;
}

void Frontier::computeFrontierInfo() {
  octomap::point3d center(0, 0, 0);
  for (int i = 0; i < voxels_.size(); ++i) {
    center = center + voxels_[i].getCoord();
    bbox_.insertPoint(voxels_[i].getCoord());
    neigh_unknown_voxels_.insert(voxels_[i].getNeighUnknownVoxels().begin(), voxels_[i].getNeighUnknownVoxels().end());
  }
  center.x() /= voxels_.size();
  center.y() /= voxels_.size();
  center.z() /= voxels_.size();
  average_ = center;
}

std::vector<FrontierVoxel> &Frontier::voxels() { return voxels_; }

const std::vector<FrontierVoxel> &Frontier::voxels() const { return voxels_; }

octomap::point3d &Frontier::average() { return average_; }

const octomap::point3d &Frontier::average() const { return average_; }

int Frontier::id() { return id_; }

const int Frontier::id() const { return id_; }

std::vector<View> &Frontier::views() { return views_; }

const std::vector<View> &Frontier::views() const { return views_; }

octomap::Boundingbox &Frontier::bbox() { return bbox_; }

const octomap::Boundingbox &Frontier::bbox() const { return bbox_; }

void Frontier::setActive(bool active) { active_ = active; }

bool Frontier::isActive() const { return active_; }

octomap::KeySet &Frontier::getNeighUnknownVoxels() { return neigh_unknown_voxels_; }

const octomap::KeySet &Frontier::getNeighUnknownVoxels() const { return neigh_unknown_voxels_; }

void Frontier::insert(const FrontierVoxel &voxel) { voxels_.push_back(voxel); }

int Frontier::id_count_ = 0;

}  // namespace active_mapping
