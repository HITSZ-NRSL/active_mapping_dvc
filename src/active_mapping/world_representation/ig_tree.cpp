/*
 * Created on Thu Jan 21 2021
 *
 * Copyright (c) 2021 HITSZ-NRSL
 * All rights reserved
 *
 * Author: Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich,
 * Switzerland)
 *
 * Rewritter: EpsAvlc
 */

#include "active_mapping/world_representation/ig_tree.h"

#include <pcl/filters/frustum_culling.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <random>
#include <cmath>
#include "active_mapping/utils/octomap_utils.h"
#include "active_mapping/utils/tictoc.hpp"

namespace octomap {
IgTreeNode::IgTreeNode() : ColorOcTreeNode(), has_no_measurement_(false) {}

IgTreeNode::~IgTreeNode() {}

bool IgTreeNode::operator==(const IgTreeNode &rhs) const {
  return rhs.value == value && rhs.has_no_measurement_ == has_no_measurement_ && rhs.color == color;
}

void IgTreeNode::copyData(const IgTreeNode &from) {
  value = from.value;
  has_no_measurement_ = from.has_no_measurement_;
  this->color = from.color;
}

double IgTreeNode::getMeanChildLogOdds() const {
  double mean = 0;
  char c = 0;
  if (children != NULL) {
    for (unsigned int i = 0; i < 8; i++) {
      IgTreeNode *child = static_cast<IgTreeNode *>(children[i]);
      if (child != NULL) {
        mean += child->getOccupancy();
        c++;
      }
    }
  }
  if (c)
    mean /= static_cast<double>(c);

  return log(mean / (1 - mean));
}

float IgTreeNode::getMaxChildLogOdds() const {
  float max = -std::numeric_limits<float>::max();
  if (children != NULL) {
    for (unsigned int i = 0; i < 8; i++) {
      IgTreeNode *child = static_cast<IgTreeNode *>(children[i]);
      if (child != NULL) {
        float l = child->getLogOdds();
        if (l > max)
          max = l;
      }
    }
  }
  return max;
}

void IgTreeNode::addValue(const float &logOdds) { value += logOdds; }

std::istream &IgTreeNode::readData(std::istream &s) {
  s.read(reinterpret_cast<char *>(&this->value), sizeof(value));  // occupancy
  s.read(reinterpret_cast<char *>(&this->color), sizeof(ColorOcTreeNode::Color));
  s.read(reinterpret_cast<char *>(&has_no_measurement_), sizeof(has_no_measurement_));
  s.read(reinterpret_cast<char *>(&is_frontier_), sizeof(is_frontier_));
  s.read(reinterpret_cast<char *>(&mu_), sizeof(mu_));
  return s;
}

std::ostream &IgTreeNode::writeData(std::ostream &s) const {
  s.write(reinterpret_cast<const char *>(&this->value), sizeof(value));
  s.write(reinterpret_cast<const char *>(&this->color), sizeof(ColorOcTreeNode::Color));
  s.write(reinterpret_cast<const char *>(&has_no_measurement_), sizeof(has_no_measurement_));
  s.write(reinterpret_cast<const char *>(&is_frontier_), sizeof(is_frontier_));
  s.write(reinterpret_cast<const char *>(&mu_), sizeof(mu_));

  return s;
}

void IgTreeNode::insertPoint(const octomap::point3d &pt) {
  octomap::point3d diff_pt = pt;
  Eigen::Vector3f diff_pt_eigen;
  diff_pt_eigen.x() = diff_pt.x();
  diff_pt_eigen.y() = diff_pt.y();
  diff_pt_eigen.z() = diff_pt.z();
  float factor = 1 / static_cast<float>(pt_cnt_ + 1);
  sigma_ =
      pt_cnt_ * factor * sigma_ + pt_cnt_ * factor * factor * (diff_pt_eigen - mu_) * (diff_pt_eigen - mu_).transpose();
  mu_ = mu_ + factor * (diff_pt_eigen - mu_);
  ++pt_cnt_;
}

void IgTreeNode::deletePoint(const octomap::point3d &pt) {
  Eigen::Vector3f pt_eigen = octomap::octomapPointToEigenVector<float>(pt);
  float factor = 1 / static_cast<float>(pt_cnt_ - 1);
  mu_ = mu_ + factor * (mu_ - pt_eigen);
  sigma_ = pt_cnt_ * factor * sigma_ - pt_cnt_ * factor * factor * (pt_eigen - mu_) * (pt_eigen - mu_).transpose();
  --pt_cnt_;
}

/*************************** IgTree *****************************/

IgTree::Config::Config()
    : resolution_m(0.1), occupancy_threshold(0.5), hit_probability(0.9), miss_probability(0.2),
      clamping_threshold_min(0.12), clamping_threshold_max(0.97) {}

IgTree::IgTree(double resolution_m, bool use_disjoint_set)
    : ::octomap::OccupancyOcTreeBase<IgTreeNode>(resolution_m), octree_hash_(this),
      use_disjoint_set_(use_disjoint_set) {
  config_.resolution_m = resolution_m;
  ::octomap::AbstractOcTree::registerTreeType(this);
  updateOctreeConfig();
}

IgTree::IgTree(Config config)
    : ::octomap::OccupancyOcTreeBase<IgTreeNode>(config.resolution_m), config_(config), octree_hash_(this) {
  updateOctreeConfig();
}

IgTree *IgTree::create() const { return new IgTree(config_); }

const IgTree::Config &IgTree::config() const { return config_; }

void IgTree::updateOctreeConfig() {
  setOccupancyThres(config_.occupancy_threshold);
  setProbHit(config_.hit_probability);
  setProbMiss(config_.miss_probability);
  setClampingThresMin(config_.clamping_threshold_min);
  setClampingThresMax(config_.clamping_threshold_max);
}

bool IgTree::isNodeUnknown(octomap::IgTreeNode *node_ptr) {
  if (node_ptr == NULL || node_ptr->hasMeasurement() == false)
    return true;
  else
    return false;
}

std::string IgTree::getTreeType() const { return "IgTree"; }

void IgTree::expandNode(IgTreeNode *node) {
  assert(!nodeHasChildren(node));

  for (unsigned int k = 0; k < 8; k++) {
    IgTreeNode *child = createNodeChild(node, k);
    child->copyData(*node);
  }
}

bool IgTree::pruneNode(IgTreeNode *node) {
  // if (!isNodeCollapsible(node) || this->isNodeOccupied(node)
  //   || node->isFrontier())
  //   return false;

  // // set value to children's values (all assumed equal)
  // node->copyData(*(getNodeChild(node, 0)));

  // for (unsigned int i = 0; i < 8; i++) {
  //   if (getNodeChild(node, i)->hasMeasurement()) {
  //     node->updateHasMeasurement(true);
  //     break;
  //   }
  // }

  // node->updateHasMeasurement(true);

  // if (node->hasMeasurement()) {
  //   node->updateOccDist(node->getMinChildOccDist());
  //   node->setMaxDist(node->getMaxChildDist());

  //   if (node->isColorSet()) {
  //     node->setColor(node->getAverageChildColor());
  //   }
  // }

  // // delete children
  // for (unsigned int i = 0; i < 8; i++) {
  //   deleteNodeChild(node, i);
  // }
  // delete[] node->children;
  // node->children = NULL;

  return false;
}

IgTreeNode *IgTree::setNodeColor(const ::octomap::OcTreeKey &key, uint8_t r, uint8_t g, uint8_t b) {
  IgTreeNode *n = search(key);
  if (n != 0) {
    n->setColor(r, g, b);
  }
  return n;
}



void IgTree::insertPointCloud(const octomap::Pointcloud &valid_pc, const octomap::point3d &origin, octomap::Boundingbox &building_box) {
  /* During inserting point cloud, the voxels can't be queried.*/
  KeySet free_cells, occupied_cells;
  KeyRay key_ray_temp;

  std::unordered_map<octomap::OcTreeKey, std::vector<point3d>, octomap::OcTreeKey::KeyHash> key_point_set;
  for (size_t i = 0; i < valid_pc.size(); ++i) {
    point3d point = valid_pc[i];
    point3d dir = point - origin;
    dir /= dir.norm();
    point3d truncted_origin;
    if (truncted_length_ < 0 ) {
      truncted_origin = origin;
    } else {
      truncted_origin = point - dir * truncted_length_;
    }

    if (this->computeRayKeys(truncted_origin, point, key_ray_temp)) {
      free_cells.insert(key_ray_temp.begin(), key_ray_temp.end());
    }
    OcTreeKey key;
    if (this->coordToKeyChecked(point, key)) {
      occupied_cells.insert(key);
      key_point_set[key].push_back(point);
    }
  }

  for (KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
    OcTreeKey voxel_key = *it;
    if (occupied_cells.find(voxel_key) == occupied_cells.end()) {
      IgTreeNode *voxel = search(*it);
      if (voxel == NULL) {
        voxel = this->updateNode(voxel_key, false);
        voxel->updateHasMeasurement(true);
      } else {
        if (!voxel->hasMeasurement()) {
          float logodds_first_miss = octomap::logodds(this->config().miss_probability);
          voxel->setLogOdds(logodds_first_miss);
          voxel->updateHasMeasurement(true);
        } else {
          this->updateNode(voxel_key, false);
          voxel->updateHasMeasurement(true);
        }
      }
      // octree_hash_.insert(voxel_key, voxel, isNodeOccupied(voxel));
    }
  }

  if (use_disjoint_set_) {
    for (KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
      disjoint_set_.insert(*it);
    }
    for (KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
      std::vector<OcTreeKey> neigh_keys;
      getNeighborKeys(*it, &neigh_keys, 5);
      for (auto &key : neigh_keys) {
        if (disjoint_set_.count(key)) {
          disjoint_set_.merge(*it, key);
        }
      }
    }

	if(has_building_root_)	 	// work
	{
		for (KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) 
		{
			OcTreeKey voxel_key = *it;
			if(this->disjointSet().find(voxel_key) == this->getBuildingRoot())
			{
				octomap::point3d voxel_center = this->keyToCoord(voxel_key);
				building_box.insertPoint(voxel_center);
			}
		}		
	}
  }

  for (KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it) {
    OcTreeKey voxel_key = *it;
    IgTreeNode *voxel = search(voxel_key);

    if (voxel == NULL) {
      voxel = this->updateNode(voxel_key, true);
      voxel->updateHasMeasurement(true);
    } else {
      if (!voxel->hasMeasurement()) {
        float logodds_first_hit = octomap::logodds(this->config().hit_probability);
        voxel->setLogOdds(logodds_first_hit);
        voxel->updateHasMeasurement(true);
      } else {
        this->updateNode(voxel_key, true);
        // voxel->updateHasMeasurement(true);
      }
    }

    octomap::point3d voxel_center = this->keyToCoord(voxel_key);
    for (point3d &pt : key_point_set[voxel_key]) {
      voxel->insertPoint(pt);
    }
  }
}

void IgTree::pullOutPointCloud(const octomap::Pointcloud &pc, const octomap::point3d &origin) {
  for (int i = 0; i < pc.size(); ++i) {
    IgTreeNode *tree_node = this->search(pc[i]);
    if (tree_node) {
      if (tree_node->pointCount() == 1) {
        this->deleteNode(pc[i]);
      } else {
        tree_node->deletePoint(pc[i]);
      }
    }
  }
}

void IgTree::deleteNodesOutsideBbox(const octomap::Boundingbox &bbox) {
  std::vector<OcTreeKey> keys_to_delete;
  for (auto it = this->begin_leafs(); it != this->end_leafs(); ++it) {
    OcTreeKey leaf_key = it.getKey();
    point3d leaf_pt = this->keyToCoord(leaf_key);
    if (!bbox.ifContain(leaf_pt)) {
      keys_to_delete.push_back(leaf_key);
    }
  }

  for (auto &key : keys_to_delete) {
    this->deleteNode(key);
  }
}

int IgTree::getBuildingVoxelsNum() {
  return disjoint_set_.getUnionedVoxelSize(building_root_);
}

std::vector<OcTreeKey> IgTree::getBuildingVoxels() {
  return disjoint_set_.getUnionedVoxels(building_root_);
}

void IgTree::getNeighborhoodAtPoint27(const octomap::point3d &pt, std::vector<octomap::IgTreeNode *> *neighborhood) {
  neighborhood->clear();
  OcTreeKey pt_key = this->coordToKey(pt);
  OcTreeKey neigh_key;
  for (int i = -1; i <= 1; ++i)
    for (int j = -1; j <= 1; ++j)
      for (int k = -1; k <= 1; ++k) {
        neigh_key.k[0] = pt_key[0] + i;
        neigh_key.k[1] = pt_key[1] + j;
        neigh_key.k[2] = pt_key[2] + k;
        neighborhood->push_back(this->search(neigh_key));
      }
}

void IgTree::getNeighborhoodAtPoint7(const octomap::point3d &pt, std::vector<octomap::IgTreeNode *> *neighborhood) {
  neighborhood->clear();
  static std::vector<int> dx = {0, -1, 1, 0, 0, 0, 0};
  static std::vector<int> dy = {0, 0, 0, -1, 1, 0, 0};
  static std::vector<int> dz = {0, 0, 0, 0, 0, -1, 1};
  std::vector<octomap::OcTreeKey> neigh_keys;
  OcTreeKey key = this->coordToKey(pt);
  for (int i = 0; i < dx.size(); ++i) {
    octomap::OcTreeKey neigh_key;
    neigh_key[0] = dx[i] + key[0];
    neigh_key[1] = dy[i] + key[1];
    neigh_key[2] = dz[i] + key[2];
    neigh_keys.push_back(neigh_key);
  }
  for (int i = 0; i < neigh_keys.size(); ++i) {
    IgTreeNode *neigh_node = this->search(neigh_keys[i]);
    neighborhood->push_back(neigh_node);
  }
}

void IgTree::getNeighborhoodAtPoint1(const octomap::point3d &pt, std::vector<octomap::IgTreeNode *> *neighborhood) {
  neighborhood->clear();
  neighborhood->push_back(this->search(pt));
}

};  // namespace octomap
