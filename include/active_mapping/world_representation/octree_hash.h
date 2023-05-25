/*
 * Created on Wed Jun 02 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#ifndef ACTIVE_MAPPING_WORLD_REPRESENTATION_OCTREE_HASH_H_
#define ACTIVE_MAPPING_WORLD_REPRESENTATION_OCTREE_HASH_H_

#include <octomap/octomap_types.h>
#include <unordered_map>
#include <limits>
#include <vector>
#include <mutex>

namespace octomap {
template <typename TREE_TYPE> class OcTreeHash {
 public:
  typedef typename TREE_TYPE::NodeType NODE_TYPE;
  typedef std::unordered_map<OcTreeKey, NODE_TYPE *, OcTreeKey::KeyHash>
      NodeMap;

  explicit OcTreeHash(TREE_TYPE* tree_ptr) {
    octree_ptr_ = tree_ptr;
  }

  NodeMap &occupiedNodes() { return occ_nodes_; }

  const NodeMap &occupiedNodes() const { return occ_nodes_; }

  NodeMap &freeNodes() { return free_nodes_; }

  const NodeMap &freeNodes() const { return free_nodes_; }

  NodeMap &allNodes() { return all_nodes_; }

  const NodeMap &allNodes() const { return all_nodes_; }

  NODE_TYPE *search(const OcTreeKey &key) const {
    std::lock_guard<std::mutex> locker(query_mutex_);
    if (all_nodes_.count(key)) {
      return all_nodes_.at(key);
    } else {
      return NULL;
    }
  }

  NODE_TYPE *search(const point3d &pt) const {
    OcTreeKey key = octree_ptr_->coordToKey(pt);
    return search(key);
  }

  void insert(const octomap::OcTreeKey &key, NODE_TYPE *node, bool occupied) {
    std::lock_guard<std::mutex> locker(query_mutex_);
    if (occupied) {
      occ_nodes_[key] = node;
      free_nodes_.erase(key);
    } else {
      occ_nodes_[key] = node;
      free_nodes_.erase(key);
    }
    all_nodes_[key] = node;
  }

  void getNeighborKeys(const octomap::OcTreeKey &key,
                       std::vector<octomap::OcTreeKey> *neigh_keys,
                       int window_size) {
    neigh_keys->clear();
    neigh_keys->resize(window_size*window_size*window_size);
    std::vector<int> dx(window_size), dy(window_size), dz(window_size);
    for (int i = 0; i < window_size; ++i) {
      dx[i] = i - window_size / 2;
      dy[i] = i - window_size / 2;
      dz[i] = i - window_size / 2;
    }
    octomap::OcTreeKey neigh_key;
    for (int i = 0; i < window_size; ++i)
      for (int j = 0; j < window_size; ++j)
        for (int k = 0; k < window_size; ++k) {
          neigh_key[0] = key[0] + dx[i];
          neigh_key[1] = key[1] + dy[j];
          neigh_key[2] = key[2] + dz[k];

          neigh_keys->at(i*window_size + j * window_size + k * window_size)
                                                                    = neigh_key;
        }
  }

  void getNeighbor(const octomap::OcTreeKey &key,
                   std::vector<NODE_TYPE *> *neighbors,
                   std::vector<octomap::OcTreeKey> *neigh_keys,
                   int window_size) {
    neighbors->clear();
    neigh_keys->clear();
    getNeighborKeys(key, neigh_keys, window_size);
    if (neighbors == nullptr)
      return;
    neighbors->resize(neigh_keys->size());
    for (int i = 0; i < neigh_keys->size(); ++i) {
      NODE_TYPE *neigh_node = search((*neigh_keys)[i]);
      neighbors->at(i) = neigh_node;
    }
  }

  bool castRay(const point3d &origin, const point3d &directionP,
               point3d* end, bool ignoreUnknown, double maxRange) const {
    OcTreeKey current_key;
    if (!octree_ptr_->coordToKeyChecked(origin, current_key)) {
      OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");
      return false;
    }

    NODE_TYPE *startingNode = search(current_key);
    if (startingNode) {
      if (octree_ptr_->isNodeOccupied(startingNode)) {
        *end = octree_ptr_->keyToCoord(current_key);
        // ROS_ERROR("START HIT.");
        return true;
      }
    } else if (!ignoreUnknown) {
      *end = octree_ptr_->keyToCoord(current_key);
      return false;
    }

    point3d direction = directionP.normalized();
    bool max_range_set = (maxRange > 0.0);

    int step[3];
    double tMax[3];
    double tDelta[3];

    for (unsigned int i = 0; i < 3; ++i) {
      // compute step direction
      if (direction(i) > 0.0)
        step[i] = 1;
      else if (direction(i) < 0.0)
        step[i] = -1;
      else
        step[i] = 0;

      // compute tMax, tDelta
      if (step[i] != 0) {
        // corner point of voxel (in direction of ray)
        double voxelBorder = octree_ptr_->keyToCoord(current_key[i]);
        voxelBorder +=
          static_cast<double>(step[i] * octree_ptr_->getResolution() * 0.5);

        tMax[i] = (voxelBorder - origin(i)) / direction(i);
        tDelta[i] = octree_ptr_->getResolution() / fabs(direction(i));
      } else {
        tMax[i] = std::numeric_limits<double>::max();
        tDelta[i] = std::numeric_limits<double>::max();
      }
    }

    if (step[0] == 0 && step[1] == 0 && step[2] == 0) {
      OCTOMAP_ERROR("Raycasting in direction (0,0,0) is not possible!");
      return false;
    }

    // for speedup:
    double maxrange_sq = maxRange * maxRange;

    // Incremental phase
    // ---------------------------------------------------------

    bool done = false;

    while (!done) {
      unsigned int dim;

      // find minimum tMax:
      if (tMax[0] < tMax[1]) {
        if (tMax[0] < tMax[2])
          dim = 0;
        else
          dim = 2;
      } else {
        if (tMax[1] < tMax[2])
          dim = 1;
        else
          dim = 2;
      }

      // check for overflow:
      if ((step[dim] < 0 && current_key[dim] == 0) ||
          (step[dim] > 0 &&
           current_key[dim] == 2* octree_ptr_->getTreeMaxVal() - 1)) {
        OCTOMAP_WARNING("Coordinate hit bounds in dim %d, aborting raycast\n",
                        dim);
        // return border point nevertheless:
        *end = octree_ptr_->keyToCoord(current_key);
        return false;
      }

      // advance in direction "dim"
      current_key[dim] += step[dim];
      tMax[dim] += tDelta[dim];

      // generate world coords from key
      *end = octree_ptr_->keyToCoord(current_key);

      // check for maxrange:
      if (max_range_set) {
        double dist_from_origin_sq(0.0);
        for (unsigned int j = 0; j < 3; j++) {
          dist_from_origin_sq += ((end->operator()(j) - origin(j))
            * (end->operator()(j) - origin(j)));
        }
        if (dist_from_origin_sq > maxrange_sq) {
          return false;
        }
      }

      NODE_TYPE *currentNode = search(current_key);
      if (currentNode) {
        if (octree_ptr_->isNodeOccupied(currentNode)) {
          done = true;
          break;
        }
        // otherwise: node is free and valid, raycasting continues
      } else if (!ignoreUnknown) {  // no node found, this usually means we are
                                    // in "unknown" areas
        return false;
      }
    }  // end while

    return true;
  }

 private:
  NodeMap all_nodes_, free_nodes_, occ_nodes_;
  TREE_TYPE* octree_ptr_;
  mutable std::mutex query_mutex_;
};
}  // namespace octomap

#endif  // ACTIVE_MAPPING_WORLD_REPRESENTATION_OCTREE_HASH_H_
