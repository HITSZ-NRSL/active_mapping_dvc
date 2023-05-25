/*
 * Created on Mon Nov 30 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#ifndef ACTIVE_MAPPING_UTILS_OCTOMAP_UTILS_H_
#define ACTIVE_MAPPING_UTILS_OCTOMAP_UTILS_H_

#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <utility>
#include "active_mapping/world_representation/ig_tree.h"

namespace octomap {
/**
 * @brief Convert pcl pointcloud into octomap pointcloud
 *
 * @tparam PointT pcl point type
 * @param pcl_pc pcl pointcloud
 * @param octomap_pc octomap pointcloud
 */
template <typename PointT>
void pclToOctomap(const pcl::PointCloud<PointT> &pcl_pc,
                  octomap::Pointcloud *octomap_pc) {
  for (int i = 0; i < pcl_pc.size(); i++) {
    octomap::point3d pt(pcl_pc[i].x, pcl_pc[i].y, pcl_pc[i].z);
    octomap_pc->push_back(pt);
  }
}

/**
 * @brief Get neighbors' keys of a specific node
 *
 * @tparam TREE_TYPE octree type
 * @tparam NODE_TYPE octree node type
 * @param tree octree
 * @param key node's key
 * @param neigh_keys neigh's key
 */
void getNeighborKeys(const octomap::OcTreeKey &key,
                     std::vector<octomap::OcTreeKey> *neigh_keys,
                     int window_size = 3);

/**
 * @brief Get the Neighbors of a specific node
 *
 * @tparam TreeType Octree type
 * @tparam NodeType Octree node type
 * @param tree Octree
 * @param key The node's key
 * @param neighbors
 */
template <typename NODE_TYPE>
void getNeighbor(const octomap::OccupancyOcTreeBase<NODE_TYPE> &tree,
                 const octomap::OcTreeKey &key,
                 std::vector<NODE_TYPE *> *neighbors,
                 std::vector<octomap::OcTreeKey> *neigh_keys,
                 int window_size = 3) {
  getNeighborKeys(key, neigh_keys, window_size);
  if (neighbors == nullptr)
    return;
  for (int i = 0; i < neigh_keys->size(); ++i) {
    NODE_TYPE *neigh_node = tree.search((*neigh_keys)[i]);
    neighbors->push_back(neigh_node);
  }
}

template <typename NODE_TYPE>
void getNeighborSix(const octomap::OccupancyOcTreeBase<NODE_TYPE> &tree,
                    const octomap::OcTreeKey &key,
                    std::vector<NODE_TYPE *> *neighbors,
                    std::vector<octomap::OcTreeKey> *neigh_keys) {
  static std::vector<int> dx = {-1, 1, 0, 0, 0, 0};
  static std::vector<int> dy = {0, 0, -1, 1, 0, 0};
  static std::vector<int> dz = {0, 0, 0, 0, -1, 1};
  for (int i = 0; i < dx.size(); ++i) {
    octomap::OcTreeKey neigh_key;
    neigh_key[0] = dx[i] + key[0];
    neigh_key[1] = dy[i] + key[1];
    neigh_key[2] = dz[i] + key[2];
    neigh_keys->push_back(neigh_key);
  }
  if (neighbors == nullptr)
    return;
  for (int i = 0; i < neigh_keys->size(); ++i) {
    NODE_TYPE *neigh_node = tree.search((*neigh_keys)[i]);
    neighbors->push_back(neigh_node);
  }
}

/**
 * @brief Get neighbor keys of a specific node in a specific direction
 *
 * @tparam TREE_TYPE
 * @tparam NODE_TYPE
 * @param tree
 * @param key
 * @param neighbors
 * @param neigh_keys
 * @param dir direction
 * @param window_size
 */
template <typename NODE_TYPE>
void getNeighborOfDirection(const octomap::OccupancyOcTreeBase<NODE_TYPE> &tree,
                            const octomap::OcTreeKey &key,
                            const Eigen::Vector3d &dir,
                            std::vector<NODE_TYPE *> *neighbors,
                            std::vector<octomap::OcTreeKey> *neigh_keys,
                            int window_size = 3) {
  octomap::OcTreeKey neigh_key;
  for (int i = -window_size / 2; i <= window_size / 2; ++i) {
    neigh_key[0] = round(key[0] + i * dir[0]);
    neigh_key[1] = round(key[1] + i * dir[1]);
    neigh_key[2] = round(key[2] + i * dir[2]);
    neigh_keys->push_back(neigh_key);
    auto neigh_node = tree.search(neigh_key);
    neighbors->push_back(neigh_node);
  }
}

/**
 * @brief Given point3d collection, compute the point set's centroid and
 * covariance matrix
 *
 * @param pts point set.
 * @param centroid point set's centroid
 * @param covariance_matrix point set's covariance matrix
 */
void computeCentroidAndCovarianceMatrix(const octomap::point3d_collection &pts,
                                        Eigen::Vector3d *centroid,
                                        Eigen::Matrix3d *covariance_matrix);

template <typename NODE_TYPE> bool isNodeUnknown(NODE_TYPE *node_ptr) {
  if (node_ptr == NULL)
    return true;
  else
    return false;
}

template <typename NODE_TYPE>
std::vector<Eigen::Vector3d>
computeNormals(const octomap::OccupancyOcTreeBase<NODE_TYPE> &tree,
               const octomap::OcTreeKey &key, const int window_size = 3) {
  std::vector<NODE_TYPE *> neighs;
  std::vector<octomap::OcTreeKey> neigh_keys;
  getNeighbor(tree, key, &neighs, &neigh_keys, window_size);

  std::vector<octomap::point3d> neigh_pts;
  for (int i = 0; i < neigh_keys.size(); ++i) {
    if (isNodeUnknown(neighs[i])) {
      continue;
    }

    neigh_pts.push_back(tree.keyToCoord(neigh_keys[i]));
  }

  Eigen::Vector3d centroid;
  Eigen::Matrix3d corvariance_matrix;
  computeCentroidAndCovarianceMatrix(neigh_pts, &centroid, &corvariance_matrix);

  std::vector<Eigen::Vector3d> normals(3);
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      corvariance_matrix, Eigen::DecompositionOptions::ComputeFullU);
  for (int i = 0; i < 3; ++i) {
    normals[i](0) = svd.matrixU().col(i)(0);
    normals[i](1) = svd.matrixU().col(i)(1);
    normals[i](2) = svd.matrixU().col(i)(2);
  }

  return normals;
}

template <typename data_type>
octomap::point3d toOctomap(const Eigen::Matrix<data_type, 3, 1> &pt) {
  octomap::point3d res;
  res.x() = pt.x();
  res.y() = pt.y();
  res.z() = pt.z();
  return res;
}

octomap::point3d toOctomap(const Eigen::Vector3d& pt);

template<typename scalar>
Eigen::Matrix<scalar, 3, 1> octomapPointToEigenVector(octomap::point3d pt) {
  Eigen::Matrix<scalar, 3, 1> res;
  res.x() = pt.x();
  res.y() = pt.y();
  res.z() = pt.z();
  return res;
}

}  // namespace octomap

#endif  // ACTIVE_MAPPING_UTILS_OCTOMAP_UTILS_H_
