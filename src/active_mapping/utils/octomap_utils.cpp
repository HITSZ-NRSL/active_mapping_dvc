/*
 * Created on Wed May 12 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#include "active_mapping/utils/octomap_utils.h"

namespace octomap {
void getNeighborKeys(const octomap::OcTreeKey &key,
                     std::vector<octomap::OcTreeKey>* neigh_keys,
                     int window_size) {
  neigh_keys->clear();
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

        neigh_keys->push_back(neigh_key);
      }
}

void computeCentroidAndCovarianceMatrix(const octomap::point3d_collection &pts,
                                        Eigen::Vector3d* centroid,
                                        Eigen::Matrix3d* covariance_matrix) {
  *centroid = Eigen::Vector3d::Zero();
  for (auto &pt : pts) {
    centroid->x() += pt.x();
    centroid->y() += pt.y();
    centroid->z() += pt.z();
  }
  centroid->x() /= pts.size();
  centroid->y() /= pts.size();
  centroid->z() /= pts.size();

  covariance_matrix->setZero();
  for (auto &point : pts) {
    Eigen::Vector3d pt;
    pt[0] = point.x() - (*centroid)[0];
    pt[1] = point.y() - (*centroid)[1];
    pt[2] = point.z() - (*centroid)[2];

    (*covariance_matrix)(1, 1) += pt.y() * pt.y();
    (*covariance_matrix)(1, 2) += pt.y() * pt.z();
    (*covariance_matrix)(2, 2) += pt.z() * pt.z();

    pt *= pt.x();
    (*covariance_matrix)(0, 0) += pt.x();
    (*covariance_matrix)(0, 1) += pt.y();
    (*covariance_matrix)(0, 2) += pt.z();
  }

  (*covariance_matrix)(1, 0) = (*covariance_matrix)(0, 1);
  (*covariance_matrix)(2, 0) = (*covariance_matrix)(0, 2);
  (*covariance_matrix)(2, 1) = (*covariance_matrix)(1, 2);
}

octomap::point3d toOctomap(const Eigen::Vector3d& pt) {
  octomap::point3d res;
  res.x() = pt.x();
  res.y() = pt.y();
  res.z() = pt.z();
  return res;
}


}  // namespace octomap
