/*
 * Created on Sun Apr 25 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#ifndef SRC_ACTIVE_MAPPING_INCLUDE_ACTIVE_MAPPING_UTILS_PCL_UTILS_H_
#define SRC_ACTIVE_MAPPING_INCLUDE_ACTIVE_MAPPING_UTILS_PCL_UTILS_H_

#include <pcl/point_cloud.h>
#include <utility>

namespace pcl_utils {
template <typename PointT>

  const Eigen::Isometry3d &cur_pose, const double cur_delta_t,
  const pcl::PointCloud<PointT> &pc) {
  pcl::PointCloud<PointT> res;
  res.points.resize(pc.size());

  Eigen::Isometry3d incre_T = sensor_pose.inverse() * cur_pose;
  Eigen::AngleAxisd cur_angle_axis;
  cur_angle_axis.fromRotationMatrix(incre_T.rotation());
  Eigen::Vector3d cur_omega =
      cur_angle_axis.axis() / cur_angle_axis.axis().norm();
  double cur_theta = cur_angle_axis.angle();
  Eigen::Matrix3d omega_hat;
  omega_hat.setZero();
  omega_hat(0, 1) = -cur_omega(2);
  omega_hat(1, 0) = cur_omega(2);
  omega_hat(0, 2) = cur_omega(1);
  omega_hat(2, 0) = -cur_omega(1);
  omega_hat(1, 2) = -cur_omega(0);
  omega_hat(2, 1) = cur_omega(0);
  Eigen::Matrix3d omega_hat_sq2 = omega_hat * omega_hat;

#pragma omp parallel for
  for (int i = 0; i < pc.size(); ++i) {
    double inter_s =
        (pc[i].timestamp - pc[0].timestamp) / (cur_delta_t - pc[0].timestamp);
    double inter_theta = cur_theta * inter_s;
    /* Rodrigues */
    Eigen::Matrix3d inter_R = Eigen::Matrix3d::Identity() +
                              sin(inter_theta) * omega_hat +
                              (1 - cos(inter_theta)) * omega_hat_sq2;
    /* Slerp */
    // Eigen::Quaterniond incre_R_q(incre_T.rotation());
    // Eigen::Quaterniond inter_R_q =
    // Eigen::Quaterniond::Identity().slerp(inter_s, incre_R_q); Eigen::Matrix3d
    // inter_R = inter_R_q.matrix();

    Eigen::Vector3d inter_t = incre_T.translation() * inter_s;
    Eigen::Vector3d point_curr(pc[i].x, pc[i].y, pc[i].z);
    Eigen::Vector3d point_w =
        sensor_pose.rotation() * (inter_R * point_curr + inter_t) +
        sensor_pose.translation();
    res[i].x = point_w.x();
    res[i].y = point_w.y();
    res[i].z = point_w.z();
  }
  return res;
}
}  // namespace pcl_utils

#endif  // SRC_ACTIVE_MAPPING_INCLUDE_ACTIVE_MAPPING_UTILS_PCL_UTILS_H_
