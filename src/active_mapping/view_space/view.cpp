/*
 * Created on Fri Dec 11 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include "view_space/view.h"

namespace active_mapping {

View::View(const double x, const double y, const double z, const double yaw,
  const double sensor_pitch) :sensor_pitch_(sensor_pitch), yaw_(yaw) {
  pose_.position.x() = x;
  pose_.position.y() = y;
  pose_.position.z() = z;
  pose_.orientation = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-sensor_pitch, Eigen::Vector3d::UnitY());
}
View::View(const Eigen::Vector3d& position, const double yaw,
    const double sensor_pitch)
: sensor_pitch_(sensor_pitch), yaw_(yaw) {
    pose_.position = position;
    /* We define that when the sensor look something higher than itself, 
        the pitch angle is positive otherwise it is negative.
    */
    pose_.orientation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(-sensor_pitch, Eigen::Vector3d::UnitY());
}

movements::Pose View::pose2d() const {
    movements::Pose pose_2d;
    pose_2d.position = pose_.position;
    pose_2d.orientation.w() = cos(yaw_ / 2);
    pose_2d.orientation.x() = 0;
    pose_2d.orientation.y() = 0;
    pose_2d.orientation.z() = sin(yaw_/2);
    return pose_2d;
}
}  // namespace active_mapping
