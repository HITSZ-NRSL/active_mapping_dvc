/* Copyright (c) 2015, Stefan Isler, islerstefan@bluewin.ch
*
This file is part of movements, a library for representations and calculations
of movements in space,

movements is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
movements is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with movements. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ACTIVE_MAPPING_MOVEMENTS_GEOMETRY_POSE_H_
#define ACTIVE_MAPPING_MOVEMENTS_GEOMETRY_POSE_H_

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <geometry_msgs/Pose.h>
#include <vector>

namespace movements {
/** class to represent a geometry pose in 3d space */
class Pose {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Pose();
  Pose(Eigen::Vector3d _position, Eigen::Quaterniond _orientation);

  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;

  bool operator!=(const movements::Pose &_to_compare);
  bool operator==(const movements::Pose &_to_compare);

  geometry_msgs::Pose toROSPose() const;

  Eigen::Isometry3d toEigenPose() const;

  Eigen::Matrix4d matrix() const;

  /**
   * @brief
   *
   * @param transform
   * @return Pose
   */
  Pose transTo(const Eigen::Isometry3d &transform);

};

typedef std::vector<movements::Pose, Eigen::aligned_allocator<movements::Pose>>
    PoseVector;
}  // namespace movements

std::ostream &operator<<(std::ostream &_out, movements::Pose &_pose);

#endif  // ACTIVE_MAPPING_MOVEMENTS_GEOMETRY_POSE_H_
