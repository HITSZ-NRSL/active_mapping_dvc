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

#include "movements/geometry_pose.h"

namespace movements {

Pose::Pose() {}

Pose::Pose(Eigen::Vector3d _position, Eigen::Quaterniond _orientation)
    : position(_position), orientation(_orientation) {}

geometry_msgs::Pose Pose::toROSPose() const {
  geometry_msgs::Pose res;
  res.orientation.w = orientation.w();
  res.orientation.x = orientation.x();
  res.orientation.y = orientation.y();
  res.orientation.z = orientation.z();

  res.position.x = position.x();
  res.position.y = position.y();
  res.position.z = position.z();

  return res;
}

Eigen::Isometry3d Pose::toEigenPose() const {
  Eigen::Isometry3d res = Eigen::Isometry3d::Identity();
  res.rotate(this->orientation);
  res.translation() = this->position;
  // TODO(caoming): Evalutate it.
  return res;
}

Pose Pose::transTo(const Eigen::Isometry3d &transform) {
  Eigen::Isometry3d cur = Eigen::Isometry3d::Identity();
  cur = transform * cur;
  cur.rotate(orientation);
  cur.translation() = cur.translation() + position;

  Pose res;
  res.position = cur.translation();
  res.orientation = Eigen::Quaterniond(cur.rotation());

  return res;
}

bool Pose::operator!=(const movements::Pose &_to_compare) {
  return (position(0) != _to_compare.position(0)) ||
         (position(1) != _to_compare.position(1)) ||
         (position(2) != _to_compare.position(2)) ||
         (orientation.x() != _to_compare.orientation.x()) ||
         (orientation.y() != _to_compare.orientation.y()) ||
         (orientation.z() != _to_compare.orientation.z()) ||
         (orientation.w() != _to_compare.orientation.w());
}

bool Pose::operator==(const movements::Pose &_to_compare) {
  return !operator!=(_to_compare);
}

Eigen::Matrix4d Pose::matrix() const {
  Eigen::Matrix4d result = Eigen::Matrix4d::Zero();
  result.block(0, 0, 3, 3) = orientation.matrix();
  result.block(0, 3, 3, 1) = position.matrix();
  result(3, 3) = 1;
  return result;
}

// movements::Pose Pose::operator+( movements::RelativeMovement _second )
// {
//   return _second.applyToBasePose(*this);
// }

// Pose Pose::operator+( movements::CombinedRelativeMovement  _second )
// {
//   return _second.applyToBasePose(*this);
// }

// Pose& Pose::operator+=( movements::RelativeMovement  _second )
// {
//   *this = _second.applyToBasePose(*this);
//   return *this;
// }

// Pose& Pose::operator+=( movements::CombinedRelativeMovement  _second )
// {
//   *this = _second.applyToBasePose(*this);
//   return *this;
// }

}  // namespace movements

std::ostream &operator<<(std::ostream &_out, movements::Pose &_pose) {
  _out << "Pose:\n";
  _out << "  position: \n";
  _out << "    x: " << _pose.position.x() << "\n";
  _out << "    y: " << _pose.position.y() << "\n";
  _out << "    z: " << _pose.position.z() << "\n";
  _out << "  orientation: \n";
  _out << "    x: " << _pose.orientation.x() << "\n";
  _out << "    y: " << _pose.orientation.y() << "\n";
  _out << "    z: " << _pose.orientation.z() << "\n";
  _out << "    w: " << _pose.orientation.w() << "\n";
  return _out;
}
