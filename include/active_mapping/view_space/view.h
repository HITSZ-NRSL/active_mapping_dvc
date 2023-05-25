/*
 * Created on Mon Nov 16 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#ifndef ACTIVE_MAPPING_VIEW_SPACE_VIEW_H_
#define ACTIVE_MAPPING_VIEW_SPACE_VIEW_H_

#include <vector>

#include "active_mapping/movements/geometry_pose.h"
#include "active_mapping/world_representation/ig_tree.h"

namespace active_mapping {
class View {
 public:
  typedef uint32_t IdType;

  View() : pose_(), sensor_pitch_(0), yaw_(0), is_reachable_(true) {}
  View(const double x, const double y, const double z, const double yaw, const double sensor_pitch);
  View(const Eigen::Vector3d &position, const double yaw, const double sensor_pitch);
  View(const movements::Pose &pose, double sensor_pitch) : pose_(pose), sensor_pitch_(sensor_pitch) {}
  movements::Pose &pose() { return pose_; }
  movements::Pose pose2d() const;
  const movements::Pose pose() const { return pose_; }
  const Eigen::Vector3d &origin() const { return pose_.position; }
  const double sensorPitch() const { return sensor_pitch_; }
  void setSensorPitch(double pitch) { sensor_pitch_ = pitch; }
  const double yaw() const { return yaw_; }
  bool reachable() { return is_reachable_; }

  void setFrontierId(uint32_t frontier_id) { frontier_id_ = frontier_id; }
  int getFrontierId() const { return frontier_id_; }

  double getCoverage() { return coverage_; }
  void setCoverage(double coverage) { coverage_ = coverage; }

  void setViewScore(double view_score) { view_score_ = view_score; }

  double getViewScore() const { return view_score_; }

 private:
  movements::Pose pose_;
  double sensor_pitch_;
  double yaw_;
  bool is_reachable_;
  int frontier_id_ = -1;
  double coverage_ = 0;
  double view_score_ = 0;
};

typedef std::vector<View> ViewSpace;
}  // namespace active_mapping

#endif  // ACTIVE_MAPPING_VIEW_SPACE_VIEW_H_
