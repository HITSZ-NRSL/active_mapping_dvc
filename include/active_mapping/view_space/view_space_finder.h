/*
 * Created on Mon Nov 16 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#pragma once
#include <octomap/octomap_types.h>
#include <string>

#include "active_mapping/view_space/view.h"
#include "active_mapping/world_representation/world_representation.h"
#include <nav_msgs/Path.h>

namespace active_mapping {
class ViewSpaceFinder {
 public:

  virtual void setWorldRepresentation(WorldRepresentation::Ptr wr) = 0;

  virtual nav_msgs::Path getFrontiersAndViewpoints(const Eigen::Isometry3d &sensor_pose,
													const octomap::Boundingbox &update_bbox) = 0;	
};
};  // namespace active_mapping

