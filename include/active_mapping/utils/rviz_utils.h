/*
 * Created on Wed Dec 16 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#ifndef ACTIVE_MAPPING_UTILS_RVIZ_UTILS_H_
#define ACTIVE_MAPPING_UTILS_RVIZ_UTILS_H_

#include <geometry_msgs/PoseArray.h>
#include <octomap/octomap_types.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <vector>
#include "active_mapping/view_space/view.h"
#include "active_mapping/utils/bounding_box.h"
#include "active_mapping/view_space/view.h"

namespace rviz_utils {
geometry_msgs::PoseArray
viewSpaceToPoseArray(const active_mapping::ViewSpace &view_space,
                     const std::string &frame_id, const ros::Time &time_stamp);

// visualization_msgs::Marker
// raySetToMarker(const active_mapping::RayCaster::RaySet &ray_set,
//                const double max_range, const std::string &frame_id,
//                const ros::Time &time_stamp);

visualization_msgs::MarkerArray
displayScoresInPoses(const std::vector<geometry_msgs::Pose>& positions,
                         const std::vector<double>& scores);

visualization_msgs::Marker drawBoundingbox(const octomap::Boundingbox &bbox);
};  // namespace rviz_utils

#endif  // ACTIVE_MAPPING_UTILS_RVIZ_UTILS_H_
