/*
 * Created on Wed Dec 16 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include "utils/rviz_utils.h"

namespace rviz_utils {

geometry_msgs::PoseArray
viewSpaceToPoseArray(const active_mapping::ViewSpace &view_space,
                     const std::string &frame_id, const ros::Time &time_stamp) {
  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = frame_id;
  pose_array.header.stamp = time_stamp;
  pose_array.poses.resize(view_space.size());
  for (int i = 0; i < view_space.size(); ++i) {
    pose_array.poses[i].position.x = view_space[i].pose().position.x();
    pose_array.poses[i].position.y = view_space[i].pose().position.y();
    pose_array.poses[i].position.z = view_space[i].pose().position.z();

    pose_array.poses[i].orientation.x = view_space[i].pose().orientation.x();
    pose_array.poses[i].orientation.y = view_space[i].pose().orientation.y();
    pose_array.poses[i].orientation.z = view_space[i].pose().orientation.z();
    pose_array.poses[i].orientation.w = view_space[i].pose().orientation.w();
  }

  return pose_array;
}

// visualization_msgs::Marker
// raySetToMarker(const active_mapping::RayCaster::RaySet &ray_set,
//                const double max_range, const std::string &frame_id,
//                const ros::Time &time_stamp) {
//   visualization_msgs::Marker line_msg;
//   line_msg.header.frame_id = frame_id;
//   line_msg.ns = "lines";
//   line_msg.action = visualization_msgs::Marker::MODIFY;
//   line_msg.id = 2;
//   line_msg.type = visualization_msgs::Marker::LINE_LIST;
//   line_msg.scale.x = 0.01;
//   line_msg.scale.y = 0.01;
//   line_msg.scale.z = 0.01;
//   line_msg.color.r = 0;
//   line_msg.color.g = 0;
//   line_msg.color.b = 1;
//   line_msg.color.a = 1.0;

//   for (int i = 0; i < ray_set.size(); ++i) {
//     geometry_msgs::Point origin, end_point;
//     origin.x = ray_set[i].origin.x();
//     origin.y = ray_set[i].origin.y();
//     origin.z = ray_set[i].origin.z();
//     line_msg.points.push_back(origin);
//     Eigen::Vector3d d = max_range * ray_set[i].direction;
//     end_point.x = origin.x + d.x();
//     end_point.y = origin.y + d.y();
//     end_point.z = origin.z + d.z();
//     line_msg.points.push_back(end_point);
//   }

//   return move(line_msg);
// }

visualization_msgs::Marker drawBoundingbox(const octomap::Boundingbox& bbox) {
  visualization_msgs::Marker bbox_msg;
  bbox_msg.ns = "lines";
  bbox_msg.action = visualization_msgs::Marker::MODIFY;
  bbox_msg.id = 2;
  bbox_msg.type = visualization_msgs::Marker::LINE_LIST;
  bbox_msg.scale.x = 0.1;
  bbox_msg.scale.y = 0.1;
  bbox_msg.scale.z = 0.1;
  bbox_msg.color.r = 1;
  bbox_msg.color.g = 0;
  bbox_msg.color.b = 0;
  bbox_msg.color.a = 1.0;

  if (bbox.isReset()) {
    return bbox_msg;
  }

  geometry_msgs::Point flu;  /* front left up */
  flu.x = bbox.maxX();
  flu.y = bbox.maxY();
  flu.z = bbox.maxZ();

  geometry_msgs::Point fru;  /* front right up */
  fru = flu;
  fru.y = bbox.minY();

  geometry_msgs::Point blu;  /* behind left up */
  blu = flu;
  blu.x = bbox.minX();

  geometry_msgs::Point bru;  /* behind left up */
  bru = blu;
  bru.y = bbox.minY();

  geometry_msgs::Point fld;  /* front left down */
  fld = flu;
  fld.z = bbox.minZ();

  geometry_msgs::Point frd;  /* front right down */
  frd = fld;
  frd.y = bbox.minY();

  geometry_msgs::Point bld;  /* behind left down */
  bld = fld;
  bld.x = bbox.minX();

  geometry_msgs::Point brd;  /* behind right down */
  brd = bld;
  brd.y = bbox.minY();

  std::vector<geometry_msgs::Point> points =
    {blu, flu, blu, bru, blu, bld, flu, fld, flu, fru, fru, bru, fru, frd,
     frd, brd, frd, fld, brd, bru, brd, bld, bld, fld};
  for (int i = 0; i < points.size(); ++i) {
    bbox_msg.points.push_back(points[i]);
  }
  return bbox_msg;
}

visualization_msgs::MarkerArray
displayScoresInPoses(const std::vector<geometry_msgs::Pose>& poses,
                         const std::vector<double>& scores) {
  visualization_msgs::MarkerArray result;
  for (int i = 0; i < poses.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.ns = "texts";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = poses[i];
    marker.id = i;
    marker.scale.z = 0.4;
    marker.color.a = 1;
    marker.color.r = 255;
    marker.color.g = 255;
    marker.color.b = 255;
    marker.text = std::to_string(scores[i]);
    result.markers.push_back(marker);
  }
  return result;
}
}  // namespace rviz_utils
