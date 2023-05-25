/*
 * Created on Mon Nov 16 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#ifndef SIMPLE_VIEW_SPACE_FINDER_H__
#define SIMPLE_VIEW_SPACE_FINDER_H__

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include "movements/geometry_pose.h"
#include "view_space/view_space_finder.h"

namespace active_mapping {
template <typename NODE_TYPE>
class SimpleViewSpaceFinder : ViewSpaceFinder<NODE_TYPE> {
 public:
  virtual bool loadFromFile(const std::string &file_path) {
    std::ifstream input(file_path);
    std::string line_str;
    auto split = [](std::string s, char delim, std::vector<std::string> &strs) {
      size_t pos = s.find(delim);
      size_t initialPos = 0;
      strs.clear();

      // Decompose statement
      while (pos != std::string::npos) {
        strs.push_back(s.substr(initialPos, pos - initialPos));
        initialPos = pos + 1;

        pos = s.find(delim, initialPos);
      }

      // Add the last one
      strs.push_back(
          s.substr(initialPos, std::min(pos, s.size()) - initialPos + 1));
    };
    /* Skip first line */
    std::getline(input, line_str);
    while (std::getline(input, line_str)) {
      std::vector<std::string> strs;
      split(line_str, ' ', strs);
      movements::Pose pose;
      pose.position.x() = std::stod(strs[0]);
      pose.position.y() = std::stod(strs[1]);
      pose.position.z() = std::stod(strs[2]);
      pose.orientation.x() = std::stod(strs[3]);
      pose.orientation.y() = std::stod(strs[4]);
      pose.orientation.z() = std::stod(strs[5]);
      pose.orientation.w() = std::stod(strs[6]);

      Eigen::AngleAxisd rotation_vector(pose.orientation);
      /* Make it vertical to z axis*/
      // rotation_vector.axis().z() = 0;
      // pose.orientation = Eigen::Quaterniond(rotation_vector);

      view_space_.emplace_back(pose, 0);
    }
  }
  virtual ViewSpace &
  getViewSpace(const octomap::point3d &sensor_origin = octomap::point3d()) {
    return view_space_;
  }

 private:
  ViewSpace view_space_;
};
};  // namespace active_mapping
#endif
