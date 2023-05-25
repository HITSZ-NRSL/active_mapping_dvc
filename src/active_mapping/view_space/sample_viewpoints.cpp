#include "active_mapping/view_space/sample_viewpoints.h"
#include "active_mapping/aster_query.h"

#pragma region "visualization"

visualization_msgs::Marker SamplingRegion::drawColorBoundingbox(bbox_t bbox, float r, float g, float b,
                                                                std::string ns) {
  visualization_msgs::Marker bbox_msg;
  bbox_msg.header.frame_id = world_frame_;
  bbox_msg.header.stamp = ros::Time::now();
  bbox_msg.color.r = r;
  bbox_msg.color.b = b;
  bbox_msg.color.g = g;
  bbox_msg.ns = ns;
  bbox_msg.action = visualization_msgs::Marker::MODIFY;
  bbox_msg.id = 1;
  bbox_msg.type = visualization_msgs::Marker::LINE_LIST;
  bbox_msg.scale.x = 0.1;
  bbox_msg.scale.y = 0.1;
  bbox_msg.scale.z = 0.1;
  bbox_msg.color.a = 1.0;

  geometry_msgs::Point flu; /* front left up */
  flu.x = bbox.maxX;
  flu.y = bbox.maxY;
  flu.z = bbox.maxZ;

  geometry_msgs::Point fru; /* front right up */
  fru = flu;
  fru.y = bbox.minY;

  geometry_msgs::Point blu; /* behind left up */
  blu = flu;
  blu.x = bbox.minX;

  geometry_msgs::Point bru; /* behind left up */
  bru = blu;
  bru.y = bbox.minY;

  geometry_msgs::Point fld; /* front left down */
  fld = flu;
  fld.z = bbox.minZ;

  geometry_msgs::Point frd; /* front right down */
  frd = fld;
  frd.y = bbox.minY;

  geometry_msgs::Point bld; /* behind left down */
  bld = fld;
  bld.x = bbox.minX;

  geometry_msgs::Point brd; /* behind right down */
  brd = bld;
  brd.y = bbox.minY;

  std::vector<geometry_msgs::Point> points = {blu, flu, blu, bru, blu, bld, flu, fld, flu, fru, fru, bru,
                                              fru, frd, frd, brd, frd, fld, brd, bru, brd, bld, bld, fld};
  for (int i = 0; i < points.size(); ++i) {
    bbox_msg.points.push_back(points[i]);
  }

  bbox_pub.publish(bbox_msg);
  return bbox_msg;
}

std::vector<visualization_msgs::Marker> SamplingRegion::drawViewpointMarkers(nav_msgs::Path path) {
  std::vector<visualization_msgs::Marker> marker_vector;
  visualization_msgs::Marker marker_msg;
  marker_msg.header.frame_id = world_frame_;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.ns = "route";
  marker_msg.action = visualization_msgs::Marker::DELETEALL;
  marker_msg.type = visualization_msgs::Marker::ARROW;
  marker_msg.scale.x = 2;  // length
  marker_msg.scale.y = 0.15;
  marker_msg.scale.z = 0.15;  // radius
  marker_vector.push_back(marker_msg);

  marker_msg.action = visualization_msgs::Marker::MODIFY;
  for (int i = 1; i < path.poses.size(); i++) {
    marker_msg.id = i;
    marker_msg.pose = path.poses.at(i).pose;
    marker_msg.color.r = 1;
    marker_msg.color.g = 0;
    marker_msg.color.b = 0;
    marker_msg.color.a = 1.0;
    marker_vector.push_back(marker_msg);
  }

  for (int j = 0; j < marker_vector.size(); j++) {
    view_points_pub.publish(marker_vector[j]);
  }
  path_pub.publish(path);
  return marker_vector;
}

std::vector<visualization_msgs::Marker> SamplingRegion::drawFrontierNormal(int clear) {
  std::vector<visualization_msgs::Marker> marker_vector;
  visualization_msgs::Marker marker_msg;
  marker_msg.header.frame_id = world_frame_;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.ns = "normal_vertices";
  marker_msg.action = visualization_msgs::Marker::DELETEALL;
  marker_vector.push_back(marker_msg);
  frontier_normal_pub.publish(marker_msg);
  if (clear == 1) {
    return marker_vector;
  }

  marker_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.type = visualization_msgs::Marker::ARROW;
  marker_msg.scale.x = 2;  // length
  marker_msg.scale.y = 0.1;
  marker_msg.scale.z = 0.1;  // radius

  int cnt = 0;
  for (int i = 0; i < frontier_vectors_.size(); i++) {
    for (int j = 0; j < frontier_vectors_[i].size(); j++) {
      marker_msg.id = cnt++;
      marker_msg.pose.position.x =
          frontier_vectors_[i][j][6];  // maxX, maxY, maxZ, minX, minY, minZ, x, y, z, qw, qx, qy, qz, r, g, b
      marker_msg.pose.position.y = frontier_vectors_[i][j][7];
      marker_msg.pose.position.z = frontier_vectors_[i][j][8];
      marker_msg.pose.orientation.w = frontier_vectors_[i][j][9];
      marker_msg.pose.orientation.x = frontier_vectors_[i][j][10];
      marker_msg.pose.orientation.y = frontier_vectors_[i][j][11];
      marker_msg.pose.orientation.z = frontier_vectors_[i][j][12];
      marker_msg.color.r = ((int)frontier_vectors_[i][j][13] % 256) / 255.0;
      marker_msg.color.g = ((int)frontier_vectors_[i][j][14] % 256) / 255.0;
      marker_msg.color.b = ((int)frontier_vectors_[i][j][15] % 256) / 255.0;
      marker_msg.color.a = 1.0;
      marker_vector.push_back(marker_msg);
      frontier_normal_pub.publish(marker_msg);
    }
  }

  marker_msg.type = visualization_msgs::Marker::SPHERE;
  marker_msg.scale.x = 0.3;
  marker_msg.scale.y = 1;
  marker_msg.scale.z = 1;

  for (int i = 0; i < frontier_poses_.size(); i++) {
    marker_msg.id = cnt++;
    marker_msg.pose.position.x =
        frontier_poses_[i][6];  // maxX, maxY, maxZ, minX, minY, minZ, x, y, z, qw, qx, qy, qz, r, g, b
    marker_msg.pose.position.y = frontier_poses_[i][7];
    marker_msg.pose.position.z = frontier_poses_[i][8];
    marker_msg.pose.orientation.w = frontier_poses_[i][9];
    marker_msg.pose.orientation.x = frontier_poses_[i][10];
    marker_msg.pose.orientation.y = frontier_poses_[i][11];
    marker_msg.pose.orientation.z = frontier_poses_[i][12];
    marker_msg.color.r = ((int)frontier_poses_[i][13] % 256) / 255.0;
    marker_msg.color.g = ((int)frontier_poses_[i][14] % 256) / 255.0;
    marker_msg.color.b = ((int)frontier_poses_[i][15] % 256) / 255.0;
    marker_msg.color.a = 1.0;
    marker_vector.push_back(marker_msg);
    frontier_normal_pub.publish(marker_msg);
  }
  return marker_vector;
}

std::vector<visualization_msgs::Marker> SamplingRegion::drawSamplingRegion() {
  visualization_msgs::Marker marker_msg;
  marker_msg.header.frame_id = world_frame_;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.ns = "sample_points";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::POINTS;
  marker_msg.action = visualization_msgs::Marker::MODIFY;
  marker_msg.color.r = 200.0 / 255;
  marker_msg.color.g = 50.0 / 255;
  marker_msg.color.b = 200.0 / 255;  // purple (0.5, 0.5, 1.0)
  marker_msg.color.a = 1.0;
  marker_msg.scale.x = 0.2;
  marker_msg.scale.y = 0.2;
  marker_msg.scale.z = 0.2;

  visualization_msgs::Marker marker_msg1 = marker_msg;
  marker_msg1.ns = "sample_points1";
  marker_msg1.color.r = 0.9;
  marker_msg1.color.g = 0.9;
  marker_msg1.color.b = 0.0;  // yellow (0.9, 0.9, 0)
  visualization_msgs::Marker marker_msg2 = marker_msg;
  marker_msg2.ns = "sample_points2";
  marker_msg2.color.r = 200.0 / 255;
  marker_msg2.color.g = 100.0 / 255;
  marker_msg2.color.b = 50.0 / 255;  // orange (1, 0.5, 0.3)
  visualization_msgs::Marker marker_msg3 = marker_msg;
  marker_msg3.ns = "sample_points3";
  marker_msg3.color.r = 0.5;
  marker_msg3.color.g = 0.9;
  marker_msg3.color.b = 0.1;  // green (0.5, 0.9, 0.1)

  marker_msg.points = quadrant_vector_[0];
  marker_msg1.points = quadrant_vector_[1];
  marker_msg2.points = quadrant_vector_[2];
  marker_msg3.points = quadrant_vector_[3];

  std::vector<visualization_msgs::Marker> markers_vector;
  markers_vector.push_back(marker_msg);
  markers_vector.push_back(marker_msg1);
  markers_vector.push_back(marker_msg2);
  markers_vector.push_back(marker_msg3);

  for (int j = 0; j < markers_vector.size(); j++) {
    sample_points_pub.publish(markers_vector[j]);
  }

  return markers_vector;
}

#pragma endregion

#pragma region "tools"

float SamplingRegion::calcYawBetweenVectors(Eigen::Vector3f vec1, Eigen::Vector3f vec2) {
  vec1[2] = 0;
  vec2[2] = 0;
  vec1.normalize();
  vec2.normalize();
  Eigen::Vector3f vec_cross = vec1.cross(vec2);
  float vec_dot = vec1.dot(vec2);
  float theta = calc3DAngleBetweenVectors(vec1, vec2);

  if (vec_cross.z() > 0)
    theta = 360 - theta;

  return theta;
}

float SamplingRegion::calc3DAngleBetweenVectors(Eigen::Vector3f vec1, Eigen::Vector3f vec2) {
  float cos_theta = vec1.dot(vec2) / (vec1.norm() * vec2.norm());
  float theta = fabs(acos(cos_theta) / M_PI * 180);
  return theta;
}

void SamplingRegion::updateBuildingBoundingBox(bbox_t bbox_new) {
  bbox_ = bbox_new;
  bbox_length_ = bbox_.maxX - bbox_.minX;
  bbox_width_ = bbox_.maxY - bbox_.minY;
  diagonal_length_ = sqrt(bbox_length_ * bbox_length_ + bbox_width_ * bbox_width_);
  float center_x = (bbox_.maxX + bbox_.minX) / 2.0;
  float center_y = (bbox_.maxY + bbox_.minY) / 2.0;
  bbox_center_ = Eigen::Vector2f(center_x, center_y);

  {
    float th = atan2(bbox_.maxY - bbox_.minY, bbox_.maxX - bbox_.minX) / M_PI * 180;
    side_normals_.clear();

    angle_range[0][0] = 360.0 - th;  // quadral I
    angle_range[0][1] = th;
    side_normals_.push_back(Eigen::Vector3f(1, 0, 0));

    angle_range[1][0] = th;  // quadral II
    angle_range[1][1] = 180.0 - th;
    side_normals_.push_back(Eigen::Vector3f(0, -1, 0));

    angle_range[2][0] = 180.0 - th;  // quadral III
    angle_range[2][1] = 180.0 + th;
    side_normals_.push_back(Eigen::Vector3f(-1, 0, 0));

    angle_range[3][0] = 180.0 + th;  // quadral IV
    angle_range[3][1] = 360.0 - th;
    side_normals_.push_back(Eigen::Vector3f(0, 1, 0));
  }

  {
    sampling_bbox_.maxX = bbox_.maxX + sampling_resolution_ * sampling_number_;
    sampling_bbox_.minX = bbox_.minX - sampling_resolution_ * sampling_number_;
    sampling_bbox_.maxY = bbox_.maxY + sampling_resolution_ * sampling_number_;
    sampling_bbox_.minY = bbox_.minY - sampling_resolution_ * sampling_number_;
  }
  {
    quadrant_nbv_.clear();
    float dt = 1.4 * GAP_SIZE * 2 + SHIFT_NBV_DIS * 1.4 * SAMP_ROI_RES;
    float da = 30;
    quadrant_nbv_.push_back(Eigen::Vector3f(bbox_.maxX + dt, bbox_.maxY + dt, 180 - da));
    quadrant_nbv_.push_back(Eigen::Vector3f(bbox_.maxX + dt, bbox_.minY - dt, -90 - da));
    quadrant_nbv_.push_back(Eigen::Vector3f(bbox_.minX - dt, bbox_.minY - dt, 0 - da));
    quadrant_nbv_.push_back(Eigen::Vector3f(bbox_.minX - dt, bbox_.maxY + dt, 90 - da));
  }
}

void SamplingRegion::updateBuildingBoundingBox(octomap::Boundingbox bbox) {
  bbox_t bbox_new;
  bbox_new.maxX = bbox.maxX();
  bbox_new.maxY = bbox.maxY();
  bbox_new.maxZ = bbox.maxZ();
  bbox_new.minX = bbox.minX();
  bbox_new.minY = bbox.minY();
  bbox_new.minZ = bbox.minZ();
  updateBuildingBoundingBox(bbox_new);
}

float SamplingRegion::positionToAzimuth(Eigen::Vector3f curr_position) {
  Eigen::Vector3f curr_vector =
      Eigen::Vector3f(curr_position.x() - bbox_center_.x(), curr_position.y() - bbox_center_.y(), 0);
  Eigen::Vector3f x_axis(1, 0, 0);
  float theta = calcYawBetweenVectors(x_axis, curr_vector);
  return theta;
}

int SamplingRegion::queryCurrentQuadrant(Eigen::Vector3f curr_position) {
  float theta = positionToAzimuth(curr_position);
  for (int i = 0; i < 4; i++) {
    if (i == 0) {
      if ((angle_range[i][0] < theta && theta < 360) || (0 <= theta && theta < angle_range[i][1])) {
        return i;
      }
    } else if (angle_range[i][0] < theta && theta < angle_range[i][1])
      return i;
  }
  return -1;
}

void SamplingRegion::generateSamplingRegion() {
  std::vector<geometry_msgs::Point> quadrant_points, quadrant1_points, quadrant2_points, quadrant3_points;

  for (float i = sampling_bbox_.minX + sampling_resolution_ * 0; i < sampling_bbox_.maxX - sampling_resolution_ * 0;
       i += sampling_resolution_) {
    for (float j = sampling_bbox_.minY + sampling_resolution_ * 0; j < sampling_bbox_.maxY - sampling_resolution_ * 0;
         j += sampling_resolution_) {
      if (bbox_.minX - sampling_resolution_ * GAP_SIZE <= i && i <= bbox_.maxX + sampling_resolution_ * GAP_SIZE &&
          bbox_.minY - sampling_resolution_ * GAP_SIZE <= j && j <= bbox_.maxY + sampling_resolution_ * GAP_SIZE) {
        continue;
      }
      if (!wr_->isDrivable(i, j)) {
        continue;
      }

      geometry_msgs::Point point;
      point.x = i;
      point.y = j;
      point.z = 0;
      int quad = queryCurrentQuadrant(Eigen::Vector3f(i, j, 0));
      switch (quad) {
      case 0:
        quadrant_points.push_back(point);
        break;
      case 1:
        quadrant1_points.push_back(point);
        break;
      case 2:
        quadrant2_points.push_back(point);
        break;
      case 3:
        quadrant3_points.push_back(point);
        break;
      }
    }
  }

  quadrant_vector_.clear();
  quadrant_vector_.push_back(quadrant_points);
  quadrant_vector_.push_back(quadrant1_points);
  quadrant_vector_.push_back(quadrant2_points);
  quadrant_vector_.push_back(quadrant3_points);

  {
    int row_size = (PITCH_UPPER_BOUND - 0) / PITCH_INTERVAL + 2;
    int col_size = (YAW_UPPER_BOUND * 2) / YAW_INTERVAL + 2;
    Eigen::MatrixXi score_mat(row_size, col_size);
    score_mat.setZero();

    std::vector<Eigen::MatrixXi> quad_scores, quad1_scores, quad2_scores, quad3_scores;

    for (int i = 0; i < quadrant_points.size(); i++)
      quad_scores.push_back(score_mat);
    for (int i = 0; i < quadrant1_points.size(); i++)
      quad1_scores.push_back(score_mat);
    for (int i = 0; i < quadrant2_points.size(); i++)
      quad2_scores.push_back(score_mat);
    for (int i = 0; i < quadrant3_points.size(); i++)
      quad3_scores.push_back(score_mat);

    score_vector_.clear();
    score_vector_.push_back(quad_scores);
    score_vector_.push_back(quad1_scores);
    score_vector_.push_back(quad2_scores);
    score_vector_.push_back(quad3_scores);
  }

  {
    frontier_vp_vector_.clear();
    frontier_vectors_.clear();
    std::vector<std::vector<float>> frontier_vec, frontier_vec1, frontier_vec2, frontier_vec3;
    std::vector<std::vector<Eigen::Vector3f>> frontier_vp, frontier_vp1, frontier_vp2, frontier_vp3;
    std::vector<Eigen::Vector3f> vp_dummy;

    for (int i = 0; i < frontier_poses_.size(); i++) {
      Eigen::Vector3f frontier_point(frontier_poses_[i][6], frontier_poses_[i][7], frontier_poses_[i][8]);
      Eigen::Quaternionf frontier_orientation(frontier_poses_[i][9], frontier_poses_[i][10], frontier_poses_[i][11],
                                              frontier_poses_[i][12]);
      Eigen::Vector3f frontier_normal =
          transformPoint(frontier_orientation, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(1, 0, 0));
      int side = calcFrontierSide(frontier_point);
      if (side == -1)
        continue;
      float theta =
          calc3DAngleBetweenVectors(Eigen::Vector3f(frontier_normal(0), frontier_normal(1), 0), side_normals_[side]);
      if (theta > 90) {
        frontier_normal *= -1;
        theta =
            calc3DAngleBetweenVectors(Eigen::Vector3f(frontier_normal(0), frontier_normal(1), 0), side_normals_[side]);
        if (theta > 90) {

        } else {
          frontier_orientation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(1, 0, 0), frontier_normal);
          frontier_poses_[i][9] = frontier_orientation.w();
          frontier_poses_[i][10] = frontier_orientation.x();
          frontier_poses_[i][11] = frontier_orientation.y();
          frontier_poses_[i][12] = frontier_orientation.z();
        }
      }

      std::vector<float> tmp = frontier_poses_[i];
      switch (side) {
      case 0: {
        frontier_vp.push_back(vp_dummy);
        tmp[13] = 255 * 0.5;
        tmp[14] = 255 * 0.5;
        tmp[15] = 255;
        frontier_vec.push_back(tmp);  // purple (0.5, 0.5, 1.0)
      } break;
      case 1: {
        frontier_vp1.push_back(vp_dummy);
        tmp[13] = 255 * 0.9;
        tmp[14] = 255 * 0.9;
        tmp[15] = 0;
        frontier_vec1.push_back(tmp);  // yellow (0.9, 0.9, 0)
      } break;
      case 2: {
        frontier_vp2.push_back(vp_dummy);
        tmp[13] = 255;
        tmp[14] = 255 * 0.5;
        tmp[15] = 255 * 0.3;
        frontier_vec2.push_back(tmp);  // orange (1, 0.5, 0.3)

      } break;
      case 3: {
        frontier_vp3.push_back(vp_dummy);
        tmp[13] = 255 * 0.5;
        tmp[14] = 255 * 0.9;
        tmp[15] = 255 * 0.1;
        frontier_vec3.push_back(tmp);  // green (0.5, 0.9, 0.1)
      } break;
      }
    }

    frontier_vectors_.push_back(frontier_vec);
    frontier_vectors_.push_back(frontier_vec1);
    frontier_vectors_.push_back(frontier_vec2);
    frontier_vectors_.push_back(frontier_vec3);

    frontier_vp_vector_.push_back(frontier_vp);
    frontier_vp_vector_.push_back(frontier_vp1);
    frontier_vp_vector_.push_back(frontier_vp2);
    frontier_vp_vector_.push_back(frontier_vp3);
  }
}

int SamplingRegion::calcFrontierSide(Eigen::Vector3f curr_position) {
  std::vector<float> dis;
  dis.push_back(fabs(curr_position.x() - bbox_.maxX));
  dis.push_back(fabs(curr_position.y() - bbox_.minY));
  dis.push_back(fabs(curr_position.x() - bbox_.minX));
  dis.push_back(fabs(curr_position.y() - bbox_.maxY));

  int min_idx = -1;
  float min_dis = 1e9;
  for (int i = 0; i < 4; i++) {
    if (dis[i] < min_dis) {
      min_idx = i;
      min_dis = dis[i];
    }
  }
  return min_idx;
}

geometry_msgs::Pose SamplingRegion::viewpointIndexToPose(Eigen::Vector3f vp_index, int side) {
  int index = vp_index(0);
  float pitch = 0;
  if (vp_index(1) != 0)
    pitch = -(vp_index(1) + 0.5) * PITCH_INTERVAL / 180.0 * M_PI;

  float yaw_offset = (2 - side) * 90.0;
  float yaw_comp = (vp_index(2) * YAW_INTERVAL - YAW_UPPER_BOUND) > 0 ? 1 : -1;
  float yaw =
      (-(vp_index(2) * YAW_INTERVAL - YAW_UPPER_BOUND) + yaw_offset - yaw_comp * YAW_INTERVAL / 2) / 180.0 * M_PI;

  geometry_msgs::Pose p;
  p.position.x = quadrant_vector_[side][index].x;
  p.position.y = quadrant_vector_[side][index].y;
  p.position.z = quadrant_vector_[side][index].z;

  // euler to quaternion
  Eigen::Vector3f eulerAngle(0, -PITCH_UPPER_BOUND * 2 / 3 / 180.0 * M_PI, yaw);
  Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(0), Eigen::Vector3f::UnitX()));
  Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1), Eigen::Vector3f::UnitY()));
  Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(2), Eigen::Vector3f::UnitZ()));
  Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;

  p.orientation.w = q.w();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();

  return p;
}

void SamplingRegion::recvNewInfo(nav_msgs::Path &frontiers_pose, nav_msgs::Path &frontiers_color,
                                 geometry_msgs::Pose &robot_pose, geometry_msgs::Pose &bbox) {
  frontier_poses_.clear();
  std::vector<float> tmpvec;

  for (int i = 0; i < frontiers_pose.poses.size(); i++) {
    tmpvec.clear();
    for (int j = 0; j < 6; j++)
      tmpvec.push_back(0);  // dummy: maxXYZ, minXYZ
    tmpvec.push_back(frontiers_pose.poses[i].pose.position.x);
    tmpvec.push_back(frontiers_pose.poses[i].pose.position.y);
    tmpvec.push_back(frontiers_pose.poses[i].pose.position.z);
    tmpvec.push_back(frontiers_pose.poses[i].pose.orientation.w);
    tmpvec.push_back(frontiers_pose.poses[i].pose.orientation.x);
    tmpvec.push_back(frontiers_pose.poses[i].pose.orientation.y);
    tmpvec.push_back(frontiers_pose.poses[i].pose.orientation.z);
    tmpvec.push_back(frontiers_color.poses[i].pose.position.x);
    tmpvec.push_back(frontiers_color.poses[i].pose.position.y);
    tmpvec.push_back(frontiers_color.poses[i].pose.position.z);
    frontier_poses_.push_back(tmpvec);
  }

  bbox_t bbox_new = {(float)bbox.position.x,    (float)bbox.position.y,    (float)bbox.position.z,      // max XYZ
                     (float)bbox.orientation.x, (float)bbox.orientation.y, (float)bbox.orientation.z};  // min XYZ
  updateBuildingBoundingBox(bbox_new);
}

geometry_msgs::Pose SamplingRegion::quadrantViewpointToPose(int side) {
  Eigen::Vector3f vp_index = quadrant_nbv_[side];
  geometry_msgs::Pose p;
  p.position.x = vp_index.x();
  p.position.y = vp_index.y();
  p.position.z = 0;
  float yaw = -vp_index.z() / 180.0 * M_PI;
  Eigen::Vector3f eulerAngle(0, -PITCH_UPPER_BOUND * 1 / 3 / 180.0 * M_PI, yaw);  /////////////////
  Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(0), Eigen::Vector3f::UnitX()));
  Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1), Eigen::Vector3f::UnitY()));
  Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(2), Eigen::Vector3f::UnitZ()));
  Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;

  p.orientation.w = q.w();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();

  Eigen::Vector3f rpy = toEulerAngle(q);
  return p;
}

Eigen::MatrixXf SamplingRegion::constructCostMatrix(Eigen::Vector3f curr_position,
                                                    std::vector<Eigen::Vector3f> &viewpoint_subset, int side) {
  Eigen::MatrixXf res = Eigen::MatrixXf::Zero(viewpoint_subset.size() + 1, viewpoint_subset.size() + 1);
  for (int i = 0; i < viewpoint_subset.size(); ++i) {
    Eigen::Vector3f vp(quadrant_vector_[side][viewpoint_subset[i](0)].x,
                       quadrant_vector_[side][viewpoint_subset[i](0)].y,
                       quadrant_vector_[side][viewpoint_subset[i](0)].z);
    res(0, i + 1) = (curr_position - vp).norm();
    for (int j = i + 1; j < viewpoint_subset.size(); ++j) {
      Eigen::Vector3f vp_(quadrant_vector_[side][viewpoint_subset[j](0)].x,
                          quadrant_vector_[side][viewpoint_subset[j](0)].y,
                          quadrant_vector_[side][viewpoint_subset[j](0)].z);
      active_mapping::aster_query srv;
      srv.request.origin.x = vp_.x();
      srv.request.origin.y = vp_.y();
      srv.request.origin.z = vp_.z();
      srv.request.target.x = vp.x();
      srv.request.target.y = vp.y();
      srv.request.target.z = vp.z();

      float dist = (vp - vp_).norm();
      res(i + 1, j + 1) = res(j + 1, i + 1) = dist;
    }
  }
  return res;
}

std::vector<int> SamplingRegion::solveATSPLKH(Eigen::MatrixXf &cost_matrix) {
  std::string package_path = ros::package::getPath("lkh_tsp_solver");
  std::string resource_path = package_path + "/resource";
  std::ofstream par_file(resource_path + "/single.par");
  par_file << "PROBLEM_FILE = " << resource_path << "/single.tsp\n";

  /******************** Par file ********************/
  // Specifies whether the Gain23 function is used,
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << resource_path << "/single.txt\n";
  // The total number of runs.
  par_file << "RUNS = 1\n";
  par_file.close();

  /******************** tsp file ********************/
  // Write params and cost matrix to problem file
  std::ofstream prob_file(resource_path + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB

  int dimension = cost_matrix.rows();
  std::string prob_spec = "NAME : single\n"
                          "TYPE : ATSP\n"
                          "DIMENSION : " +
                          std::to_string(dimension) +
                          "\n"
                          "EDGE_WEIGHT_TYPE : "
                          "EXPLICIT\n"
                          "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n"
                          "EDGE_WEIGHT_SECTION\n";
  prob_file << prob_spec;

  const int scale = 100;
  // Use Asymmetric TSP
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_matrix(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }
  prob_file << "EOF";
  prob_file.close();

  solveTSPLKH((resource_path + "/single.par").c_str());
  /******************** txt file ********************/
  std::vector<int> indices;
  std::ifstream res_file(resource_path + "/single.txt");
  std::string res;
  while (std::getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }

  // Read path for ATSP formulation
  while (std::getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = std::stoi(res);
    if (id == 1)  // Ignore the current state
      continue;
    if (id == -1)
      break;
    indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
  }
  res_file.close();

  return indices;
}

float SamplingRegion::calcPathLength(Eigen::Vector3f curr_position, std::vector<Eigen::Vector3f> &viewpoint_subset,
                                     std::vector<int> &indices, int side) {
  Eigen::Vector3f vp(quadrant_vector_[side][viewpoint_subset[0](0)].x, quadrant_vector_[side][viewpoint_subset[0](0)].y,
                     quadrant_vector_[side][viewpoint_subset[0](0)].z);
  float total_length = (vp - curr_position).norm();

  for (int i = 0; i < indices.size() - 1; i++) {
    Eigen::Vector3f vp1(quadrant_vector_[side][viewpoint_subset[i](0)].x,
                        quadrant_vector_[side][viewpoint_subset[i](0)].y,
                        quadrant_vector_[side][viewpoint_subset[i](0)].z);
    Eigen::Vector3f vp2(quadrant_vector_[side][viewpoint_subset[i + 1](0)].x,
                        quadrant_vector_[side][viewpoint_subset[i + 1](0)].y,
                        quadrant_vector_[side][viewpoint_subset[i + 1](0)].z);
    total_length += (vp1 - vp2).norm();
  }
  return total_length;
}

int SamplingRegion::generateViewpointVoteSequence(std::vector<Eigen::MatrixXi> &score_vector,
                                                  std::vector<Eigen::Vector3f> &viewpoint_vec) {
  for (int i = 0; i < score_vector.size(); i++) {
    Eigen::MatrixXi score_mat = score_vector[i];
    for (int j = 0; j < PITCH_UPPER_BOUND / PITCH_INTERVAL; j++) {
      for (int k = 0; k < (YAW_UPPER_BOUND / YAW_INTERVAL) * 2; k++) {
        int cnt = score_mat(j, k);
        while (cnt-- > 0) {
          for (int q = 0; q < SAMPLE_WEIGHT; q++)
            viewpoint_vec.push_back(Eigen::Vector3f(i, j, k));
        }
      }
    }
  }
  int length = viewpoint_vec.size();
  if (length <= MIN_SEQ_LENGTH) {
    length = -1;
  }
  return length;
}

#pragma endregion

#pragma region "Sampling"

SamplingRegion::SamplingRegion(bbox_t bbox_new, float resolution, float sample_num) {
  nh.getParam("active_mapping/sample_viewpoints/sample_same_point", SAMPLE_SAME_POINT);
  nh.getParam("active_mapping/sample_viewpoints/resample_times", RESAMPLE_TIMES);
  nh.getParam("active_mapping/sample_viewpoints/pitch_upper_bound", PITCH_UPPER_BOUND);
  nh.getParam("active_mapping/sample_viewpoints/pitch_interval", PITCH_INTERVAL);
  nh.getParam("active_mapping/sample_viewpoints/yaw_upper_bound", YAW_UPPER_BOUND);
  nh.getParam("active_mapping/sample_viewpoints/yaw_interval", YAW_INTERVAL);
  nh.getParam("active_mapping/sample_viewpoints/min_seq_length", MIN_SEQ_LENGTH);

  nh.getParam("active_mapping/sample_viewpoints/gap_size", GAP_SIZE);
  nh.getParam("active_mapping/sample_viewpoints/samp_roi_res", SAMP_ROI_RES);
  nh.getParam("active_mapping/sample_viewpoints/samp_roi_size", SAMP_ROI_SIZE);
  nh.getParam("active_mapping/sample_viewpoints/vote_yaw_th", VOTE_ANGLE_TH);
  nh.getParam("active_mapping/sample_viewpoints/vote_dis_th", VOTE_DIS_TH);
  nh.getParam("active_mapping/sample_viewpoints/sample_weight", SAMPLE_WEIGHT);
  nh.getParam("active_mapping/sample_viewpoints/planning_times", PLANNING_TIMES);
  nh.getParam("active_mapping/sample_viewpoints/enable_frontier_chk", FRONTIER_CHK_ENB);
  nh.getParam("active_mapping/sample_viewpoints/enable_raycast_chk", ENABLE_RAYCAST_CHK);
  nh.getParam("active_mapping/sample_viewpoints/shift_nbv_dis", SHIFT_NBV_DIS);
  nh.getParam("active_mapping/world_frame", world_frame_);

  bbox_pub = nh.advertise<visualization_msgs::Marker>("/active_mapping/world_representation/building_bbox", 100000);
  frontier_normal_pub = nh.advertise<visualization_msgs::Marker>("frontier_normal_markers", 10000);
  sample_points_pub = nh.advertise<visualization_msgs::Marker>("sample_points_markers", 10000);
  view_points_pub = nh.advertise<visualization_msgs::Marker>("view_points_markers", 10000);
  path_pub = nh.advertise<nav_msgs::Path>("vp_path", 10000);
  octomap_pub = nh.advertise<octomap_msgs::Octomap>("/world_representation/occ", 1);
  ray_casting_pub = nh.advertise<visualization_msgs::Marker>("ray_casting", 10000);
  astar_client = nh.serviceClient<active_mapping::aster_query>("aster_query");

  sampling_resolution_ = SAMP_ROI_RES;
  sampling_number_ = SAMP_ROI_SIZE;
  updateBuildingBoundingBox(bbox_new);
}

int SamplingRegion::checkFrontierAngle(Eigen::Vector3f curr_viewpoint, Eigen::Vector3f frontier_point,
                                       Eigen::Vector3f frontier_normal, Eigen::Vector3f side_normal) {
  if (FRONTIER_CHK_ENB == false)
    return 1;
  Eigen::Vector3f dis_vector = curr_viewpoint - frontier_point;
  Eigen::Vector3f dis_vector_xy(dis_vector.x(), dis_vector.y(), 0);
  float theta_dt = calc3DAngleBetweenVectors(frontier_normal, dis_vector);

  if (dis_vector_xy.norm() > VOTE_DIS_TH || theta_dt > VOTE_ANGLE_TH)
    return -1;

  return 1;
}

int SamplingRegion::calcViewAngle(Eigen::Vector3f curr_viewpoint, Eigen::Vector3f frontier_point,
                                  Eigen::Vector3f side_normal, int *pitch_index, int *yaw_index) {
  Eigen::Vector3f dis_vector = curr_viewpoint - frontier_point;
  dis_vector = -1 * dis_vector;
  Eigen::Vector3f dis_vector_xy(dis_vector.x(), dis_vector.y(), 0);

  float pitch = calc3DAngleBetweenVectors(dis_vector_xy, dis_vector);
  float yaw = calcYawBetweenVectors(-1 * side_normal, dis_vector_xy);

  if (yaw > 180)  // 0~360° --> ±180°
    yaw -= 360;

  if (fabs(yaw) > YAW_UPPER_BOUND)
    return -1;
  if (pitch < 0)
    pitch = 0;
  else if (pitch >= PITCH_UPPER_BOUND)
    pitch = PITCH_UPPER_BOUND;

  *pitch_index = round(pitch / PITCH_INTERVAL);
  *yaw_index = round((yaw + YAW_UPPER_BOUND) / YAW_INTERVAL);
  return 1;
}

bool SamplingRegion::checkVisible(Eigen::Vector3f curr_viewpoint, Eigen::Vector3f frontier_point) {
  if (ENABLE_RAYCAST_CHK == false)
    return true;
  octomap::point3d view_origin;
  view_origin.x() = curr_viewpoint.x();
  view_origin.y() = curr_viewpoint.y();
  view_origin.z() = curr_viewpoint.z();

  octomap::point3d frontier;
  frontier.x() = frontier_point.x();
  frontier.y() = frontier_point.y();
  frontier.z() = frontier_point.z();

  double ray_cast_length = (frontier - view_origin).norm();
  octomap::point3d ray_cast_dir = frontier - view_origin;
  octomap::point3d end_pt;
  bool find_end_pt = this->octree_ptr_->castRay(view_origin, ray_cast_dir, end_pt, true, ray_cast_length + 0.5);
  if (find_end_pt) {
    double ray_cast_length2 = (view_origin - end_pt).norm();
    double dis = ray_cast_length2 - ray_cast_length;
    if (dis < 0) {
      return false;
    }
  }
  return true;
}

void SamplingRegion::voteSamplePoints(int side) {
  Eigen::Vector3f side_normal = side_normals_[side];

  for (int i = 0; i < quadrant_vector_[side].size(); i++) {
    geometry_msgs::Point curr_vp = quadrant_vector_[side][i];
    Eigen::Vector3f curr_viewpoint(curr_vp.x, curr_vp.y, curr_vp.z);
    for (int j = 0; j < frontier_vectors_[side].size(); j++) {
      Eigen::Vector3f frontier_point(frontier_vectors_[side][j][6], frontier_vectors_[side][j][7],
                                     frontier_vectors_[side][j][8]);
      Eigen::Quaternionf frontier_orientation(frontier_vectors_[side][j][9], frontier_vectors_[side][j][10],
                                              frontier_vectors_[side][j][11], frontier_vectors_[side][j][12]);
      Eigen::Vector3f frontier_normal =
          transformPoint(frontier_orientation, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(1, 0, 0));

      int pitch_index, yaw_index;
      if (calcViewAngle(curr_viewpoint, frontier_point, side_normal, &pitch_index, &yaw_index) == -1)
        continue;
      if (checkVisible(curr_viewpoint, frontier_point) == false)
        continue;
      if (checkFrontierAngle(curr_viewpoint, frontier_point, frontier_normal, side_normal) == -1)
        continue;

      score_vector_[side][i](pitch_index, yaw_index)++;
      Eigen::Vector3f vp = Eigen::Vector3f(i, pitch_index, yaw_index);
      frontier_vp_vector_[side][j].push_back(vp);
    }
  }
}

void SamplingRegion::sampleVoteDecount(std::vector<std::vector<Eigen::Vector3f>> &frontier_vp_vector,
                                       std::vector<bool> &frontier_covered_state,
                                       std::vector<Eigen::MatrixXi> &score_vector, Eigen::Vector3f selected_vp) {
  int selected_vp_pitch = selected_vp(1);
  int selected_vp_yaw = selected_vp(2);
  int selected_vp_index = selected_vp(0);
  if (!SAMPLE_SAME_POINT)
    for (int i = 0; i < PITCH_UPPER_BOUND / PITCH_INTERVAL; i++) {
      for (int j = 0; j < YAW_UPPER_BOUND * 2 / YAW_INTERVAL; j++) {
        score_vector[selected_vp_index](i, j) = 0;
      }
    }
  score_vector[selected_vp_index](selected_vp_pitch, selected_vp_yaw) = 0;

  for (int i = 0; i < frontier_vp_vector.size(); i++) {
    if (frontier_covered_state[i] == true)
      continue;

    for (std::vector<Eigen::Vector3f>::iterator it = frontier_vp_vector[i].begin(); it != frontier_vp_vector[i].end();
         it++) {
      Eigen::Vector3f frontier_vp = *it;
      if (selected_vp == frontier_vp) {
        frontier_covered_state[i] = true;
        break;
      }
    }

    if (frontier_covered_state[i] == true) {
      for (std::vector<Eigen::Vector3f>::iterator it = frontier_vp_vector[i].begin(); it != frontier_vp_vector[i].end();
           it++) {
        Eigen::Vector3f frontier_vp = *it;
        int pitch_index = frontier_vp(1);
        int yaw_index = frontier_vp(2);
        int vp_index = frontier_vp(0);

        if (score_vector[vp_index](pitch_index, yaw_index) > 0)
          score_vector[vp_index](pitch_index, yaw_index)--;
      }
    }
  }
}

nav_msgs::Path SamplingRegion::generateMinimalPath(geometry_msgs::Pose curr_pose, int side_id) {

  Eigen::Vector3f curr_position(curr_pose.position.x, curr_pose.position.y, curr_pose.position.z);
  int side = side_order_[side_id];
  std::vector<Eigen::Vector3f> best_vp_subset;
  std::vector<int> best_vp_indices;

  if (frontier_poses_.size() != 0) {
    voteSamplePoints(side);
    int sample_times = 0;
    float min_dist = 1e9;
    while (sample_times++ < RESAMPLE_TIMES) {
      std::vector<Eigen::Vector3f> viewpoint_subset;
      sampleMinimalViewpointSubset(side, viewpoint_subset);
      if (viewpoint_subset.size() < 1)  // 2, 1, 0
        continue;
      Eigen::MatrixXf cost_mat;
      std::vector<int> waypoints_indices;
      if (viewpoint_subset.size() != 1)  // work
      {
        cost_mat = constructCostMatrix(curr_position, viewpoint_subset, side);
        waypoints_indices = solveATSPLKH(cost_mat);
        float dist = waypoints_indices.size();
        if (dist < min_dist) {
          dist = min_dist;
          best_vp_subset = viewpoint_subset;
          best_vp_indices = waypoints_indices;
        }
      } else {
        best_vp_subset = viewpoint_subset;
        waypoints_indices.push_back(0);
        best_vp_indices = waypoints_indices;
        sample_times = RESAMPLE_TIMES + 10;  // break while
      }
    }
  }

  nav_msgs::Path path;
  path.header.frame_id = world_frame_;
  path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped waypoint;
  waypoint.header.frame_id = world_frame_;
  waypoint.pose = curr_pose;
  path.poses.push_back(waypoint);
  
  int inteval = best_vp_indices.size() > 8 ? 2 : 1;
  inteval = best_vp_indices.size() > 14 ? 4 : inteval;
  inteval = 1;
  int cnt = 1;
  for (int i = 0; i < best_vp_indices.size(); i += inteval)
  {
    waypoint.pose = viewpointIndexToPose(best_vp_subset[best_vp_indices[i]], side);
    path.poses.push_back(waypoint);
    if (cnt++ >= 5)
      break;
  }
  return path;
}

void SamplingRegion::sampleMinimalViewpointSubset(int side, std::vector<Eigen::Vector3f> &viewpoint_subset) {
  std::vector<Eigen::MatrixXi> score_vector = score_vector_[side];
  std::vector<std::vector<Eigen::Vector3f>> frontier_vp_vector = frontier_vp_vector_[side];

  std::vector<bool> frontier_covered_state;
  frontier_covered_state.clear();
  for (int i = 0; i < frontier_vectors_[side].size(); i++) {
    if (frontier_vp_vector_[side][i].size() != 0)
      frontier_covered_state.push_back(false);
    else
      frontier_covered_state.push_back(true);
  }

  bool isFrontierCleared = true;
  viewpoint_subset.clear();
  do {
    std::vector<Eigen::Vector3f> viewpoint_vec;
    if (generateViewpointVoteSequence(score_vector, viewpoint_vec) == -1)
      break;

    int selected_vp_index = generateRandomInteger(0, viewpoint_vec.size());
    Eigen::Vector3f selected_vp = viewpoint_vec[selected_vp_index];

    viewpoint_subset.push_back(selected_vp);

    sampleVoteDecount(frontier_vp_vector, frontier_covered_state, score_vector, selected_vp);

    int cnt = 0;
    for (int i = 0; i < frontier_covered_state.size(); i++) {
      if (frontier_covered_state[i] == false)
        isFrontierCleared = false;
      else
        cnt++;
    }
  } while (isFrontierCleared != true && ros::ok());
}

#pragma endregion

nav_msgs::Path SamplingRegion::sampleViewpointsFunc(nav_msgs::Path &frontiers_pose, nav_msgs::Path &frontiers_color,
                                                    geometry_msgs::Pose &robot_pose, geometry_msgs::Pose &bbox_pose,
                                                    int *side_id) {
  static int planning_times = 0;

  recvNewInfo(frontiers_pose, frontiers_color, robot_pose, bbox_pose);
  generateSamplingRegion();
  nav_msgs::Path path = generateMinimalPath(robot_pose, *side_id);

  bool shift_side = false;
  if (path.poses.size() == 1) {
    shift_side = true;
  } else if (planning_times++ > PLANNING_TIMES) {
    shift_side = true;
  }

  if (shift_side) {
    planning_times = 0;
    path.poses.clear();
    geometry_msgs::PoseStamped ps;
    ps.pose = robot_pose;
    path.poses.push_back(ps);
    ps.pose = quadrantViewpointToPose(side_order_[*side_id]);
    path.poses.push_back(ps);
    *side_id = *side_id + 1;

    if (*side_id >= 4) {
      path.poses.clear();  // exit exploring
      ROS_WARN_STREAM("<sampleViewpointsFunc>: exploring completed");
    }
  }

  {
    drawColorBoundingbox(bbox_, 0, 0, 1, "building_bbox");  // bbox_pub
    drawColorBoundingbox(sampling_bbox_, 0, 1, 0, "sampling_bbox");
    drawSamplingRegion();  // sample_points_pub
    drawFrontierNormal();  // frontier_normal_pub
    drawViewpointMarkers(path);
  }

  return path;
}
