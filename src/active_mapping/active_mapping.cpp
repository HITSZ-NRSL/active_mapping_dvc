#include "active_mapping/active_mapping.h"
#include <ros/package.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <queue>
#include <chrono>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "active_mapping/controller/gazebo_gimbal_controller.h"
#include "active_mapping/view_space/frontier_view_space_finder.h"
#include "active_mapping/utils/octomap_utils.h"
#include "active_mapping/utils/pointxyzirt.h"
#include "active_mapping/utils/tictoc.hpp"
#include "active_mapping/utils/rviz_utils.h"
#include "active_mapping/utils/math_utils.h"
#include "active_mapping/utils/common_utils.h"

namespace active_mapping {
using std::literals::chrono_literals::operator""s;

ActiveMapping::ActiveMapping(ros::NodeHandle *nh)
    : nh_(*nh), move_base_client_("move_base", false),
      sensor_angle_update_action_server_(*nh, "SensorPitchUpdate",
                                         boost::bind(&ActiveMapping::sensorAngleUpdateCallback, this, _1), false),
      sensor_angle_update_action_client_(*nh, "SensorPitchUpdate", false) {

  loadParameters();

  while (!getSensorToBaseLinkTransform(&transform_sensor_to_base_link_) && ros::ok()) {
    ROS_ERROR("Failed to get transform from sensor to base link");
  }

  run_thread_ = std::thread(std::bind(&ActiveMapping::run, this));
  nbv_.setViewScore(-1);
  /* for Debug */
  nbf_views_pub_ = nh->advertise<geometry_msgs::PoseArray>("nbf_views", 1);
  nbf_views_scores_pub_ = nh->advertise<visualization_msgs::MarkerArray>("nbf_view_scores", 1);
  nbf_dir_pub_ = nh->advertise<geometry_msgs::PoseStamped>("nbf_dir", 1);
  update_bbox_pub_ = nh->advertise<visualization_msgs::Marker>("update_bbox", 1);

  odom_path_pub_ = nh->advertise<nav_msgs::Path>("odom_path", 1);
  odom_pcd_pub_ = nh->advertise<sensor_msgs::PointCloud2>("odom_path_pcd", 1);
  agent_pose_pub = nh->advertise<geometry_msgs::PoseStamped>("aeplanner/agent_pose", 1);
}

bool ActiveMapping::loadParameters() {
  world_representation_.reset(new WorldRepresentation());

  /* ROS paramters */
  ros::NodeHandle nh_local("/active_mapping"), nh_teb("/move_base/TebLocalPlannerROS");
  am_state_ = ActiveMappingState::Initialization;
  /* teb parameters */
  nh_teb.getParam("max_vel_x", max_vel_x_);
  nh_teb.getParam("max_vel_theta", max_vel_theta_);
  nh_teb.getParam("xy_goal_tolerance", xy_goal_tolerance_);
  xy_goal_tolerance_ += 0.3;  // 0.2
  nh_teb.getParam("yaw_goal_tolerance", yaw_goal_tolerance_);
  yaw_goal_tolerance_ += 0.15;
  /* local parameters */

  nh_local.getParam("view_space_finder", view_space_finder_type);
  view_space_finder_.reset(new FrontierViewSpaceFinder);
  view_space_finder_->setWorldRepresentation(world_representation_);
  world_representation_->setTrunctedLength(1);

  std::string lidar_topic;
  nh_local.getParam("lidar_topic", lidar_topic);
  lidar_sub_ = nh_.subscribe(lidar_topic, 1, &ActiveMapping::lidarCallback, this, ros::TransportHints().tcpNoDelay());
  nh_local.getParam("lidar_frame", lidar_frame_);
  nh_local.getParam("world_frame", world_frame_);
  nh_local.getParam("base_frame", base_frame_);
  nh_local.getParam("odom_frame", odom_frame_);
  nh_local.getParam("use_disjoint_set", use_disjoint_set_);

  gimbal_controller_.reset(new GazeboGimbalController);

  sensor_pitch_query_client_ = nh_.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
  nh_local.getParam("lidar_crank_joint", lidar_crank_joint_);
  sensor_angle_update_action_server_.start();

  nh_local.param<float>("lidar_tf_dt", lidar_tf_dt_, 0);
  nh_local.param<std::string>("odom_topic", odom_topic_, "active_mapping/odom");
  odom_sub_ = nh_.subscribe(odom_topic_, 1, &ActiveMapping::odomCallback, this, ros::TransportHints().tcpNoDelay());
  nh_local.param<bool>("rotate_for_initialization", rotate_for_initialization_, true);

  return true;
}

void ActiveMapping::run() {
  Eigen::Isometry3d sensor_pose_eigen;
  octomap::point3d sensor_origin;
  while (ros::ok()) {
    switch (am_state_) {
    case ActiveMappingState::Initialization: {
      Eigen::Isometry3d curr_pose;
      getSensorPoseEigen(&curr_pose);
      initialization();
      am_state_ = ActiveMappingState::ViewPointPlanning;
    } break;
    case ActiveMappingState::ViewPointPlanning: {
      if (!getSensorPoseEigen(&sensor_pose_eigen)) {
        ROS_WARN("Cannot get sensor origin now.");
        continue;
      }

      sensor_origin = octomap::toOctomap(sensor_pose_eigen.translation());
      std::lock_guard<std::mutex> lock(view_space_mutex_);
      view_space_.clear();
      octomap::Boundingbox update_bbox = world_representation_->getInputPointcloudBbox();
      update_bbox.insertPoint(sensor_origin);
      static bool first_exec = true;
      visualization_msgs::Marker update_bbox_msg = rviz_utils::drawBoundingbox(update_bbox);
      update_bbox_msg.header.frame_id = world_frame_;
      update_bbox_msg.header.stamp = ros::Time::now();
      update_bbox_pub_.publish(update_bbox_msg);

      bool success = true;
      {
        if (first_exec) {

          first_exec = false;
          world_representation_->setBuildingBoundingbox(world_representation_->getInputPointcloudBbox());
        }

        current_path_ = view_space_finder_->getFrontiersAndViewpoints(sensor_pose_eigen,
                                                                      world_representation_->getBuildingBoundingbox());
        if (current_path_.poses.size() == 0)
          success = false;

        world_representation_->resetInputPointcloudBbox();
      }

      if (!success) {
        am_state_ = ActiveMappingState::Exit;
      } else {
        std::lock_guard<std::mutex> lock(exe_mutex_);
        exe_state_ = FarToReach;
        am_state_ = ActiveMappingState::Executing;
      }

    } break;
    case ActiveMappingState::Executing: {
      for (int i = 1; i < current_path_.poses.size(); i++) {
        {
          Eigen::Quaternionf q(current_path_.poses[i].pose.orientation.w, current_path_.poses[i].pose.orientation.x,
                               current_path_.poses[i].pose.orientation.y, current_path_.poses[i].pose.orientation.z);
          Eigen::Vector3f rpy = toEulerAngle(q);

          View next_vp(current_path_.poses[i].pose.position.x, current_path_.poses[i].pose.position.y,
                       current_path_.poses[i].pose.position.z, rpy(2), rpy(1));
          nbv_ = next_vp;
          nbv_.setViewScore(1);
        }

        static bool submap_flag = false;
        publishSensorAngle(0, PubAngleType::PITCH, true);
        std::this_thread::sleep_for(1s);
        publishViewAsGoal(nbv_, false);
        {
          std::lock_guard<std::mutex> lock(exe_mutex_);
          exe_state_ = ExecuteState::FarToReach;
        }

        ExecuteState exe_state;
        geometry_msgs::Twist rot_vel;
        ros::Publisher cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        ros::Publisher move_base_cancel_pub = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
        actionlib_msgs::GoalID goal;
        do {
          static unsigned char count = 0;
          static unsigned char max_count = 5;
          std::this_thread::sleep_for(0.1s);
          {
            std::lock_guard<std::mutex> lock(exe_mutex_);
            exe_state = exe_state_;
          }
          if (exe_state == ExecuteState::AboutToReach) {
            move_base_cancel_pub.publish(goal);

            tf::StampedTransform sensor_pose_tf;
            getSensorPoseTF(&sensor_pose_tf);
            double curr_yaw = tf::getYaw(sensor_pose_tf.getRotation());
            double diff_yaw = math_utils::angleWarp(nbv_.yaw() - curr_yaw);

            rot_vel.linear.x = 0;
            rot_vel.angular.z = (diff_yaw > 0) ? 1.0 : -1.0;
            cmd_vel_pub.publish(rot_vel);

          } else if (exe_state == ExecuteState::Reached) {
            rot_vel.linear.x = 0;
            rot_vel.angular.z = 0;
            cmd_vel_pub.publish(rot_vel);
            std::this_thread::sleep_for(0.3s);
            world_representation_->startUpdating();  // work
            publishSensorAngle(fabs(nbv_.sensorPitch()), PubAngleType::PITCH, true);
            std::this_thread::sleep_for(1.5s);
            world_representation_->stopUpdating();
            if (i == current_path_.poses.size() - 1)
              am_state_ = ActiveMappingState::ViewPointPlanning;
            break;
          }

        } while (am_state_ == ActiveMappingState::Executing && ros::ok());
      }

    } break;
    case ActiveMappingState::Exit: {
      ROS_WARN_STREAM_FUNC("\nFSM State"
                           << "State: Exit");
      return;
    } break;
    }
  }
}

void ActiveMapping::initialization() {
  gimbal_controller_->sendPitch(0);
  std::this_thread::sleep_for(2s);
  /* Wait for lidar topic to be published. */
  while (!new_point_cloud_flag_) {
    if (!ros::ok())
      return;
    std::this_thread::yield();
  }

  std::this_thread::sleep_for(2s);
  if (use_disjoint_set_) {
    auto disjoint_roots = world_representation_->getOctreePtr()->disjointSet().roots();
    std::unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash> root_count;
    for (auto pair : disjoint_roots) {
      ++root_count[pair.second];
    }
    octomap::OcTreeKey max_root;
    int max_count = 0;
    for (auto root : root_count) {
      if (max_count < root.second) {
        max_count = root.second;
        max_root = root.first;
      }
    }
    world_representation_->setBuildingRoot(max_root);
  }

  if (rotate_for_initialization_) {
    ros::Publisher cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    geometry_msgs::Twist rot_vel;
    rot_vel.angular.z = -1;
    cmd_vel_pub.publish(rot_vel);
    //
    tf::StampedTransform sensor_pose_tf;
    getSensorPoseTF(&sensor_pose_tf);
    double last_yaw = tf::getYaw(sensor_pose_tf.getRotation());
    double diff_yaw = 0;
    for (int i = 0; i < 20; i++) {
      cmd_vel_pub.publish(rot_vel);
      std::this_thread::sleep_for(0.1s);
    }
    do {
      getSensorPoseTF(&sensor_pose_tf);
      double curr_yaw = tf::getYaw(sensor_pose_tf.getRotation());
      diff_yaw += math_utils::angleWarp(last_yaw - curr_yaw);
      last_yaw = curr_yaw;
      cmd_vel_pub.publish(rot_vel);
      std::this_thread::sleep_for(0.1s);
    } while (fabs(diff_yaw) < 2 * M_PI && ros::ok());
    rot_vel.angular.z = 0;
    cmd_vel_pub.publish(rot_vel);
    std::this_thread::sleep_for(0.05s);
    cmd_vel_pub.publish(rot_vel);
  }
}

bool ActiveMapping::getSensorPoseTF(tf::StampedTransform *sensor_pose) {
  static tf::TransformListener listener;
  try {
    listener.waitForTransform(world_frame_, lidar_frame_, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(world_frame_, lidar_frame_, ros::Time(0), *sensor_pose);
  } catch (...) {
    // ROS_WARN("Sensor data is earlier than tf!1");
    return false;
  }
  return true;
}

bool ActiveMapping::getSensorPoseEigen(Eigen::Isometry3d *sensor_pose) {
  tf::StampedTransform lidar_transform_tf;
  static tf::TransformListener listener;
  try {
    listener.waitForTransform(world_frame_, lidar_frame_, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(world_frame_, lidar_frame_, ros::Time(0), lidar_transform_tf);
  } catch (...) {
    // ROS_WARN("Sensor data is earlier than tf!2");
    return false;
  }
  tf::transformTFToEigen(lidar_transform_tf, *sensor_pose);

  return true;
}

bool ActiveMapping::getRobotPoseEigen(Eigen::Isometry3d *robot_pose) {
  tf::StampedTransform lidar_transform_tf;
  static tf::TransformListener listener;
  try {
    listener.waitForTransform(world_frame_, base_frame_, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(world_frame_, base_frame_, ros::Time(0), lidar_transform_tf);
  } catch (...) {
    // ROS_WARN("Sensor data is earlier than tf!5");
    return false;
  }
  tf::transformTFToEigen(lidar_transform_tf, *robot_pose);

  return true;
}

bool ActiveMapping::getSensorOrigin(octomap::point3d *sensor_origin) {
  Eigen::Isometry3d lidar_transform_eigen;
  getSensorPoseEigen(&lidar_transform_eigen);

  sensor_origin->x() = lidar_transform_eigen.translation().x();
  sensor_origin->y() = lidar_transform_eigen.translation().y();
  sensor_origin->z() = lidar_transform_eigen.translation().z();

  return true;
}

bool ActiveMapping::getSensorToBaseLinkTransform(Eigen::Isometry3d *transform_sensor_to_base_link) {
  tf::StampedTransform transform_sensor_to_base_link_tf;
  static tf::TransformListener listener;
  try {
    listener.waitForTransform(lidar_frame_, base_frame_, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(lidar_frame_, base_frame_, ros::Time(0), transform_sensor_to_base_link_tf);
  } catch (...) {
    // ROS_WARN("Sensor data is earlier than tf!3");
    return false;
  }
  tf::transformTFToEigen(transform_sensor_to_base_link_tf, transform_sensor_to_base_link_);

  return true;
}

bool ActiveMapping::publishViewAsGoal(const View &candidate_view, bool wait_for_result) {
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = world_frame_;
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose = candidate_view.pose2d().transTo(transform_sensor_to_base_link_).toROSPose();

  move_base_client_.sendGoal(goal);
  if (wait_for_result) {
    move_base_client_.waitForResult();
  }

  if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    return true;
  else
    return false;
}

bool ActiveMapping::publishSensorAngle(float sensor_angle, PubAngleType type, bool wait_for_result) {
  gimbal_controller_->sendPitch(sensor_angle);
  std::this_thread::sleep_for(1s);
  return true;
}

void ActiveMapping::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &lidar_msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_pc_pcl(new pcl::PointCloud<pcl::PointXYZ>),
      world_pc_pcl(new pcl::PointCloud<pcl::PointXYZ>), local_pc_pcl_(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*lidar_msg, *local_pc_pcl);

  tf::StampedTransform lidar_transform_tf;
  static tf::TransformListener listener;
  ros::Time lidar_stamp = lidar_msg->header.stamp + ros::Duration(lidar_tf_dt_);
  try {
    listener.waitForTransform(world_frame_, lidar_frame_, lidar_stamp, ros::Duration(3.0));
    listener.lookupTransform(world_frame_, lidar_frame_, lidar_stamp, lidar_transform_tf);
  } catch (...) {
    // ROS_WARN_STREAM_FUNC("Sensor data is earlier than tf! world_frame: " << world_frame_ << ", lidar_frame_: " <<
    // lidar_frame_);
    return;
  }
  Eigen::Isometry3d lidar_transform_eigen;
  tf::transformTFToEigen(lidar_transform_tf, lidar_transform_eigen);
  for (int i = 0; i < local_pc_pcl->points.size(); i++) {
    if (fabs(local_pc_pcl->points[i].x) < 0.1 && fabs(local_pc_pcl->points[i].y) < 0.1 &&
        fabs(local_pc_pcl->points[i].z) < 0.1)
      continue;
    local_pc_pcl_->points.push_back(local_pc_pcl->points[i]);
  }
  pcl::transformPointCloud(*local_pc_pcl_, *world_pc_pcl, lidar_transform_eigen.matrix());

  Eigen::Vector3d pose_origin = lidar_transform_eigen.translation();
  world_representation_->insertPointCloud(pose_origin, *world_pc_pcl, true, false);  // work

  new_point_cloud_flag_ = true;
}

void ActiveMapping::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
  {
    static int count = 0;
    if (odom_path_.poses.size() > 1) {
      Eigen::Vector3d curr_pos(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                               odom_msg->pose.pose.position.z);
      Eigen::Vector3d last_pos(odom_path_.poses.back().pose.position.x, odom_path_.poses.back().pose.position.y,
                               odom_path_.poses.back().pose.position.z);
      odomPath_length_ += (last_pos - curr_pos).norm();
    }
    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header.frame_id = world_frame_;
    odom_pose.header.stamp = ros::Time::now();
    odom_pose.pose = odom_msg->pose.pose;
    odom_path_.poses.push_back(odom_pose);
    odom_path_.header.stamp = ros::Time::now();
    odom_path_.header.frame_id = world_frame_;
    odom_path_pub_.publish(odom_path_);
    agent_pose_pub.publish(odom_pose);

    pcl::PointXYZI p;
    p.x = odom_msg->pose.pose.position.x;
    p.y = odom_msg->pose.pose.position.y;
    p.z = odom_msg->pose.pose.position.z;
    p.intensity = (odom_path_.poses.size() / 10) % 255;
    path_points_.points.push_back(p);
    path_points_.height = 1;
    path_points_.width = path_points_.points.size();
    sensor_msgs::PointCloud2 path_pcd_msg;
    pcl::toROSMsg(path_points_, path_pcd_msg);
    path_pcd_msg.header.frame_id = world_frame_;
    odom_pcd_pub_.publish(path_pcd_msg);
  }

  /* no next best view selected */
  if (nbv_.getViewScore() < 0) {
    return;
  }

  Eigen::Isometry3d robot_pose;
  getRobotPoseEigen(&robot_pose);
  Eigen::Matrix3d robot_rot = robot_pose.rotation().matrix();

  Eigen::Vector3d trans_diff = robot_pose.translation() - nbv_.origin();
  trans_diff.z() = 0;

  double odom_yaw = math_utils::getYaw(robot_rot);
  double yaw_diff = math_utils::angleWarp(nbv_.yaw() - odom_yaw);

  {
    static unsigned char cout_count = 0;
    unsigned char max_count = 3;
    std::lock_guard<std::mutex> lock(exe_mutex_);
    if (exe_state_ == ExecuteState::Reached) {
    } else if (exe_state_ == ExecuteState::AboutToReach) {
      if (fabs(yaw_diff) < 0.1) {
        exe_state_ = ExecuteState::Reached;
      }
    } else if (exe_state_ == ExecuteState::FarToReach) {
      if (odom_msg->twist.twist.linear.x < 0.1 && odom_msg->twist.twist.angular.z < 0.05) {
        if (trans_diff.norm() < xy_goal_tolerance_ && fabs(yaw_diff) < yaw_goal_tolerance_) {
          exe_state_ = ExecuteState::AboutToReach;
        } else {
          static unsigned char cnt = 0;
          if (cnt++ > 7) {
            cnt = 0;
            publishViewAsGoal(nbv_, false);
          }
        }
      }
    }

    if (cout_count++ > max_count)
      cout_count = 0;
  }
}

void ActiveMapping::sensorAngleUpdateCallback(const SensorAngleUpdateGoalConstPtr &goal) {
  std::lock_guard<std::mutex> lock(drivable_map_update_mutex_);
  SensorAngleUpdateFeedback feed_back;
  SensorAngleUpdateResult result;
  if (goal->type == PubAngleType::PITCH) {
    double pub_pitch_angle = goal->angle;
    gimbal_controller_->sendPitch(pub_pitch_angle);
    while (fabs(gimbal_controller_->getPitch() - pub_pitch_angle) > 0.05 && ros::ok()) {
      feed_back.angle_sequnce.push_back(gimbal_controller_->getPitch());
      std::this_thread::sleep_for(0.1s);
    }
    result.residual_angle = gimbal_controller_->getPitch() - pub_pitch_angle;
  } else if (goal->type == PubAngleType::YAW) {
    double pub_yaw_angle = goal->angle;
    gimbal_controller_->sendYaw(pub_yaw_angle);
    while (fabs(gimbal_controller_->getYaw() - pub_yaw_angle) > 0.01 && ros::ok()) {
      feed_back.angle_sequnce.push_back(gimbal_controller_->getYaw());
      std::this_thread::sleep_for(0.1s);
    }
    result.residual_angle = gimbal_controller_->getYaw() - pub_yaw_angle;
  }

  sensor_angle_update_action_server_.publishFeedback(feed_back);
  sensor_angle_update_action_server_.setSucceeded(result);
}

}  // namespace active_mapping
