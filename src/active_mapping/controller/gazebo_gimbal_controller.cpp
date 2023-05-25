/*
 * Created on Tue Sep 07 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#include "active_mapping/controller/gazebo_gimbal_controller.h"

#include <std_msgs/Float64.h>
#include <gazebo_msgs/GetJointProperties.h>
#include "active_mapping/utils/common_utils.h"

namespace active_mapping {

GazeboGimbalController::GazeboGimbalController() {
  ros::NodeHandle nh, nh_local("active_mapping/gimbal_controller");
  pitch_controller_topic_ = getParam<std::string>(nh_local, "pitch_controller_topic", "");
  yaw_controller_topic_ = getParam<std::string>(nh_local, "yaw_controller_topic", "");

  pitch_pub_ = nh.advertise<std_msgs::Float64>(pitch_controller_topic_, 1);
  yaw_pub_ = nh.advertise<std_msgs::Float64>(yaw_controller_topic_, 1);
  sensor_angle_query_client_ = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
}

void GazeboGimbalController::sendPitch(double pitch_angle) {
  pitch_angle = std::max(-1.57, pitch_angle);
  pitch_angle = std::min(1.57, pitch_angle);
  std_msgs::Float64 pitch_msg;
  pitch_msg.data = -pitch_angle;
  pitch_pub_.publish(pitch_msg);
}

void GazeboGimbalController::sendYaw(double yaw_angle) {
  yaw_angle = std::max(-1.57, yaw_angle);
  yaw_angle = std::min(1.57, yaw_angle);
  std_msgs::Float64 yaw_msg;
  yaw_msg.data = yaw_angle;
  yaw_pub_.publish(yaw_msg);
}

double GazeboGimbalController::getPitch() const {
  gazebo_msgs::GetJointProperties curr_pitch_query;
  curr_pitch_query.request.joint_name = "livox_pitch_joint";
  sensor_angle_query_client_.call(curr_pitch_query);
  return -curr_pitch_query.response.position[0];
}

double GazeboGimbalController::getYaw() const {
  gazebo_msgs::GetJointProperties curr_pitch_query;
  curr_pitch_query.request.joint_name = "livox_yaw_joint";
  sensor_angle_query_client_.call(curr_pitch_query);
  return curr_pitch_query.response.position[0];
}

}  // namespace active_mapping
