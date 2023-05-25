/*
 * Created on Tue Sep 07 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#ifndef ACTIVE_MAPPING_CONTROLLER_GIMBAL_CONTROLLER_H_
#define ACTIVE_MAPPING_CONTROLLER_GIMBAL_CONTROLLER_H_
#include <std_msgs/Float64.h>
#include <ros/ros.h>
namespace active_mapping {
class GimbalController {
 public:
  virtual void sendPitch(double pitch_angle) = 0;
  virtual void sendYaw(double yaw_angle) = 0;
  virtual double getPitch() const = 0;
  virtual double getYaw() const = 0;
  virtual ~GimbalController() {};
  virtual void gimbalTopicCallback(std_msgs::Float64ConstPtr msg) = 0;
};
}  // namespace active_mapping

#endif  // ACTIVE_MAPPING_CONTROLLER_GIMBAL_CONTROLLER_H_
