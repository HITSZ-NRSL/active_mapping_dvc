/*
 * Created on Tue Sep 07 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#ifndef ACTIVE_MAPPING_CONTROLLER_GAZEBO_GIMBAL_CONTROLLER_H_
#define ACTIVE_MAPPING_CONTROLLER_GAZEBO_GIMBAL_CONTROLLER_H_

#include <ros/ros.h>
#include <string>
#include "active_mapping/controller/gimbal_controller.h"

namespace active_mapping {
class GazeboGimbalController : public GimbalController {
 public:
  GazeboGimbalController();
  virtual void sendPitch(double pitch_angle);
  virtual void sendYaw(double yaw_angle);
  virtual double getPitch() const;
  virtual double getYaw() const;
  void gimbalTopicCallback(std_msgs::Float64ConstPtr msg)  override {};
 private:
  std::string pitch_controller_topic_;
  std::string yaw_controller_topic_;
  ros::Publisher pitch_pub_;
  ros::Publisher yaw_pub_;
  mutable ros::ServiceClient sensor_angle_query_client_;
};
}  // namespace active_mapping

#endif  // ACTIVE_MAPPING_CONTROLLER_GAZEBO_GIMBAL_CONTROLLER_H_
