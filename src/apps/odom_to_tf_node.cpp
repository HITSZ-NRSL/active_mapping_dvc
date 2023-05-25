/*
 * Created on Fri Oct 08 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <tf/transform_broadcaster.h>

#include "active_mapping/utils/math_utils.h"

ros::Publisher odom_lio_pub;
bool publish_tf;
std::string odom_topic;
std::string odom_frame;
void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  static tf::TransformBroadcaster br;
  tf::Transform trans;
  tf::poseMsgToTF(msg->pose.pose, trans);

  static bool first_exec = true;
  static tf::Transform init_trans;
  if (first_exec) {
    init_trans = trans;
    first_exec = false;
  }
  tf::Transform odom_trans = init_trans.inverse() * trans;
  tf::Transform odom_trans_inv = odom_trans.inverse();
  if (publish_tf) {
    br.sendTransform(tf::StampedTransform(odom_trans_inv, msg->header.stamp, "livox", "odom_lio"));
  }

  nav_msgs::Odometry odom_msg;
  static nav_msgs::Odometry last_odom_msg = odom_msg;
  odom_msg.header.stamp = msg->header.stamp;
  odom_msg.header.frame_id = odom_frame;
  tf::poseTFToMsg(odom_trans, odom_msg.pose.pose);
  double dt = (odom_msg.header.stamp - last_odom_msg.header.stamp).toSec();
  if (dt >= 0.1) {
    double dx = odom_msg.pose.pose.position.x - last_odom_msg.pose.pose.position.x;
    double dy = odom_msg.pose.pose.position.y - last_odom_msg.pose.pose.position.y;
    double dz = odom_msg.pose.pose.position.z - last_odom_msg.pose.pose.position.z;

    double curr_yaw = tf::getYaw(odom_msg.pose.pose.orientation);
    double last_yaw = tf::getYaw(last_odom_msg.pose.pose.orientation);
    double dyaw = math_utils::angleWarp(curr_yaw - last_yaw);

    odom_msg.twist.twist.linear.x = sqrt(dx * dx + dy * dy + dz * dz) / dt;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.linear.z = 0;

    odom_msg.twist.twist.angular.x = 0;
    odom_msg.twist.twist.angular.y = 0;
    odom_msg.twist.twist.angular.z = dyaw / dt;

    odom_lio_pub.publish(odom_msg);

    last_odom_msg = odom_msg;
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "odom_to_tf_node");
  ros::NodeHandle nh, nh_local("odom_to_tf_node");
  ros::Subscriber odom_sub = nh.subscribe("odometry_gt", 1, odomCallback);
  nh_local.param<std::string>("odom_topic", odom_topic, "odometry_lio");
  nh_local.param<bool>("publish_tf", publish_tf, true);
  nh_local.param<std::string>("odom_frame", odom_frame, "odom_lio");
  odom_lio_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1);
  ros::spin();

  return 0;
}
