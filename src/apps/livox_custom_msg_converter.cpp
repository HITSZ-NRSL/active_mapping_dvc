/*
 * Created on Mon Oct 04 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher pc_pub;

void lidarCallback(const livox_ros_driver::CustomMsgConstPtr &msg_ptr) {
  size_t pc_size = msg_ptr->points.size();
  pcl::PointCloud<pcl::PointXYZ> cvt_pc;
  cvt_pc.points.reserve(pc_size);
  const std::vector<livox_ros_driver::CustomPoint>& msg_pts = msg_ptr->points;
  for (int i = 0; i < pc_size; ++i) {
    cvt_pc.points.emplace_back(msg_pts[i].x, msg_pts[i].y, msg_pts[i].z);
  }
  cvt_pc.height = 1;
  cvt_pc.width = cvt_pc.size();
  sensor_msgs::PointCloud2 cvt_msg;
  pcl::toROSMsg(cvt_pc, cvt_msg);
  cvt_msg.header = msg_ptr->header;
  pc_pub.publish(cvt_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_custom_msg_converter");

  ros::NodeHandle nh;
  ros::Subscriber pc_sub = nh.subscribe<livox_ros_driver::CustomMsg>("livox/lidar", 1, lidarCallback);
  pc_pub = nh.advertise<sensor_msgs::PointCloud2>("livox/lidar_pointcloud2", 1);
  ros::spin();
  return 0;
}
