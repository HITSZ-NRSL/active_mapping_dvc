/*
 * Created on Wed Dec 16 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include <octomap/ColorOcTree.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <iostream>
#define BACKWARD_HAS_BFD 1
#include <backward_ros/backward.hpp>

#include "active_mapping/active_mapping.h"
#include "active_mapping/world_representation/ig_tree.h"
#include "active_mapping/world_representation/world_representation.h"

using OcTreeType = octomap::IgTree;

int main(int argc, char **argv) {
  backward::SignalHandling sh;
  ros::init(argc, argv, "turtlebot3_active_mapping_task");
  ros::NodeHandle nh, nh_local("turtlebot3_active_mapping_task");

  active_mapping::ActiveMapping active_mapping(&nh);
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  active_mapping.run_thread_.join();
}
