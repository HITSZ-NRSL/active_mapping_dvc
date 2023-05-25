/*
 * Created on Fri Apr 23 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#ifndef SRC_ACTIVE_MAPPING_INCLUDE_ACTIVE_MAPPING_UTILS_TICTOC_HPP_
#define SRC_ACTIVE_MAPPING_INCLUDE_ACTIVE_MAPPING_UTILS_TICTOC_HPP_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <string>
#include <unordered_map>
#include <list>
#include <thread>
#include <chrono>

class TicToc {
 public:
  TicToc() {
  }
  void tic(std::string name = "nop") {
    // if (time_pubs_.count(name) == 0) {
    //   time_pubs_[name] =
    //     nh_.advertise<std_msgs::Float32>("tictoc/" + name, 1);
    // }
    tic_dict_[name] = ros::Time::now().toSec();
  }

  double toc(std::string name = "nop") {
    if (tic_dict_.find(name) == tic_dict_.end())
      return 0;
    if (toc_dict_.find(name) == toc_dict_.end()) {
      toc_dict_[name] = 0;
    }
    double toc_time = ros::Time::now().toSec() - tic_dict_[name];
    toc_dict_[name] += toc_time;
    return toc_time;
  }

  double totalToc(std::string name = "nop") {
    return toc_dict_[name];
  }

 private:
  static void timePubThread() {
    // while (!ros::ok()) {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // }
    // while (ros::ok()) {
    //   for (auto & pub_pair : time_pubs_) {
    //     std_msgs::Float32 msg;
    //     msg.data = toc_dict_[pub_pair.first];
    //     pub_pair.second.publish(msg);
    //   }
    //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // }
  }

  static std::unordered_map<std::string, double> toc_dict_;
  static std::unordered_map<std::string, double> tic_dict_;
  ros::NodeHandle nh_;
  static std::unordered_map<std::string, ros::Publisher> time_pubs_;
  static std::thread pub_thread_;
};

#endif  // SRC_ACTIVE_MAPPING_INCLUDE_ACTIVE_MAPPING_UTILS_TICTOC_HPP_
