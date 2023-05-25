/*
 * Created on Thu May 13 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#include "active_mapping/utils/tictoc.hpp"

std::thread TicToc::pub_thread_ = std::thread(&TicToc::timePubThread);
std::unordered_map<std::string, double> TicToc::toc_dict_
  = std::unordered_map<std::string, double>();
std::unordered_map<std::string, double> TicToc::tic_dict_
  = std::unordered_map<std::string, double>();
std::unordered_map<std::string, ros::Publisher> TicToc::time_pubs_
  = std::unordered_map<std::string, ros::Publisher>();
