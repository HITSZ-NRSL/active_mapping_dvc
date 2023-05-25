/*
 * Created on Fri Aug 13 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#pragma once
#include <ros/ros.h>
#include <string>
#include <utility>

static std::string _CutParenthesesNTail(std::string &&pretty_func) {
  size_t pos = pretty_func.find('(');
  if (pos != std::string::npos) {
    pretty_func.erase(pretty_func.begin() + pos, pretty_func.end());
  }

  pos = pretty_func.find(' ');
  if (pos != std::string::npos) {
    pretty_func.erase(pretty_func.begin(), pretty_func.begin() + pos + 1);
  }

  pretty_func = "[" + pretty_func + "] ";
  return pretty_func;
}

#define __STR_FUNCTION__ _CutParenthesesNTail(std::string(__PRETTY_FUNCTION__))
#define ROS_INFO_STREAM_FUNC(args) ROS_INFO_STREAM(__STR_FUNCTION__ << args)
#define ROS_WARN_STREAM_FUNC(args) ROS_WARN_STREAM(__STR_FUNCTION__ << args)
#define ROS_INFO_STREAM_FUNC(args) ROS_INFO_STREAM(__STR_FUNCTION__ << args)
#define ROS_ERROR_STREAM_FUNC(args) ROS_ERROR_STREAM(__STR_FUNCTION__ << args)

template <typename T> T getParam(const ros::NodeHandle &nh, std::string name, T default_val) {
  T res = nh.param(name, default_val);
//   ROS_INFO_STREAM(nh.getNamespace() << "/" << name << ": " << res);
  return res;
}

static std::string replace_all_distinct(std::string str, std::string old_value, std::string new_value) 
{ 
	for(std::string::size_type pos(0); pos!=std::string::npos; pos++) { 
	if( (pos=str.find(old_value,pos))!=std::string::npos ) 
		str.replace(pos,old_value.length(),new_value); 
	else 
		break; 
	} 
	return str; 
} 