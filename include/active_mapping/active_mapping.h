/*
 * Created on Wed Sep 22 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */
#pragma once

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <lkh_tsp_solver/lkh_interface.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <pcl/point_types.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <mutex>
#include <memory>
#include <iostream>
#include <string>

#include "active_mapping/active_mapping.h"
#include "active_mapping/SensorAngleUpdateAction.h"
#include "active_mapping/world_representation/world_representation.h"
#include "active_mapping/view_space/view_space_finder.h"
#include "active_mapping/controller/gimbal_controller.h"
#include "active_mapping/utils/bounding_box.h"

namespace active_mapping {
class ActiveMapping {
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/**
	 * @brief Constructor. Parameters are loaded and some ros publishers are initialized.
	 * 
	 * @param [in] nh input ros node handle.
	 */
	explicit ActiveMapping(ros::NodeHandle *nh);

	/**
	 * @brief Main thread for the active mapping system. It is a finite state machine. For more explanation of the main thread, you may refer to the wiki of this repo.
	 * 
	 */
	void run();

	std::thread run_thread_;

	std::string view_space_finder_type;
	pcl::PointCloud<pcl::PointXYZI> path_points_;

 private:
	/**
	 * @brief Initialization of the whole system. It includes following steps:
	 * 1. Zero the pitch angle of the gimbal.
	 * 2. Turn a round to initialize the 2d grid map.
	 * 3. Initialize the disjoint set (set building root).
	 */
	void initialization();
	/**
	 * @brief Load parameters from ros.
	 * 
	 * @return true success load or set all parameters.
	 * @return false In no case will it return false.
	 */
	bool loadParameters();
	/**
	 * @anchor getSensorPoseTf
	 * @brief Get the Sensor Pose from tf tree.
	 * 
	 * @param [out] tf result tf transform
	 * @return true if succeed to get transformation
	 * @return false if failed to get transformation
	 */
	bool getSensorPoseTF(tf::StampedTransform *tf);

	/**
	 * @brief This function is an encapsulation of @ref getSensorPoseTf "getSensorPoseTf". It return Eigen-style transformation.
	 * 
	 * @param [out] sensor_pose result of transformation in Eigen-style
	 * @return true if succeed to get transformation
	 * @return false if failed to get transformation
	 */
	bool getSensorPoseEigen(Eigen::Isometry3d *sensor_pose);

	/**
	 * @brief This function is an encapsulation of @ref getSensorPoseTf "getSensorPoseTf". It return Octomap-style sensor_origin.
	 *
	 * @param [out] sensor_origin result of sensor origin in world frame
	 * @return true if succeed to get origin 
	 * @return false if succeed to get origin
	 */
	bool getSensorOrigin(octomap::point3d *robot_pose);

	/**
	 * @brief This function is an encapsulation of @ref getSensorPoseTf "getSensorPoseTf". It return Eigen-style sensor_origin.
	 * 
	 * @param [out] sensor_origin result of sensor origin in world frame
	 * @return true if succeed to get origin 
	 * @return false if succeed to get origin 
	 */
	bool getRobotPoseEigen(Eigen::Isometry3d *sensor_pose);

	/**
	 * @brief Get Transformation from sensor to base link.
	 * 
	 * @param [out] transform_sensor_to_base_link result of the transformation.
	 * @return true 
	 * @return false 
	 */
	bool getSensorToBaseLinkTransform(Eigen::Isometry3d *transform_sensor_to_base_link);

	std::unique_ptr<ViewSpaceFinder> view_space_finder_; 
	std::shared_ptr<WorldRepresentation> world_representation_;
	std::unique_ptr<GimbalController> gimbal_controller_;
	ViewSpace view_space_;

	/* When octomap has not been updated finishly, this flag is false*/
	bool octomap_update_flag_ = false;
	bool octomap_update_finish_flag_ = false;
	bool new_point_cloud_flag_ = false;
	/* Indicate that whether to show next best view(red arrow). */
	bool move_base_ready_flag_ = false; /**< Indicate that whether it is ready for move_base to move robot. */
	bool use_disjoint_set_ = false;
	bool rotate_for_initialization_ = false;
	float lidar_tf_dt_ = 0;
	float xy_goal_tolerance_;
	float yaw_goal_tolerance_;
	
	std::mutex view_space_mutex_, new_point_cloud_mutex_, drivable_map_update_mutex_;

	Eigen::Isometry3d transform_sensor_to_base_link_;
	
	/**
	 * @brief Enumerate type for representation of the system state.
	 * 
	 */
	enum ActiveMappingState {
		Initialization, /**< The system has not been initialized. */
		ViewPointPlanning, /**< Try to extract frontiers and compute next best view. */
		Executing, /**< Make the robot drive to the next best view. */
		Exit, /**< No viewpoint is valid. System exit. */
	};

	ActiveMappingState am_state_ = ActiveMappingState::Initialization;

	/**
	 * @brief Enumerate type for representation of the move base state.
	 * 
	 */
	enum ExecuteState {
		Initialize, /**< The system has not been initialized. */
		AboutToReach, /**< The robot is near the target viewpoint. */
		FarToReach, /**< The robot is far from the target view point. */
		Reached, /**< The robot has reached the viewpoint. */
		Replan, /**< In the way to the next best viewpoint, the robot find that the viewpoint become invalid. */
	};

	ExecuteState exe_state_ = ExecuteState::Initialize;

 private:
	/**
	 * @brief ros callback of lidar
	 *
	 * @param [in] lidar_msg
	 */
	void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &lidar_msg);

	/**
	 * @brief ActiveMappings' odom callback
	 *
	 * @param [in] odom_msg
	 */
	void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);

	/**
	 * @brief Thread function to publish view space in a specify frequence.
	 *
	 */
	void publishViewSpaceMsg();

	/**
	 * @brief Publish this view's postion to movebase.
	 *
	 * @param [in] candidate_view View to publish.
	 * @param [in] wait_for_result if set true, current thread will be hang on until robot reached the viewpoint. 
	 * @return true if succeed.
	 * @return false if failed.
	 */
	bool publishViewAsGoal(const View &candidate_view, bool wait_for_result = true);

	enum PubAngleType { YAW = 0, PITCH = 1 };
	/**
	 * @brief Publish view's pitch angle to ros control.
	 *
	 * @param [in] sensor_pitch Pitch angle to publish.
	 * @return true if succeed.
	 * @return false if failed.
	 */
	bool publishSensorAngle(float sensor_angle, PubAngleType type, bool wait_for_result = true);

	/**
	 * @brief Callback function of sensorPitchUpdate serveice.
	 *
	 * @param [in] goal The goal pitch angle that need to be reached.
	 */
	void sensorAngleUpdateCallback(const active_mapping::SensorAngleUpdateGoalConstPtr &goal);



	ros::NodeHandle nh_, nh_local_;
	ros::Subscriber lidar_sub_, scan_sub_, odom_sub_;
	ros::Publisher nbv_pose_pub_; /**< next-best-view's postion publisher */
	ros::ServiceClient sensor_pitch_query_client_;
	/* For debug */
	ros::Publisher nbf_views_pub_; /**< next best frontier's views publisher */
	ros::Publisher nbf_views_scores_pub_;
	ros::Publisher nbf_dir_pub_;
	ros::Publisher update_bbox_pub_;
	nav_msgs::Odometry odom_;
	/*************/
	std::string lidar_frame_, world_frame_, base_frame_, odom_frame_, lidar_crank_joint_, odom_topic_;
	double max_vel_x_, max_vel_theta_;
	View nbv_;

	/* ROS actions */
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
	actionlib::SimpleActionServer<active_mapping::SensorAngleUpdateAction> sensor_angle_update_action_server_;
	actionlib::SimpleActionClient<active_mapping::SensorAngleUpdateAction> sensor_angle_update_action_client_;

	
	ros::Publisher odom_path_pub_, agent_pose_pub, odom_pcd_pub_;
	nav_msgs::Path odom_path_;	
	std::mutex nbv_mutex_, exe_mutex_;

	float odomPath_length_ = 0;
	nav_msgs::Path current_path_;


};
};  // namespace active_mapping
