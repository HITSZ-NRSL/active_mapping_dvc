#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <time.h>
#include <omp.h>
#include <lkh_tsp_solver/lkh_interface.h>

#include "active_mapping/world_representation/ig_tree.h"
#include "active_mapping/world_representation/world_representation.h"
#include "active_mapping/utils/common_utils.h"
#include <octomap_msgs/conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>

typedef struct {
	float maxX;
	float maxY;
	float maxZ;
	float minX;
	float minY;
	float minZ;
} bbox_t;
bbox_t bbbox = {0.1, 0.1, 3, 
				-0.1, -0.1, 0};

bool SAMPLE_SAME_POINT = false;
int SAMPLE_WEIGHT = 5;
int RESAMPLE_TIMES = 40;
int PLANNING_TIMES = 5;
int PITCH_UPPER_BOUND = 85; 
int PITCH_INTERVAL = 20;
int YAW_UPPER_BOUND = 80; 
int YAW_INTERVAL = 20;
bool FRONTIER_CHK_ENB = false;
bool ENABLE_RAYCAST_CHK = false;

int MIN_SEQ_LENGTH = 5;
float GAP_SIZE = 2;
float SAMP_ROI_RES = 2;
float SAMP_ROI_SIZE = 3+GAP_SIZE*2;

float VOTE_ANGLE_TH = 20;
float VOTE_DIS_TH = 20;
int SHIFT_NBV_DIS = 2;

std::string world_frame_ = "map";

class SamplingRegion{
public:
	SamplingRegion(bbox_t bbox_new=bbbox, float resolution=SAMP_ROI_RES, float sample_num=SAMP_ROI_SIZE);

	void updateBuildingBoundingBox(bbox_t bbox_new);
	void updateBuildingBoundingBox(octomap::Boundingbox bbox);
	float positionToAzimuth(Eigen::Vector3f curr_position);
	int queryCurrentQuadrant(Eigen::Vector3f curr_position);
	void generateSamplingRegion();

	float calcYawBetweenVectors(Eigen::Vector3f vec1, Eigen::Vector3f vec2);
	float calc3DAngleBetweenVectors(Eigen::Vector3f vec1, Eigen::Vector3f vec2);
	std::vector<visualization_msgs::Marker> drawSamplingRegion();
	std::vector<visualization_msgs::Marker> drawFrontierNormal(int clear=0);
	visualization_msgs::Marker drawColorBoundingbox(bbox_t bbox, float r, float g, float b, std::string ns);
	
	int calcFrontierSide(Eigen::Vector3f curr_position);
	void voteSamplePoints(int side);
	void sampleVoteCount(int side, int index);
	void sampleVoteDecount(std::vector<std::vector<Eigen::Vector3f>> &frontier_vp_vector,
							std::vector<bool> &frontier_covered_state,
							std::vector<Eigen::MatrixXi> &score_vector,
							Eigen::Vector3f selected_vp);

	void sampleMinimalViewpointSubset(int side, std::vector<Eigen::Vector3f> &viewpoint_subset);
	int generateViewpointVoteSequence(std::vector<Eigen::MatrixXi> &score_vector, 
										std::vector<Eigen::Vector3f> &viewpoint_vec);
	std::vector<visualization_msgs::Marker> drawViewpointMarkers(nav_msgs::Path path);

	void generateTestViewpoints(int side, Eigen::Vector3f curr_position, 
								std::vector<Eigen::Vector3f> &viewpoint_subset);
	int  calcViewAngle(Eigen::Vector3f curr_viewpoint, Eigen::Vector3f frontier_point, 
						Eigen::Vector3f side_normal, int *pitch_index, int *yaw_index);
	int  checkFrontierAngle(Eigen::Vector3f curr_viewpoint, Eigen::Vector3f frontier_point, 
						Eigen::Vector3f frontier_normal, Eigen::Vector3f side_normal);
	bool checkVisible(Eigen::Vector3f curr_viewpoint, Eigen::Vector3f frontier_point);

	Eigen::MatrixXf constructCostMatrix(Eigen::Vector3f curr_position,
										std::vector<Eigen::Vector3f> &viewpoint_subset, int side);
	std::vector<int> solveATSPLKH(Eigen::MatrixXf &cost_matrix);
	float calcPathLength(Eigen::Vector3f curr_position, 
							std::vector<Eigen::Vector3f> &viewpoint_subset,
							std::vector<int> &indices, int side);
	geometry_msgs::Pose viewpointIndexToPose(Eigen::Vector3f vp_index, int side);
	nav_msgs::Path generateMinimalPath(geometry_msgs::Pose curr_pose, int side_id);	
	
	void recvNewInfo(nav_msgs::Path &frontiers_pose, nav_msgs::Path &frontiers_color, 
									geometry_msgs::Pose &robot_pose,
									geometry_msgs::Pose &bbox );
	nav_msgs::Path sampleViewpointsFunc(nav_msgs::Path &frontiers_pose, nav_msgs::Path &frontiers_color, 
										geometry_msgs::Pose &robot_pose,
										geometry_msgs::Pose &bbox_pose, int *side_id);
	geometry_msgs::Pose quadrantViewpointToPose(int side);

	bbox_t bbox_;
	bbox_t sampling_bbox_;
	std::vector<std::vector<geometry_msgs::Point>> quadrant_vector_;
	std::vector<std::vector<Eigen::MatrixXi>> score_vector_;

	// [0-5]: maxX, maxY, maxZ, minX, minY, minZ, [6-15]: x, y, z, qw, qx, qy, qz, r, g, b
	std::vector<std::vector<float>> frontier_poses_; 
	std::vector<std::vector<std::vector<float>>> frontier_vectors_;
	std::vector<std::vector<std::vector<Eigen::Vector3f>>> frontier_vp_vector_;

	
	float bbox_width_;
	float bbox_length_;	// width
	float bbox_height_;
	float diagonal_length_;
	Eigen::Vector2f bbox_center_;

	float sampling_radius;
	float sampling_resolution_;
	float sampling_number_;
	float angle_range[4][2]; 
	std::vector<Eigen::Vector3f> side_normals_;	
	std::vector<Eigen::Vector3f> quadrant_nbv_;
	int side_order_[4] = {2, 1, 0, 3};

	// 0:2, 1:1, 2:0, 3:3    2:0, 1:1, 0:2, 3:3
	int side_order_inv[4] = {2, 1, 0, 3};
	

	ros::Publisher bbox_pub, sample_points_pub, frontier_normal_pub;
	ros::Publisher octomap_pub, path_pub, view_points_pub, ray_casting_pub;
	ros::Publisher text_pub;
	ros::ServiceClient astar_client;

	// visualization_msgs::Marker bbox_marker, bbox_marker2;
	ros::NodeHandle nh;
	ros::Subscriber pose_sub;
	octomap::IgTree *octree_ptr_;
	active_mapping::WorldRepresentation::Ptr wr_;
	nav_msgs::Path history_path_;

	// ros::ServiceServer service;
};


int generateRandomInteger(int min, int max)
{
	int d = max - min;
	return (rand() % d + min);
}

Eigen::Vector3f transformPoint(Eigen::Quaternionf q, Eigen::Vector3f p, Eigen::Vector3f vec)
{
	Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
	T.rotate(q.toRotationMatrix());
	T.pretranslate(p);
	Eigen::Vector3f vec_ = T * vec;
	return vec_;
}

Eigen::Vector3f toEulerAngle(const Eigen::Quaternionf& q)
{
	double roll, pitch, yaw;
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
	pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
	pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
	yaw = atan2(siny_cosp, cosy_cosp);
	return Eigen::Vector3f(roll, pitch, yaw);
}

