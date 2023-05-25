/*
 * Created on Sat Nov 28 2020
 *
 * Copyright (c) 2020 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */
#include "active_mapping/world_representation/world_representation.h"

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>
#include <fstream>
#include <list>
#include <memory>
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "active_mapping/utils/rviz_utils.h"
#include "active_mapping/utils/tictoc.hpp"
#include "active_mapping/utils/common_utils.h"

#include <pcl/filters/statistical_outlier_removal.h>	


namespace active_mapping {
WorldRepresentation::WorldRepresentation() {
  loadParameters();

  ros::NodeHandle nh;
  if (publish_octree_) {
    octomap_publisher_ = nh.advertise<octomap_msgs::Octomap>("world_representation/occ", 1);
    octomap_pub_thread_ = std::thread(&WorldRepresentation::publishOcTreeThread, this);
  }

  if (visualize_roi_) {
    roi_publisher_ = nh.advertise<visualization_msgs::Marker>("world_representation/ROI", 1);
    roi_pub_thread_ = std::thread(&WorldRepresentation::publishROIThread, this);
  }

  if (visualize_update_bbox_) {
    update_bbox_publisher_ = nh.advertise<visualization_msgs::Marker>("world_representation/update_bbox", 1);
  }

  insert_cloud_thread_ = std::thread(&WorldRepresentation::insertCloudThread, this);

  save_octomap_server_ =
      nh.advertiseService("save_octomap", &WorldRepresentation::saveOctomapServerCallback, this);

  cost_map_subscriber_ =
      nh.subscribe("/move_base/global_costmap/costmap", 1, &WorldRepresentation::costmapCallback, this);

  filtered_pcd_publisher_ = nh.advertise<sensor_msgs::PointCloud2>( "/livox/pcd_filtered", 1 );	
}

bool WorldRepresentation::loadParameters() {
  ros::NodeHandle nh_local("/active_mapping/world_representation");
  ros::NodeHandle nh("/active_mapping");
  nh_local.getParam("octree_resolution", octree_resolution_);
  nh_local.getParam("octree_frame", octree_frame_);
  nh_local.getParam("publish_octree", publish_octree_);
  nh_local.getParam("publish_frequency", publish_frequency_);
  nh_local.getParam("min_sensor_dist", min_sensor_dist_);
  nh_local.getParam("max_sensor_dist", max_sensor_dist_);
  nh_local.getParam("roi_x_min", roi_.bottom_right_front_pt.x());
  nh_local.getParam("roi_y_min", roi_.bottom_right_front_pt.y());
  nh_local.getParam("roi_z_min", roi_.bottom_right_front_pt.z());
  nh_local.getParam("roi_x_max", roi_.top_left_rear_pt.x());
  nh_local.getParam("roi_y_max", roi_.top_left_rear_pt.y());
  nh_local.getParam("roi_z_max", roi_.top_left_rear_pt.z());
  nh_local.getParam("visualize_roi", visualize_roi_);
  nh_local.getParam("visualize_update_bbox", visualize_update_bbox_);
  nh_local.getParam("fov_horizon", fov_horizon_);
  nh_local.getParam("fov_vertical", fov_vertical_);
  nh.getParam("use_disjoint_set", use_disjoint_set_);
  roi_.setReset(false);

  octree_ptr_.reset(new octomap::IgTree(octree_resolution_, use_disjoint_set_));
  octree_ptr_->enableChangeDetection(true);

  double octree_hit_probability, octree_miss_probability;
  nh_local.param("octree_hit_probability", octree_hit_probability, 0.7);
  nh_local.param("octree_miss_probability", octree_miss_probability, 0.4);
  octree_ptr_->setProbHit(octree_hit_probability);
  octree_ptr_->setProbMiss(octree_miss_probability);
  
  building_bbox_publisher_ = nh_local.advertise<visualization_msgs::Marker>("building_bbox", 1);		
  return true;
}

void WorldRepresentation::removeGround(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                                             pcl::PointCloud<pcl::PointXYZ> *cloud_out) {
  *cloud_out = cloud_in;	// To do
}

void WorldRepresentation::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                                                 pcl::PointCloud<pcl::PointXYZ> *cloud_out) {
	cloud_out->points.clear();

	if(cloud_in.points.size() == 0)
		return;
		
	cloud_filtered.clear();
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_in.makeShared()); 
	sor.setMeanK (50); 
	sor.setStddevMulThresh (3.0); 
	sor.filter (cloud_filtered); 

	for (int i = 0; i < cloud_filtered.points.size(); ++i) {
		if (roi_.ifContain(cloud_filtered.points[i])) {
			cloud_out->points.push_back(cloud_filtered.points[i]);
		}
	}
	if(cloud_out->points.size() == 0)
		return;
	cloud_out->is_dense = false;
	cloud_out->height = cloud_out->points.size();
	cloud_out->width = 1;	

	sensor_msgs::PointCloud2  temp_out_msg;
	pcl::toROSMsg(*cloud_out, temp_out_msg);
	temp_out_msg.header.frame_id = octree_frame_;
	temp_out_msg.header.stamp = ros::Time::now();
	filtered_pcd_publisher_.publish(temp_out_msg);
}

void WorldRepresentation::startUpdating()
{
  std::lock_guard<std::mutex> lock_octree(octree_mutex_);
  curr_submap_ = true;
}

void WorldRepresentation::stopUpdating()
{
  std::lock_guard<std::mutex> lock_octree(octree_mutex_);
  curr_submap_ = false;
}


void WorldRepresentation::insertPointCloud(const Eigen::Vector3d &origin,
                                                 const pcl::PointCloud<pcl::PointXYZ> &cloud, bool filter_pointcloud,
                                                 bool remove_ground) {
  std::lock_guard<std::mutex> lock_octree(octree_mutex_);
  if (frame_count_++ < 10 || curr_submap_) {    	// work
    if(frame_count_ >= 10)
      frame_count_ = 10;

    pcl::PointCloud<pcl::PointXYZ> cloud_no_ground = cloud;

    if (remove_ground) {
      removeGround(cloud, &cloud_no_ground);
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_roi = cloud_no_ground;
    if (filter_pointcloud) {
      filterPointCloud(cloud_no_ground, &cloud_roi);
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_stacked;

    /*update bouding box, the bounding box is reset in active_mapping thread.*/
    input_pc_bbox_.insertPointCloud(cloud_roi);
    occ_bbox_.insertPointCloud(cloud_roi);
    if (visualize_update_bbox_) {
      visualization_msgs::Marker bbox_marker = rviz_utils::drawBoundingbox(input_pc_bbox_);
      bbox_marker.header.frame_id = octree_frame_;
      bbox_marker.header.stamp = ros::Time::now();
      bbox_marker.color.b = 0;
      bbox_marker.color.r = 1;
      update_bbox_publisher_.publish(bbox_marker);
    }

    octomap::point3d origin_octomap;
    origin_octomap.x() = origin.x();
    origin_octomap.y() = origin.y();
    origin_octomap.z() = origin.z();

    octomap::Pointcloud cloud_roi_octomap;
    octomap::pclToOctomap(cloud_roi, &cloud_roi_octomap);
    std::lock_guard<std::mutex> lock(cloud_list_mutex_);
    cloud_list_.emplace_back(cloud_roi_octomap, origin_octomap);
  }
}

void WorldRepresentation::pullOutPointCloud(const Eigen::Vector3d &origin,
                                                  const pcl::PointCloud<pcl::PointXYZ> &cloud, bool filter_pointcloud) {
  pcl::PointCloud<pcl::PointXYZ> cloud_roi;

  if (filter_pointcloud) {
    filterPointCloud(cloud, &cloud_roi);
  } else {
    cloud_roi = cloud;
  }

  octomap::Pointcloud cloud_octo;
  octomap::pclToOctomap(cloud_roi, &cloud_octo);
  std::lock_guard<std::mutex> lock_octree(octree_mutex_);
  octomap::point3d origin_octomap;
  origin_octomap.x() = origin.x();
  origin_octomap.y() = origin.y();
  origin_octomap.z() = origin.z();
  std::lock_guard<std::mutex> lock(octree_mutex_);
  octree_ptr_->pullOutPointCloud(cloud_octo, origin_octomap);
}

bool WorldRepresentation::isDrivable(const double x, const double y) {


  uint32_t grid_x = ceil((x - cost_map_.info.origin.position.x) / cost_map_.info.resolution);
  uint32_t grid_y = ceil((y - cost_map_.info.origin.position.y) / cost_map_.info.resolution);
  if (grid_x >= cost_map_.info.height || grid_y >= cost_map_.info.width)
    return false;

	if(x < buliding_bbox_.maxX() + 2 && x > buliding_bbox_.minX() - 2
		&& y < buliding_bbox_.maxY() + 2 && y > buliding_bbox_.minY() - 2)
	{
		return false;
	}
  int index = static_cast<int>(grid_x + cost_map_.info.height * grid_y);
  if (cost_map_.data[index] >= 10 || cost_map_.data[index] < 0)
    return false;
  return true;
}


void WorldRepresentation::publishOcTreeThread() {
  ros::Rate rate(publish_frequency_);
  octomap_msgs::Octomap octree_msg;
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(octree_mutex_);
      octomap_msgs::fullMapToMsg(*octree_ptr_, octree_msg);
    }

    octree_msg.header.stamp = ros::Time::now();
    octree_msg.header.frame_id = octree_frame_;
    octomap_publisher_.publish(octree_msg);

    rate.sleep();
  }
}

void WorldRepresentation::publishROIThread() {
  visualization_msgs::Marker bbox_marker;

  ros::Rate loop(2);
  while (ros::ok()) {
    if (roi_publisher_.getNumSubscribers() > 0) {
      // bbox_marker = rviz_utils::drawBoundingbox(occ_bbox_);
      bbox_marker = rviz_utils::drawBoundingbox(roi_);
      bbox_marker.header.frame_id = octree_frame_;
      bbox_marker.header.stamp = ros::Time::now();
      roi_publisher_.publish(bbox_marker);
    }
    loop.sleep();
  }
}

void WorldRepresentation::insertCloudThread() {
  while (ros::ok()) {
    if (cloud_list_.size() == 0) {
      std::this_thread::yield();
    } else {
      const std::pair<octomap::Pointcloud, octomap::point3d> &p = cloud_list_.front();
      std::lock_guard<std::mutex> lock_octree(octree_mutex_);
      octree_ptr_->insertPointCloud(p.first, p.second, buliding_bbox_); 
      std::lock_guard<std::mutex> lock_cloud_list(cloud_list_mutex_);
      cloud_list_.pop_front();
    }
  }
}

void WorldRepresentation::costmapCallback(const nav_msgs::OccupancyGridConstPtr &grid_msg) {
  cost_map_ = *grid_msg;
}



pcl::PointCloud<pcl::PointXYZ> WorldRepresentation::getOccupiedVoxelPointCloud() const {
  pcl::PointCloud<pcl::PointXYZ> res;
  std::lock_guard<std::mutex> locker(octree_mutex_);
  for (auto iter = octree_ptr_->begin_leafs(); iter != octree_ptr_->end_leafs(); ++iter) {
    if (octree_ptr_->isNodeOccupied(*iter)) {
      pcl::PointXYZ pt;
      Eigen::Vector3f mean = iter->mu();
      pt.x = mean.x();
      pt.y = mean.y();
      pt.z = mean.z();
      res.push_back(pt);
    }
  }
  return res;
}

bool WorldRepresentation::saveOctomapServerCallback(save_octomap::Request &request,
                                                          save_octomap::Response &response) {
  std::string file_path = request.file_path;
  if (file_path.empty()) {
    file_path = "octree.bt";
  }
  std::lock_guard<std::mutex> lock(octree_mutex_);
  this->octree_ptr_->write(request.file_path);
  response.success = true;
  return true;
}

void WorldRepresentation::saveOctomap(std::string file_path)
{
  if (file_path.empty()) {
    file_path = "octree.bt";
  }
  std::lock_guard<std::mutex> lock(octree_mutex_);
  this->octree_ptr_->write(file_path);
}

bool WorldRepresentation::inFieldOfView(const octomap::point3d &pt, const Eigen::Matrix4d &pose) const {
  static const float fov_vertical_limit_ = fov_vertical_ / 2 / 180 * M_PI;
  static const float fov_horizon_limit_ = fov_horizon_ / 2 / 180 * M_PI;

  Eigen::Vector4d pt_homo;
  pt_homo(0) = pt.x();
  pt_homo(1) = pt.y();
  pt_homo(2) = pt.z();
  pt_homo(3) = 1;

  Eigen::Vector3d pt_in_sensor = (pose.inverse() * pt_homo).block(0, 0, 3, 1);

  if (atan(fabs(pt_in_sensor.z() / pt_in_sensor.x())) < fov_vertical_limit_ &&
      atan(fabs(pt_in_sensor.y() / pt_in_sensor.x())) < fov_horizon_limit_) {
    return true;
  } else {
    return false;
  }
}

void WorldRepresentation::dilateFreeSpaceVoxelsFromOccupiedVoxels(const Eigen::Isometry3d &sensor_pose,
                                                                        const octomap::Boundingbox &update_bbox) {
  octomap::IgTree::leaf_bbx_iterator iter_begin, iter_end;
  if (update_bbox.isReset()) {
    octomap::OcTreeKey min_key(0, 0, 0), max_key(65535, 65535, 65535);
    iter_begin = octree_ptr_->begin_leafs_bbx(min_key, max_key);
  } else {
    iter_begin = octree_ptr_->begin_leafs_bbx(update_bbox.minPoint(), update_bbox.maxPoint());
  }

  octomap::KeySet voxels_to_dilate;
  octomap::point3d sensor_origin = octomap::toOctomap(sensor_pose.translation());

  {
    std::lock_guard<std::mutex> lock(octree_mutex_);

    for (auto it = iter_begin; it != iter_end; ++it) {
      const octomap::OcTreeKey &node_key = it.getKey();
      octomap::IgTreeNode *node = octree_ptr_->search(it.getKey());
      if (!node || !octree_ptr_->isNodeOccupied(node)) {
        continue;
      }
      voxels_to_dilate.insert(it.getKey());
    }
  }
  dilateFreeSpaceVoxels(sensor_pose, voxels_to_dilate);
}

void WorldRepresentation::dilateFreeSpaceVoxelsFromFreeVoxels(const Eigen::Isometry3d &sensor_pose,
                                                                    const std::vector<octomap::point3d> free_voxels) {
  octomap::KeySet voxels_to_dilate;
  for (int i = 0; i < free_voxels.size(); ++i) {
    octomap::OcTreeKey free_voxel_key = octree_ptr_->coordToKey(free_voxels[i]);
    voxels_to_dilate.insert(free_voxel_key);
  }
  int dilate_count = dilateFreeSpaceVoxels(sensor_pose, voxels_to_dilate);
}

int WorldRepresentation::dilateFreeSpaceVoxels(const Eigen::Isometry3d &sensor_pose,
                                                     const octomap::KeySet &voxels_to_dilate) {
  std::lock_guard<std::mutex> locker(octree_mutex_);
  TicToc tt;

  octomap::point3d sensor_origin = octomap::toOctomap(sensor_pose.translation());

  octomap::KeySet dilated_voxels;
  for (const octomap::OcTreeKey &key : voxels_to_dilate) {
    std::vector<octomap::OcTreeKey> neigh_keys;
    std::vector<octomap::IgTreeNode *> neighs;
    octomap::getNeighbor(*octree_ptr_, key, &neighs, &neigh_keys);
    for (int i = 0; i < neighs.size(); ++i) {
      octomap::point3d pt = octree_ptr_->keyToCoord(neigh_keys[i]);
      if (neighs[i] || !roi_.ifContain(pt) || !inFieldOfView(pt, sensor_pose.matrix())) {
        // if (neighs[i] || !roi_.ifContain(pt)) {
        continue;
      }

      dilated_voxels.insert(neigh_keys[i]);
    }
  }

  int cast_num = dilated_voxels.size();

  std::vector<octomap::point3d> cast_origins(cast_num), cast_dirs(cast_num);
  std::vector<double> cast_ranges(cast_num);
  int cast_count = 0;
  for (auto iter = dilated_voxels.begin(); iter != dilated_voxels.end(); ++iter) {
    octomap::point3d cast_end = octree_ptr_->keyToCoord(*iter);
    octomap::point3d cast_origin = sensor_origin;
    octomap::point3d cast_dir = cast_end - cast_origin;
    double cast_range = cast_dir.norm() + octree_ptr_->getResolution();

    cast_origins[cast_count] = cast_origin;
    cast_dirs[cast_count] = cast_dir;
    cast_ranges[cast_count] = cast_range;
	
    ++cast_count;
	
  }

  bool *find_end_pts;

  find_end_pts = new bool[cast_origins.size()];
  for (int i = 0; i < cast_origins.size(); ++i) {
    octomap::point3d end;
    find_end_pts[i] = octree_ptr_->castRay(cast_origins[i], cast_dirs[i], end, true, cast_ranges[i]);
  }

  int key_count = -1, new_free_voxels_num = 0;
  std::vector<octomap::OcTreeKey> keys_to_erase;
  for (auto iter = dilated_voxels.begin(); iter != dilated_voxels.end(); ++iter) {
    ++key_count;
    if (find_end_pts[key_count] == true) {
      octomap::IgTreeNode *node = octree_ptr_->search(*iter);
      if (node) {
        // this->deleteNode(*iter);
      }
      keys_to_erase.push_back(*iter);
      continue;
    } else {
      octomap::IgTreeNode *node = octree_ptr_->updateNode(*iter, false);
      // octree_hash_.insert(key, node, false);
      ++new_free_voxels_num;
    }
  }

  delete[] find_end_pts;

  return new_free_voxels_num;
}


pcl::PointCloud<pcl::PointXYZ> WorldRepresentation::getBuildingPointCloud() {	 
  pcl::PointCloud<pcl::PointXYZ> res;
  std::lock_guard<std::mutex> locker(octree_mutex_);
  std::vector<octomap::OcTreeKey> keyset = octree_ptr_->disjointSet().getUnionedVoxels(octree_ptr_->getBuildingRoot()); // work
  for(int i=0; i<keyset.size(); i++)
  {
	  octomap::point3d point = octree_ptr_->keyToCoord(keyset[i]);
		pcl::PointXYZ pt;
		pt.x = point.x();
		pt.y = point.y();
		pt.z = point.z();
		res.push_back(pt);
  }
  return res;
}


bool WorldRepresentation::savePoints(std::string file_name) {
  pcl::PointCloud<pcl::PointXYZ> full_pc = this->getBuildingPointCloud();
  pcl::io::savePCDFile(file_name, full_pc);
  return true;
}

};  // namespace active_mapping
