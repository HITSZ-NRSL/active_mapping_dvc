/*
 * Created on Thu Jan 21 2021
 *
 * Copyright (c) 2021 HITSZ-NRSL
 * All rights reserved
 *
 * Author: Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich,
 * Switzerland)
 * Modifier: EpsAvlc
 */

#ifndef ACTIVE_MAPPING_WORLD_REPRESENTATION_IG_TREE_H_
#define ACTIVE_MAPPING_WORLD_REPRESENTATION_IG_TREE_H_

#include <Eigen/Geometry>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <ros/ros.h>
#include <mutex>
#include <memory>
#include <utility>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include "active_mapping/utils/bounding_box.h"
#include "active_mapping/utils/disjoint_set.h"
#include "active_mapping/world_representation/octree_hash.h"

namespace octomap {

class IgTree;

class IgTreeNode : public octomap::ColorOcTreeNode {
 public:
  friend class IgTree;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IgTreeNode();
  ~IgTreeNode();

  bool operator==(const IgTreeNode &rhs) const;
  void copyData(const IgTreeNode &from);

  // -- node occupancy  ----------------------------

  /// \return occupancy probability of node
  inline double getOccupancy() const { return ::octomap::probability(value); }

  /// \return log odds representation of occupancy probability of node
  inline float getLogOdds() const { return value; }
  /// sets log odds occupancy of node
  inline void setLogOdds(float l) { value = l; }

  /**
   * @return mean of all children's occupancy probabilities, in log odds
   */
  double getMeanChildLogOdds() const;

  /**
   * @return maximum of children's occupancy probabilities, in log odds
   */
  float getMaxChildLogOdds() const;

  /**
   * @brief adds p to the node's logOdds value (with no boundary / threshold
   *        checking!)
   *
   * @param p
   */
  void addValue(const float &p);

  /**
   * @brief whether this node has been measured or not. (Not used)
   *
   * @return true
   * @return false
   */
  bool hasMeasurement() { return !has_no_measurement_; }
  /**
   * @brief Not uesd
   *
   * @param hasMeasurement
   */
  void updateHasMeasurement(bool hasMeasurement) { has_no_measurement_ = !hasMeasurement; }

  /**
   * @brief This function is a serialization function. It is VITAL for octomap rivz plugins' right visualization.
   * 
   * @param [in] s input stream
   * @return std::istream& return the stream
   */
  std::istream &readData(std::istream &s);

  /**
   * @brief This function is a serialization function. It is VITAL for octomap rivz plugins' right visualization. Function readData and writeData should have the same order of reading and writing data.
   * 
   * @param s 
   * @return std::ostream& 
   */
  std::ostream &writeData(std::ostream &s) const;

  /**
   * @brief Return if a voxel is a frontier
   * 
   * @return true if this voxel is a frontier.
   * @return false if this voxel is not a frontier
   */
  bool isFrontier() { return is_frontier_; }

  /**
   * @brief Set frontier tag
   * 
   * @param [in] is_frontier boolean.
   */
  void setFrontier(bool is_frontier) { is_frontier_ = is_frontier; }

  /**
   * @brief 
   * 
   * @param pt 
   */
  void insertPoint(const octomap::point3d &pt);

  void deletePoint(const octomap::point3d &pt);

  Eigen::Vector3f mu() { return mu_; }

  octomap::point3d muOcto() {
    octomap::point3d pt;
    pt.x() = mu_.x();
    pt.y() = mu_.y();
    pt.z() = mu_.z();
    return pt;
  }

  Eigen::Matrix3f cov() { return sigma_; }

  int pointCount() const { return pt_cnt_; }

 protected:
  bool has_no_measurement_ = true;
  bool is_frontier_ = false;
  /** NDT parameters **/
  Eigen::Vector3f mu_ = Eigen::Vector3f::Zero();
  Eigen::Matrix3f sigma_ = Eigen::Matrix3f::Zero();
  int pt_cnt_ = 0;
};

class IgTree : public ::octomap::OccupancyOcTreeBase<IgTreeNode> {
 public:
  typedef IgTreeNode NodeType;
  typedef std::shared_ptr<IgTree> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /*! Configuration for the IgTree
   */
  struct Config {
   public:
    Config();

   public:
    double resolution_m;            //! OcTree leaf node size, default: 0.1 [m].
    double occupancy_threshold;     //! Occupancy probability over which nodes are
                                    //! considered occupied, default: 0.5 [range 0-1].
    double hit_probability;         //! Probability update value for hits, default 0.7
                                    //! [range 0-1].
    double miss_probability;        //! Probability update value for misses, default
                                    //! 0.4 [range 0-1].
    double clamping_threshold_min;  //! Min probability threshold over which the
                                    //! probability is clamped, default: 0.12,
                                    //! range [0-1].
    double clamping_threshold_max;  //! Max probability threshold over which the
                                    //! probability is clamped, default: 0.97,
                                    //! range [0-1].
  };

 public:
  //! Default constructor, sets resolution of leafs
  explicit IgTree(double resolution_m, bool use_disjoint_set = false);

  /*! Constructor with complete configuration
   */
  explicit IgTree(Config config);

  /*! virtual constructor: creates a new object of same type
   * (Covariant return type requires an up-to-date compiler)
   */
  IgTree *create() const;

  std::string getTreeType() const;

  /*! Returns the current configuration.
   */
  const Config &config() const;

  unsigned int getTreeMaxVal() const { return tree_max_val; }

  /// update this node's occupancy according to its children's maximum occupancy
  inline void updateOccupancyChildren(IgTreeNode *node) {
    node->setLogOdds(node->getMaxChildLogOdds());  // conservative
  }

  void expandNode(IgTreeNode *node);
  bool pruneNode(IgTreeNode *node);

  /***************** Color Methods *********************/
  // set node color at given key or coordinate. Replaces previous color.
  IgTreeNode *setNodeColor(const ::octomap::OcTreeKey &key, uint8_t r, uint8_t g, uint8_t b);

  IgTreeNode *setNodeColor(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) {
    ::octomap::OcTreeKey key;
    if (!this->coordToKeyChecked(::octomap::point3d(x, y, z), key))
      return NULL;
    return setNodeColor(key, r, g, b);
  }

  /**
   * @brief insert point cloud into octomap
   *
   * @param pc point cloud
   * @param origin sensor origin
   */
  void insertPointCloud(const octomap::Pointcloud &pc, const octomap::point3d &origin, octomap::Boundingbox &building_box);

  /**
   * @brief pull out point cloud from octomap
   *
   * @param pc pull out point cloud
   * @param origin sensor origin
   */
  void pullOutPointCloud(const octomap::Pointcloud &pc, const octomap::point3d &origin);

  /**
   * @brief delete octree nodes outside a bounding box.
   *
   * @param bbox
   */
  void deleteNodesOutsideBbox(const octomap::Boundingbox &bbox);

  DisjointSet<OcTreeKey, OcTreeKey::KeyHash> &disjointSet() { return disjoint_set_; }

  void setBuildingRoot(const OcTreeKey &key) { building_root_ = key; has_building_root_ = true;}
  const OcTreeKey getBuildingRoot() {
    OcTreeKey root = disjoint_set_.find(building_root_);
    if (root != building_root_) {
      building_root_ = root;
    }
    return building_root_;
  }

  int getBuildingVoxelsNum();
  std::vector<OcTreeKey> getBuildingVoxels();
  OcTreeHash<IgTree> &octreeHash() { return octree_hash_; }
  const OcTreeHash<IgTree> &octreeHash() const { return octree_hash_; }

  // std::mutex &getInsertMutex() { return insert_mutex_; }

  void getNeighborhoodAtPoint27(const octomap::point3d &pt, std::vector<octomap::IgTreeNode *> *neighborhood);

  void getNeighborhoodAtPoint7(const octomap::point3d &pt, std::vector<octomap::IgTreeNode *> *neighborhood);

  void getNeighborhoodAtPoint1(const octomap::point3d &pt, std::vector<octomap::IgTreeNode *> *neighborhood);

  void setTrunctedLength(double length) { truncted_length_ = length; }
 protected:
  /*! Sets octree options based on current configuration
   */
  void updateOctreeConfig();

 protected:
  Config config_;

  /**
   * Static member object which ensures that this OcTree's prototype
   * ends up in the classIDMapping only once. You need this as a
   * static member in any derived octree class in order to read .ot
   * files through the AbstractOcTree factory. You should also call
   * ensureLinking() once from the constructor.
   */
  class StaticMemberInitializer {
   public:
    StaticMemberInitializer() {
      IgTree *tree = new IgTree(0.1);
      tree->clearKeyRays();
      ::octomap::AbstractOcTree::registerTreeType(tree);
    }

    /**
     * Dummy function to ensure that MSVC does not drop the
     * StaticMemberInitializer, causing this tree failing to register.
     * Needs to be called from the constructor of this octree.
     */
    void ensureLinking() {}
  };
  /// static member to ensure static initialization (only once)
  static StaticMemberInitializer igTreeMemberInit;

 private:
  bool isNodeUnknown(octomap::IgTreeNode *node_ptr);

  std::vector<point3d> inflate_sphere_;
  bool use_inflation_;
  bool use_disjoint_set_ = false;
  DisjointSet<OcTreeKey, OcTreeKey::KeyHash> disjoint_set_;
  OcTreeKey building_root_;
  bool has_building_root_ = false;
  // mutable std::mutex insert_mutex_;

  OcTreeHash<IgTree> octree_hash_;
  double truncted_length_ = 1;
};

}  // namespace octomap

#endif  // ACTIVE_MAPPING_WORLD_REPRESENTATION_IG_TREE_H_
