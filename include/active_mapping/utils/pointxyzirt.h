/*
 * Created on Thu Apr 08 2021
 *
 * Copyright (c) 2021 HITsz-NRSL
 *
 * Author: EpsAvlc
 */

#ifndef ACTIVE_MAPPING_POINTXYZIRT_H__
#define ACTIVE_MAPPING_POINTXYZIRT_H__
#include <pcl/point_types.h>

struct PointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
  (uint16_t, ring, ring)(double, timestamp, timestamp))

#endif  // !ACTIVE_MAPPING_POINTXYZIRT_H__
