// Copyright (c) 2022 Institute for Automotive Engineering of RWTH Aachen University

// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#pragma once

#define _USE_MATH_DEFINES
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>

#include <definitions.h>

#include <Eigen/Dense>
#include <tensorflow/core/framework/tensor.h>

namespace lidar_detection
{

  // type definitions
  typedef pcl::PointCloud<pcl::PointXYZI> LidarPointCloud;

  struct IntPairHash
  {
    std::size_t operator()(const std::pair<uint32_t, uint32_t> &p) const;
  };

  /**
   * @brief Struct defining all dimensions of the input feature network of PointPillars
   *
   */
  struct PillarPoint
  {
    float x;
    float y;
    float z;
    float intensity;
    float xc = 0.0;
    float yc = 0.0;
    float zc = 0.0;
  };

  /**
   * @brief Creates the pillar tensors from a point cloud as defined within PointPillars
   *
   * @param cloud
   * @param config
   * @param pillars
   * @param indices
   */
  void createPillars(
      const LidarPointCloud &cloud,
      const SlikafConfig &config, tensorflow::Tensor &pillars,
      tensorflow::Tensor &indices);

  void pointsToPbodInput(
      const LidarPointCloud &cloud,
      const SlikafConfig &config,
      tensorflow::Tensor &points_xyz,
      tensorflow::Tensor &points_feature,
      tensorflow::Tensor &points_mask);

  float wrap_angle_rad(float angles_rad, float min_val = -M_PI, float max_val = M_PI);

  std::vector<float> linspace(float start_in, float end_in, int num_in);

} // namespace lidar_detection
