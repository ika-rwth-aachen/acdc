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
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <unordered_map>

namespace pillar_utilities
{
struct LidarConfig
{
  float x_min;
  float x_max;
  float y_min;
  float y_max;
  float z_min;
  float z_max;
  float x_step;
  float y_step;
  float intensity_threshold;
  // Config values for point pillar network.
  int max_pillars;
  int max_points_per_pillar;
  int num_features;
  int index_size;
  float minDistance;
};

struct IntPairHash
{
  std::size_t operator()(const std::pair<uint32_t, uint32_t>& p) const;
};

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

void createPillars(const pcl::PointCloud<pcl::PointXYZI>& cloud, const LidarConfig& config, float* pillars,
                   int* indices);

}  // namespace pillar_utilities
