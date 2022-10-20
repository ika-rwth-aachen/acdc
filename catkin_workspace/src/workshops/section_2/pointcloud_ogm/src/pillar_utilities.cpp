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

#include <pillar_utilities.h>

namespace pillar_utilities
{
std::size_t IntPairHash::operator()(const std::pair<uint32_t, uint32_t>& p) const
{
  assert(sizeof(std::size_t) >= 8);
  // Shift first integer over to make room for the second integer. The two are
  // then packed side by side.
  return (((uint64_t)p.first) << 32) | ((uint64_t)p.second);
}

template <class T>
const T& clamp(const T& v, const T& lo, const T& hi) {
  assert(!(hi < lo));
  return (v < lo) ? lo : (hi < v) ? hi : v;
}

void createPillars(const pcl::PointCloud<pcl::PointXYZI>& cloud, const LidarConfig& config, float* pillars,
                   int* indices)
{
  std::unordered_map<std::pair<uint32_t, uint32_t>, std::vector<PillarPoint>, IntPairHash> map;

  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator point = cloud.begin(); point != cloud.end(); ++point)
  {
    // Filter points to detection range of Lidar model.
    if ((point->x < config.x_min) || (point->x >= config.x_max) || 
        (point->y < config.y_min) || (point->y >= config.y_max) ||
        (point->z < config.z_min) || (point->z >= config.z_max) ||
        config.minDistance > 0 && (std::pow(point->x, 2) + std::pow(point->y, 2)) < std::pow(config.minDistance, 2))
    {
      continue;
    }
    auto x_index = static_cast<uint32_t>(std::floor((point->x - config.x_min) / config.x_step));
    auto y_index = static_cast<uint32_t>(std::floor((point->y - config.y_min) / config.y_step));

    PillarPoint p = {
      point->x,
      point->y,
      point->z,
      clamp(point->intensity / config.intensity_threshold, 0.0f, 1.0f),
      0,
      0,
      0,
    };

    map[{ x_index, y_index }].emplace_back(p);
  }

  // initialize tensors with zeros
  std::fill(pillars, pillars + 1*config.max_pillars*config.max_points_per_pillar*config.num_features, 0);
  std::fill(indices, indices + 1*config.max_pillars*config.index_size, 0);

  int pillar_id = 0;
  for (auto& pair : map)
  {
    if (pillar_id >= config.max_pillars)
    {
      break;
    }

    float x_mean = 0;
    float y_mean = 0;
    float z_mean = 0;
    for (const auto& p : pair.second)
    {
      x_mean += p.x;
      y_mean += p.y;
      z_mean += p.z;
    }
    x_mean /= pair.second.size();
    y_mean /= pair.second.size();
    z_mean /= pair.second.size();

    for (auto& p : pair.second)
    {
      p.xc = p.x - x_mean;
      p.yc = p.y - y_mean;
      p.zc = p.z - z_mean;
    }

    const auto& x_index = pair.first.first;
    const auto& y_index = pair.first.second;
    indices[0 + pillar_id*3 + 0] = 0;
    indices[0 + pillar_id*3 + 1] = x_index;
    indices[0 + pillar_id*3 + 2] = y_index;

    int point_id = 0;
    for (const auto& p : pair.second)
    {
      if (point_id >= config.max_points_per_pillar)
      {
        break;
      }

      // Chapter 2.1 https://arxiv.org/pdf/1812.05784.pdf. 9 dimensional input
      // to network.
      //std::cout << "pillar: " << pillar_id << ", point: " << point_id << "\n";
      pillars[0 + pillar_id*config.max_points_per_pillar*config.num_features + point_id*config.num_features + 0] = p.x;
      pillars[0 + pillar_id*config.max_points_per_pillar*config.num_features + point_id*config.num_features + 1] = p.y;
      pillars[0 + pillar_id*config.max_points_per_pillar*config.num_features + point_id*config.num_features + 2] = p.z;
      pillars[0 + pillar_id*config.max_points_per_pillar*config.num_features + point_id*config.num_features + 3] = p.intensity;
      // Subscript c refers to the distance to the arithmetic mean of all points
      // in the pillar.
      pillars[0 + pillar_id*config.max_points_per_pillar*config.num_features + point_id*config.num_features + 4] = p.xc;
      pillars[0 + pillar_id*config.max_points_per_pillar*config.num_features + point_id*config.num_features + 5] = p.yc;
      pillars[0 + pillar_id*config.max_points_per_pillar*config.num_features + point_id*config.num_features + 6] = p.zc;
      // Subscript p offset from pillar center in x and y.
      pillars[0 + pillar_id*config.max_points_per_pillar*config.num_features + point_id*config.num_features + 7] = p.x - (x_index * config.x_step + config.x_min);
      pillars[0 + pillar_id*config.max_points_per_pillar*config.num_features + point_id*config.num_features + 8] = p.y - (y_index * config.y_step + config.y_min);
      
      ++point_id;
    }

    ++pillar_id;
  }
}

}  // namespace pillar_utilities
