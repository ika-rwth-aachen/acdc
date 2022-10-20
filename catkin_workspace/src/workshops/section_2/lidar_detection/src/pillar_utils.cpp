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

#include <pillar_utils.h>

namespace lidar_detection
{

  std::size_t IntPairHash::operator()(const std::pair<uint32_t, uint32_t> &p) const
  {
    assert(sizeof(std::size_t) >= 8);
    // shift first integer over to make room for the second integer. The two are then packed side by side.
    return (((uint64_t)p.first) << 32) | ((uint64_t)p.second);
  }

#ifdef MODE_PP
  void createPillars(
      const LidarPointCloud &cloud,
      const SlikafConfig &config, tensorflow::Tensor &pillars,
      tensorflow::Tensor &indices)
  {

    std::unordered_map<std::pair<uint32_t, uint32_t>, std::vector<PillarPoint>, IntPairHash> map;

    for (LidarPointCloud::const_iterator point = cloud.begin(); point != cloud.end(); ++point)
    {
      // filter points to detection range
      if ((point->x < config.x_min) || (point->x >= config.x_max) ||
          (point->y < config.y_min) || (point->y >= config.y_max) ||
          (point->z < config.z_min) || (point->z >= config.z_max))
      {
        continue;
      }
      auto x_index = static_cast<uint32_t>(
          std::floor((point->x - config.x_min) / config.delta_x));
      auto y_index = static_cast<uint32_t>(
          std::floor((point->y - config.y_min) / config.delta_y));

      float intensity = point->intensity / config.intensity_threshold;
      intensity = std::min(intensity, (float)1.0);
      intensity = std::max(intensity, (float)0.0);

      map[{x_index, y_index}].emplace_back(
          PillarPoint{point->x, point->y, point->z, intensity, 0.0f, 0.0f, 0.0f});
    }

    // safe tensor to an Eigen tensor map to improvement time
    Eigen::TensorMap<Eigen::Tensor<float, 4, 1, long int>, 16, Eigen::MakePointer>
        pillar_map = pillars.tensor<float, 4>();
    Eigen::TensorMap<Eigen::Tensor<int, 3, 1, long int>, 16, Eigen::MakePointer>
        indices_map = indices.tensor<int, 3>();

    // iterate over all pillars
    int pillar_id = 0;
    for (auto &pair : map)
    {
      // check max_pillars
      if (pillar_id >= config.max_pillars)
      {
        break;
      }

      // calculate pillar mean
      float x_mean = 0;
      float y_mean = 0;
      float z_mean = 0;
      for (const auto &p : pair.second)
      {
        x_mean += p.x;
        y_mean += p.y;
        z_mean += p.z;
      }
      x_mean /= pair.second.size();
      y_mean /= pair.second.size();
      z_mean /= pair.second.size();

      // calculate deviation to mean
      for (auto &p : pair.second)
      {
        p.xc = p.x - x_mean;
        p.yc = p.y - y_mean;
        p.zc = p.z - z_mean;
      }

      // save indices in map
      const auto &x_index = pair.first.first;
      const auto &y_index = pair.first.second;
      indices_map(0, pillar_id, 1) = x_index;
      indices_map(0, pillar_id, 2) = y_index;

      // iterate over all points in a pillar
      int point_id = 0;
      for (const auto &p : pair.second)
      {
        // check max_points_per_pillar
        if (point_id >= config.max_points_per_pillar)
        {
          break;
        }

        // build 9 dimensional network input from here (chapter 2.1 https://arxiv.org/pdf/1812.05784.pdf)
        pillar_map(0, pillar_id, point_id, 0) = p.x;
        pillar_map(0, pillar_id, point_id, 1) = p.y;
        pillar_map(0, pillar_id, point_id, 2) = p.z;
        pillar_map(0, pillar_id, point_id, 3) = p.intensity;
        // subscript c refers to the distance to the pillar mean
        pillar_map(0, pillar_id, point_id, 4) = p.xc;
        pillar_map(0, pillar_id, point_id, 5) = p.yc;
        pillar_map(0, pillar_id, point_id, 6) = p.zc;
        // subscript p refers to the offset to the pillar center
        pillar_map(0, pillar_id, point_id, 7) = p.x - (x_index * config.delta_x + config.x_min);
        pillar_map(0, pillar_id, point_id, 8) = p.y - (y_index * config.delta_y + config.y_min);

        ++point_id;
      }

      ++pillar_id;
    }
  }

#endif

  float wrap_angle_rad(float angles_rad, float min_val, float max_val)
  {
    //"""Wrap the value of `angles_rad` to the range [min_val, max_val]."""
    float max_min_diff = max_val - min_val;
    return min_val + std::fmod(angles_rad + max_val, max_min_diff);
  }

  std::vector<float> linspace(float start_in, float end_in, int num_in)
  {

    std::vector<float> linspaced;

    float num = static_cast<float>(num_in);

    if (num == 0)
    {
      return linspaced;
    }
    if (num == 1)
    {
      linspaced.push_back(start_in);
      return linspaced;
    }

    float delta = (end_in - start_in) / (num - 1);

    for (int i = 0; i < num - 1; ++i)
    {
      linspaced.push_back(start_in + delta * i);
    }
    linspaced.push_back(end_in); // I want to ensure that start and end
                                 // are exactly the same as the input
    return linspaced;
  }

} // namespace lidar_detection
