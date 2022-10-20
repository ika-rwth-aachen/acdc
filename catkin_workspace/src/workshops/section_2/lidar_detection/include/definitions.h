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

#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <boost/optional.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace lidar_detection
{

  /**
   * @brief Enumeration for all available message formats
   *
   */
  enum Format
  {
    ika = 1
  };

  /**
   * @brief Struct defines a cartesian coordinate with optional z value
   *
   */
  struct CartesianCoordinate
  {
    CartesianCoordinate(double x_coordinate, double y_coordinate)
        : x(x_coordinate), y(y_coordinate){};
    double x;
    double y;
    double z;

    double &operator[](int index)
    {
      if (index == 0)
      {
        return x;
      }
      else if (index == 1)
      {
        return y;
      }
      else
      {
        throw std::out_of_range("Coordinate out of range!");
      }
    }
  };

  /**
   * @brief Struct defines a 2D bounding box by its vetices
   *
   */
  struct BoundingBoxVertex
  {
    // Cartesian coordinates to describe the bounding box in the x, y plane.
    CartesianCoordinate front_right;
    CartesianCoordinate front_left;
    CartesianCoordinate rear_left;
    CartesianCoordinate rear_right;

    /**
     * @brief Return the 2D coordinate of the bounding box in the x, y plane.
     * The indices start at front right and move counter clockwise around the
     * vehicle.
     *
     * @param index
     * @return CartesianCoordinate&
     */
    CartesianCoordinate &operator[](int index)
    {
      if (index == 0)
      {
        return front_right;
      }
      else if (index == 1)
      {
        return front_left;
      }
      else if (index == 2)
      {
        return rear_left;
      }
      else if (index == 3)
      {
        return rear_right;
      }
      else
      {
        throw std::out_of_range("BoundingBox out of range!");
      }
    }
  };

  /**
   * @brief Struct defines the maximum and minimum of the four vertices of a 2D bounding box
   *
   */
  struct BoundingBoxMaxVertex
  {
    // Cartesian coordinates to describe the bounding box in the x, y plane.
    Eigen::Vector2d min;
    Eigen::Vector2d max;

    float score;
  };

  /**
   * @brief Struct defines a bounding box by its midpoint the dimensions and the heading
   *
   */
  struct BoundingBoxCenter
  {
    // Cartesian coordinate to describe the bounding box center in the x, y plane
    Eigen::Vector2d center;
    float z;
    float length;
    float width;
    float height;
    float yaw;

    float score;
    int class_idx;
    std::string class_name;
  };

  /**
   * @brief Struct defines all available parameters in a slikaf config file for lidar object detection
   *
   */
  struct SlikafConfig
  {
    void initialize(ros::NodeHandle &nh);
    int intensity_threshold = 10000;

#ifdef MODE_PP

    // anchor
    struct Anchor
    {
      float height;
      float width;
      float length;
      float z_center;
      float orientation;
    };

    // grid
    float x_min = 0.0;
    float x_max = 0.0;
    float y_min = 0.0;
    float y_max = 0.0;
    float z_min = 0.0;
    float z_max = 0.0;
    float delta_x = 0.16;
    float delta_y = 0.16;

    // network
    int max_points_per_pillar = 1;
    int max_pillars = 10000;
    int n_features = 9;
    int n_channels = 64;
    float downscaling = 2;

    // anchors.
    std::vector<Anchor> anchor_boxes;
    std::vector<double> anchor_diagonals;

    // postprocessing
    float class_amount;
    std::vector<std::string> class_names;
    std::vector<float> score_thresh;
    float minimum_thresh;
    float nms_max_objects;
    float nms_iou_thresh;
#endif
  };

  /**
   * @brief Struct defines all available parameters for lidar object detection
   *
   */
  struct Params
  {
    void initialize(ros::NodeHandle &nh);

    // required parameters
    std::string input_topic;
    std::string model_name;
    std::string model_checkpoint;

    // optional parameters
    std::string inference_frame;
    std::string output_frame;
    bool publish_pcl;

    // maximal object dimensions
    float max_length;
    float max_width;
    float max_height;

    // variance vector
    std::vector<float> variance;
  };

} // namespace lidar_detection