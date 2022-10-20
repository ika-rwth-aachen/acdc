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

#include <ros/ros.h>
#include <nodelet/nodelet.h>

// Point Cloud
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Grid Map
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

// Tensorflow C++ API
#include <tensorflow_cpp/utils.h>
#include <tensorflow_cpp/model.h>

// pointcloud_ogm
#include <pillar_utilities.h>


namespace pointcloud_ogm {

namespace tf = tensorflow;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class DeepISM : public nodelet::Nodelet {

  public:

    DeepISM();
    ~DeepISM();
    virtual void onInit();

  private:

    void messageCallback(const PointCloud::ConstPtr& pointcloud);
    void tensor_to_grid_map(const float* prediction, grid_map::GridMap& grid_map);

  private:

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;

    std::string model_path_;
    double grid_map_cell_size;
    int grid_height;
    int grid_width;
    int grid_cell_layers;
    pillar_utilities::LidarConfig config;

    ros::Publisher pub_;
    ros::Subscriber sub_;

    const std::string INPUT_NAME_PILLARS = "pillars/input";
    const std::string INPUT_NAME_INDICES = "pillars/indices";
    const std::string OUTPUT_NAME = "ogm/conv2d";
    tensorflow_cpp::Model model_;
    
    std::vector<std::pair<std::string, tf::Tensor>> model_inputs;
    std::vector<std::string> model_outputs;
    std::vector<tf::Tensor> model_output_tensors;

    grid_map::GridMap grid_map;

};


}  // end of namespace pointcloud_ogm
