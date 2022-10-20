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

#include <string>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

// Point Cloud
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Grid Map
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>


namespace pointcloud_ogm {

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class GeometricISM : public nodelet::Nodelet {

  public:

    GeometricISM();
    ~GeometricISM();
    virtual void onInit();

  private:

    void messageCallback(const PointCloud::ConstPtr& pointcloud);

  private:

    ros::NodeHandle node_handle_;

    ros::Subscriber sub_;
    ros::Publisher pub_;
};


}  // end of namespace
