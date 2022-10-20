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

#include <pluginlib/class_list_macros.h>

#include <GeometricISM.h>


PLUGINLIB_EXPORT_CLASS(pointcloud_ogm::GeometricISM, nodelet::Nodelet)


namespace pointcloud_ogm {


GeometricISM::GeometricISM() {}


GeometricISM::~GeometricISM() {
  NODELET_INFO("GeometricISM stopped");
}


void GeometricISM::onInit() {

  node_handle_ = this->getMTNodeHandle();
  NODELET_INFO("GeometricISM starting...");

  // setup publisher and subscriber
  sub_ = node_handle_.subscribe("input", 1, &GeometricISM::messageCallback, this);
  pub_ = node_handle_.advertise<grid_map_msgs::GridMap>("/grid_map_measurement", 1);
}


void GeometricISM::messageCallback(const PointCloud::ConstPtr& pointcloud) {

  ROS_INFO("Received point cloud with timestamp: %lu", pointcloud->header.stamp);

  // Initialize new measurement grid map
  grid_map::GridMap grid_map_measurement;
  grid_map_measurement.setGeometry(grid_map::Length(150.0, 60.0), 0.3, grid_map::Position(0.0, 0.0));
  grid_map_measurement.add("occupancy_probability", 0.5);
  grid_map_measurement.add("elevation", 0);  // an elevation layer is only required because of a bug in the Rviz plugin

  // Iterate over all points in point cloud
  for(int i=0; i < pointcloud->points.size(); i++) {
    const auto& point = pointcloud->points[i];

    grid_map::Position start_position(point.x, point.y);
    grid_map::Position end_position(0.0, 0.0);

    int cell = 0;
    for (grid_map::LineIterator iterator(grid_map_measurement, start_position, end_position); !iterator.isPastEnd(); ++iterator) {
      auto& occupancy_probability = grid_map_measurement.at("occupancy_probability", *iterator);

      /* inverse sensor model:
          - cell containing reflection point: 90% occupancy probability
          - next two cells towards sensor: 80%, 50% occupancy probability
          - remaining cells towards sensor: 10% occupancy probability
      */
      double p_ism;
      // TASK 2 BEGIN
      // ADD YOUR CODE HERE...

      // TASK 2 END
      
      // combine probability from ism with previous probability in cell using binary Bayes filter
      occupancy_probability = (p_ism*occupancy_probability) / 
                              (p_ism*occupancy_probability + (1-p_ism)*(1-occupancy_probability));

      cell++;
    }

  }

  // set frame id and timestamp of grid map
  grid_map_measurement.setFrameId(pointcloud->header.frame_id);
  grid_map_measurement.setTimestamp(pointcloud->header.stamp*1e3);

  // publish measurement grid map
  grid_map_msgs::GridMapPtr grid_map_msg = grid_map_msgs::GridMapPtr(new grid_map_msgs::GridMap);
  grid_map::GridMapRosConverter::toMessage(grid_map_measurement, *grid_map_msg);
  pub_.publish(grid_map_msg);

}


}  // end of namespace pointcloud_ogm
