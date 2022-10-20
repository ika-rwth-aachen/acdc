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

#include <DeepISM.h>


PLUGINLIB_EXPORT_CLASS(pointcloud_ogm::DeepISM, nodelet::Nodelet)


namespace pointcloud_ogm {


DeepISM::DeepISM() {}


DeepISM::~DeepISM() {
  NODELET_INFO("DeepISM stopped");
}


void DeepISM::onInit() {

  // NodeHandles cannot be used before this point
  node_handle_ = this->getMTNodeHandle();
  private_node_handle_ = this->getPrivateNodeHandle();
  NODELET_INFO("DeepISM starting...");

  // get parameters from ROS parameter server
  private_node_handle_.getParam("model_path", model_path_);
  private_node_handle_.getParam("grid_map/cell_size", grid_map_cell_size);
  private_node_handle_.getParam("pointcloud/x_min", config.x_min);
  private_node_handle_.getParam("pointcloud/x_max", config.x_max);
  private_node_handle_.getParam("pointcloud/y_min", config.y_min);
  private_node_handle_.getParam("pointcloud/y_max", config.y_max);
  private_node_handle_.getParam("pointcloud/z_min", config.z_min);
  private_node_handle_.getParam("pointcloud/z_max", config.z_max);
  private_node_handle_.getParam("pointcloud/min_distance", config.minDistance);
  private_node_handle_.getParam("pointcloud/step_x_size", config.x_step);
  private_node_handle_.getParam("pointcloud/step_y_size", config.y_step);
  private_node_handle_.getParam("pointcloud/intensity_threshold", config.intensity_threshold);

  model_.loadModel(model_path_);
  NODELET_INFO("Loaded lidar grid mapping model from '%s'", model_path_.c_str());

  auto shape_input_node_pillars = model_.getNodeShape(INPUT_NAME_PILLARS);
  config.max_pillars = shape_input_node_pillars[1];
  config.max_points_per_pillar = shape_input_node_pillars[2];
  config.num_features = shape_input_node_pillars[3];
  
  auto shape_input_node_indices = model_.getNodeShape(INPUT_NAME_INDICES);
  config.index_size = shape_input_node_indices[2];
  
  auto shape_output_node_ogm = model_.getNodeShape(OUTPUT_NAME);
  grid_height = shape_output_node_ogm[1];
  grid_width = shape_output_node_ogm[2];
  grid_cell_layers = shape_output_node_ogm[3];

  NODELET_INFO("Input Pillars: Max pillars %i, max points per pillar %i, point features %i",
    config.max_pillars, config.max_points_per_pillar, config.num_features);
  NODELET_INFO("Input Indices: Index size %i", config.index_size);
  NODELET_INFO("Output Grid Map: height %i, width %i, cell layers %i", grid_height, grid_width, grid_cell_layers);

  tf::Tensor input_tensor_pillars(tf::DT_FLOAT, tf::TensorShape({ 1, config.max_pillars, config.max_points_per_pillar, config.num_features }));
  tf::Tensor input_tensor_indices(tf::DT_INT32, tf::TensorShape({ 1, config.max_pillars, config.index_size }));
  model_inputs.push_back({ INPUT_NAME_PILLARS, input_tensor_pillars });
  model_inputs.push_back({ INPUT_NAME_INDICES, input_tensor_indices });
  model_outputs.push_back(OUTPUT_NAME);

  // initialize grid map
  grid_map.setGeometry(
      grid_map::Length(grid_map_cell_size * grid_height, grid_map_cell_size * grid_width), grid_map_cell_size,
      grid_map::Position(0, 0));
  grid_map.add("m_occupied", 0.0);
  grid_map.add("m_free", 0.0);
  grid_map.add("occupancy_color", 0.0);
  grid_map.add("elevation", 0);  // an elevation layer is only required because of a bug in the Rviz plugin
  NODELET_INFO("Initialized grid map with size %f m x %f m (%i x %i x %lu cells).",
                grid_map.getLength().x(),
                grid_map.getLength().y(),
                grid_map.getSize()(0),
                grid_map.getSize()(1),
                grid_map.getLayers().size());

  // setup publisher and subscriber
  sub_ = node_handle_.subscribe("input", 1, &DeepISM::messageCallback, this);
  pub_ = node_handle_.advertise<grid_map_msgs::GridMap>("/grid_map_measurement", 1);
}


void DeepISM::messageCallback(const PointCloud::ConstPtr& pointcloud) {

  ROS_INFO("Received point cloud with timestamp: %lu", pointcloud->header.stamp);
  grid_map.setFrameId(pointcloud->header.frame_id);
  grid_map.setTimestamp(pointcloud->header.stamp*1e3);

  // create input tensor from point cloud
  pillar_utilities::createPillars(*pointcloud, config, static_cast<float*>(model_inputs[0].second.data()), static_cast<int*>(model_inputs[1].second.data()));
  
  // inference step
  auto model_output_tensors = model_(model_inputs, model_outputs);
  if (model_output_tensors.empty())
    return;
  tf::Tensor& model_output_tensor = model_output_tensors[OUTPUT_NAME];

  // convert output tensor to grid map
  tensor_to_grid_map(static_cast<float*>(model_output_tensor.data()), grid_map);

  // publish grid map
  grid_map_msgs::GridMapPtr grid_map_msg = grid_map_msgs::GridMapPtr(new grid_map_msgs::GridMap);
  grid_map::GridMapRosConverter::toMessage(grid_map, *grid_map_msg);
  pub_.publish(grid_map_msg);
}


void DeepISM::tensor_to_grid_map(const float* prediction, grid_map::GridMap& grid_map)
{
  // TASK 5 START: Find the bug in the following code.
  const auto& rows = grid_map.getSize()(0);
  const auto& cols = grid_map.getSize()(1);
  for (int row = 0; row < rows; row++)
  {
    for (int col = 0; col < cols; col++)
    {
      const auto& evidence_free = prediction[0 + row*cols*2 + col*2 + 0];
      const auto& evidence_occupied = prediction[0 + row*cols*2 + col*2 + 1];
      auto& m_occupied = grid_map.at("m_occupied", grid_map::Index(row, col));
      auto& m_free = grid_map.at("m_free", grid_map::Index(row, col));
      auto& occupancy_color = grid_map.at("occupancy_color", grid_map::Index(row, col));

      const auto& S = (1 + evidence_occupied) + (1 + evidence_free);
      m_occupied = evidence_occupied;
      m_free = evidence_free;

      // create occupancy color for visualization (m_occupied = red, m_free = green)
      grid_map::colorVectorToValue(Eigen::Vector3f(m_occupied, m_free, 0), occupancy_color);
    }
  }
  // TASK 5 END
}


}  // end of namespace pointcloud_ogm
