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

#include <IkaFusionWrapperNode.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "object_fusion_wrapper_node");
  ros::NodeHandle node_handle("~");

  IkaFusionWrapper ika_fusion_wrapper = IkaFusionWrapper(node_handle);
  ika_fusion_wrapper.loadParameters();
  ika_fusion_wrapper.initiatePublisherAndSubscriber();
  ros::spin();

  return 0;
}

IkaFusionWrapper::IkaFusionWrapper(const ros::NodeHandle &node_handle)
  : ika_fusion_library_(new ikaFusLib), node_handle(node_handle) {
  ego_motion_memory_ = definitions::IkaEgoMotion();
}

IkaFusionWrapper::~IkaFusionWrapper(){
  delete object_list_fused_publisher_;
  delete ika_fusion_library_;
}

void IkaFusionWrapper::callbackObjectList(const definitions::IkaObjectListConstPtr &msg) {
  definitions::IkaObjectList object_list_measured = *msg;
  IkaUtilities::trimIkaObjectList(&object_list_measured);

  auto object_list_fused = ika_fusion_library_->fuseIntoGlobal(
        object_list_measured, ego_motion_memory_);
  
  object_list_fused.header.frame_id = this->publish_frame_id_object_list_fused;

  if (object_list_fused_publisher_ != nullptr) {
    object_list_fused_publisher_->publish(object_list_fused);
  }
}

void IkaFusionWrapper::callbackEgoMotion(const definitions::IkaEgoMotionConstPtr &msg) {
  ego_motion_memory_ = *msg;
}

void IkaFusionWrapper::initiatePublisherAndSubscriber(){
  ROS_INFO("Setup publisher.");
  if (this->publish_object_list_fused) {
    this->object_list_fused_publisher_ = new ros::Publisher();
    *this->object_list_fused_publisher_ =
        this->node_handle.advertise<definitions::IkaObjectList>(this->publish_topic_object_list_fused, 1000);
  }

  ROS_INFO("Setup subscriber.");
  ROS_INFO_STREAM("Fusion will listen the following IkaObjectList topics:");
  for (const auto &value : this->subscribe_topics) {
    ros::Subscriber subscriber_object_lists = this->node_handle.subscribe(value, 1000, &IkaFusionWrapper::callbackObjectList, this);
    this->subscriber.push_back(std::move(subscriber_object_lists));
    ROS_INFO_STREAM("- " << value.c_str());
  }
  ros::Subscriber subscriber_ego_motion = node_handle.subscribe(this->subscribe_ego_motion, 1000, &IkaFusionWrapper::callbackEgoMotion, this);
  this->subscriber.push_back(std::move(subscriber_ego_motion));
  ROS_INFO_STREAM("Fusion will listen to IkaEgoMotion topic: ");
  ROS_INFO_STREAM("- " << this->subscribe_ego_motion.c_str());
}

void IkaFusionWrapper::loadParameters(){
  // ROS_INFO("Reading object list sources to fuse.");

  if(!node_handle.getParam("input_topics/object_lists", this->subscribe_topics)){
    ROS_ERROR("Param input_topics/object_lists not found");
  }

  // ROS_INFO("Found %i object list sources to fuse.", int(this->subscribe_topics.size()));

  if(!node_handle.getParam("input_topics/ego_motion", this->subscribe_ego_motion)){
    ROS_ERROR("Param input_topics/ego_motion not found");
  }

  if(!node_handle.getParam("output_topics/object_list_fused/publish", this->publish_object_list_fused)){
    ROS_ERROR("Param sensor_fusion/object_list_fused/publish not found");
  }
  if(!node_handle.getParam("output_topics/object_list_fused/topic", this->publish_topic_object_list_fused)){
    ROS_ERROR("Param sensor_fusion/object_list_fused/topic not found");
  }
  if(!node_handle.getParam("output_topics/object_list_fused/frame_id", this->publish_frame_id_object_list_fused)){
    ROS_ERROR("Param sensor_fusion/output_topics/object_list_fused/frame_id not found");
  }

  // Read config from object_fusion
  XmlRpc::XmlRpcValue fusion_passat_config;
  if(!node_handle.getParam("fusion_passat", fusion_passat_config)){
    ROS_ERROR("fusion_passat config file not found");
  }
  ROS_ASSERT(fusion_passat_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  // Read config from kalman_filter
  XmlRpc::XmlRpcValue kalman_filter_passat_config;
  if(!node_handle.getParam("kalman_filter_passat", kalman_filter_passat_config)){
    ROS_ERROR("kalman_filter_passat config file not found");
  }
  ROS_ASSERT(kalman_filter_passat_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  ROS_INFO("Initiate fusion and Kalman filter.");

  this->initiateFusionConstants(fusion_passat_config);
  this->initiateKalmanFilter(kalman_filter_passat_config);
}

void IkaFusionWrapper::initiateFusionConstants(XmlRpc::XmlRpcValue &fusion_passat_config){
  auto constants = fusion_passat_config["constants"];
  ROS_ASSERT(constants.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  ConfigParams cfg_params;
  cfg_params.existence_probability_loss_rate = float(static_cast<double>(constants["existence_probability_loss_rate"]));

  cfg_params.existence_probability_loss_rate = float(static_cast<double>(constants["existence_probability_loss_rate"]));
  cfg_params.chosen_distance_measure = int(static_cast<int>(constants["chosen_distance_measure"]));
  cfg_params.mahalanobis_threshold = float(static_cast<double>(constants["mahalanobis_threshold"]));
  cfg_params.iou_overlap_threshold = float(static_cast<double>(constants["iou_overlap_threshold"]));
  cfg_params.mahalanobis_global_threshold = float(static_cast<double>(constants["mahalanobis_global_threshold"]));
  cfg_params.existence_probability_output_threshold = float(static_cast<double>(constants["existence_probability_output_threshold"]));
  cfg_params.existence_probability_delete_threshold = float(static_cast<double>(constants["existence_probability_delete_threshold"]));
  cfg_params.measurement_history_threshold_sec = static_cast<double>(constants["measurement_history_threshold_sec"]);

  cfg_params.time_jump_forward_sec = float(static_cast<double>(constants["time_jump"]["forward_threshold_sec"]));
  cfg_params.time_jump_backward_sec = float(static_cast<double>(constants["time_jump"]["backward_threshold_sec"]));

  auto weights = fusion_passat_config["weights"];
  ROS_ASSERT(weights.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  this->loadWeights(weights["sensor_weights_object_existence"], cfg_params.sensor_weights_object_existence);

  // variances
  auto variances = fusion_passat_config["initial_global_variances"];
  this->loadVariances(variances["velocity"], cfg_params.velocity_variances);
  this->loadVariances(variances["acceleration"], cfg_params.acceleration_variances);
  this->loadVariances(variances["other_state_variables"], cfg_params.other_state_variables_variances);
  this->loadVariances(variances["heading"], cfg_params.heading_variances);
  this->loadMahalanobisPenalty(fusion_passat_config["mahalanobis_penalty"], cfg_params.mahalanobis_penalty);

  ika_fusion_library_->initiateFusionConstants(cfg_params);
}

void IkaFusionWrapper::initiateKalmanFilter(XmlRpc::XmlRpcValue &kalman_filter_passat_config){
  Eigen::MatrixXf constant_system_matrix = IkaUtilities::XmlRpcMatrixToEigenMatrixXf(kalman_filter_passat_config["constant_system_matrix"]);
  Eigen::MatrixXf time_variant_system_matrix = IkaUtilities::XmlRpcMatrixToEigenMatrixXf(kalman_filter_passat_config["time_variant_system_matrix"]);
  Eigen::MatrixXf time_variant_process_noise_matrix
      = IkaUtilities::XmlRpcMatrixToEigenMatrixXf(kalman_filter_passat_config["time_variant_process_noise_matrix"]);

  ika_fusion_library_->initiateKalmanFilterConstants(constant_system_matrix,
                                                     time_variant_system_matrix,
                                                     time_variant_process_noise_matrix);
}

void IkaFusionWrapper::loadVariances(XmlRpc::XmlRpcValue &variance_config, std::map<int, int> &variances){
  int object_type_size = 50;  // size of object types (see object_definitions.h), adjusted to high value so no modification in future is needed

  int default_value = static_cast<int>(variance_config["default"]);
  // initialize with default values
  for (int i=0; i <= object_type_size ; ++i) {
    variances.insert(std::make_pair(i, default_value));
  }
  // overwrite default values with those that are specified in config
  for (auto &element : variance_config) {
    if (element.first == "default") continue;
    variances[std::stoi(element.first)] = static_cast<int>(element.second);
  }
}

void IkaFusionWrapper::loadWeights(
    XmlRpc::XmlRpcValue &weights_config,
    std::map<int, float> &sensor_weights)
{
  int number_possible_input_sensors = 50;  // size of input sensors (see object_definitions.h), adjusted to high value so no modification in future is needed

  double default_value = static_cast<double>(weights_config["default"]);
  // initialize with default values
  for (int i=0; i <= number_possible_input_sensors ; ++i) {
    sensor_weights.insert(std::make_pair(i, float(default_value)));
  }

  // overwrite default values with those that are specified in config
  for (auto &element : weights_config) {
    if (element.first == "default") continue;
    float value = static_cast<float>(static_cast<double>(element.second));
    sensor_weights[std::stoi(element.first)] = value;
  }

}

void IkaFusionWrapper::loadMahalanobisPenalty(XmlRpc::XmlRpcValue &config, std::map<std::pair<int, int>, float> &mahalanobis_penalty){
  auto objects_ids = config["id"];
  std::vector<int> objects_types(size_t(objects_ids.size()));

  for (int32_t i = 0; i < objects_ids.size(); ++i){
    objects_types[unsigned(long(i))] = static_cast<int>(objects_ids[i]);
  }

  for (auto &element : config) {
    if (element.first == "id") continue;
    auto list = element.second;
    for (int32_t i = 0; i < list.size(); ++i){
      float value = static_cast<float>(static_cast<double>(list[i]));
      mahalanobis_penalty.insert(
            std::make_pair(std::make_pair(stoi(element.first), objects_types[unsigned(long(i))]), value));
    }
  }
}

/**@}*/
