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

#include <lidar_detection.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(lidar_detection::LidarDetection,
                       nodelet::Nodelet)

namespace lidar_detection
{

  void LidarDetection::loadParameters(ros::NodeHandle &nh)
  {
    params_.initialize(nh);
    config_.initialize(nh);
  }

  void LidarDetection::predict(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg)
  {
    std::cout << "PRedict"<< std::endl;
    auto start_predict = Clock::now();

    LidarPointCloud pcl;
    sensor_msgs::PointCloud2 pcl_msg_trans;

    // check if tranformation is required
    if (!params_.inference_frame.empty() &&
        pcl_msg->header.frame_id != params_.inference_frame)
    {
      // get transformation
      geometry_msgs::TransformStamped trans;
      try
      {
        trans = tf_buffer_.lookupTransform(params_.inference_frame, pcl_msg->header.frame_id, ros::Time(0));
      }
      catch (tf2::LookupException &e)
      {
        ROS_ERROR("Lidar Detection Error: pcl transformation %s", e.what());
      }

      // transform point cloud
      tf2::doTransform(*pcl_msg, pcl_msg_trans, trans);
    }
    else
    {
      pcl_msg_trans = *pcl_msg;
    }

    // publish transformed point cloud
    if (params_.publish_pcl)
    {
      pcl_publisher_.publish(pcl_msg_trans);
    }

    // get internal pcl data struct
    pcl::fromROSMsg(pcl_msg_trans, pcl);

    // create pillars
    if (pcl.size() > 0)
    {
#ifdef MODE_PP
      createPillars(pcl, config_, pillars_, indices_);
#endif
    }
    else
    {
      ROS_WARN("Lidar detection: point cloud is empty.");
    }

    // run model inference
    std::vector<tensorflow::Tensor> output_tensors;
#ifdef MODE_PP
    if (model_.isSavedModel())
    {
      std::vector<std::string> model_output_names = {
          SAVED_MODEL_OUTPUT_NAME_OCCUPANCY,
          SAVED_MODEL_OUTPUT_NAME_LOCATION,
          SAVED_MODEL_OUTPUT_NAME_SIZE,
          SAVED_MODEL_OUTPUT_NAME_ANGLE,
          SAVED_MODEL_OUTPUT_NAME_HEADING,
          SAVED_MODEL_OUTPUT_NAME_CLASS};
      auto outputs = model_({{SAVED_MODEL_INPUT_NAME_PILLARS, pillars_},
                             {SAVED_MODEL_INPUT_NAME_INDICES, indices_}},
                            model_output_names);
      for (const auto &output_name : model_output_names)
        output_tensors.push_back(outputs[output_name]);
    }
    else
    {
      output_tensors = model_({pillars_, indices_});
    }
#endif
    if (output_tensors.empty())
      return;

    // create boxes from output tensors
    tensorflow::Tensor indices;
    std::vector<BoundingBoxCenter> center_boxes;
    detector_.createBoxes(output_tensors, indices, center_boxes);

    // stop prediction timer
    auto stop_predict = Clock::now();

    // object list creation
    definitions::IkaObjectList object_list;
    ika_list_creator_->createObjectList(pcl_msg_trans.header, indices, center_boxes, object_list);
    ika_publisher_.publish(object_list);
    std::cout << object_list.objects.size() << " ika objects detected in " << std::chrono::duration_cast<std::chrono::milliseconds>(stop_predict - start_predict).count() << " ms" << std::endl;

// reset tensors to zero after prediction to have them ready for the next tick.
// this has the advantage of excluding the resetting from detection latency.
#ifdef MODE_PP
    pillars_.tensor<float, 4>().setZero();
    indices_.tensor<int, 3>().setZero();
#endif
  }

  void LidarDetection::setupTopics(ros::NodeHandle &nh)
  {
    // transform listener subscription
    tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_, nh));

    // point cloud subscription
    subscriber_ = nh.subscribe(params_.input_topic, 1, &LidarDetection::predict, this);

    // point cloud advertisement
    pcl_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("output_point_cloud", 1);

    // set config to detector
    detector_.initialize(config_, params_);

    // init and register publisher
    ika_list_creator_->initialize(params_, tf_buffer_);
    ika_publisher_ = nh.advertise<definitions::IkaObjectList>("object_list_ika", 1);
  }

  void LidarDetection::setupFormat()
  {
    ika_list_creator_ = new ikaListCreator();
  }

  void LidarDetection::setupModel()
  {
    // build up model_paths
    std::string package_path = ros::package::getPath("lidar_detection");
    std::string saved_model_path = package_path + "/" + params_.model_name + "/SavedModels/" + params_.model_checkpoint;
    std::string frozen_graph_path = package_path + "/" + params_.model_name + "/FrozenGraphs/" + params_.model_checkpoint + "/" + params_.model_checkpoint + ".pb";
    std::string model_path;

    // check existence of saved model or frozen graph
    struct stat s;
    if (stat(saved_model_path.c_str(), &s) == 0)
    {
      model_path = saved_model_path;
      ROS_INFO("Found saved model!");
    }
    else if (stat(frozen_graph_path.c_str(), &s) == 0)
    {
      model_path = frozen_graph_path;
      ROS_INFO("Found frozen graph!");
    }
    else
      ROS_ERROR("Lidar Detection Error: model path does not exist");

    // load model
    model_.loadModel(model_path);

    // log model info
    std::cout << model_.getInfoString() << std::endl;

#ifdef MODE_PP
    // tensor allocation
    pillars_ = tensorflow::Tensor(
        tensorflow::DT_FLOAT,
        tensorflow::TensorShape({1, config_.max_pillars,
                                 config_.max_points_per_pillar,
                                 config_.n_features}));
    indices_ = tensorflow::Tensor(
        tensorflow::DT_INT32,
        tensorflow::TensorShape({1, config_.max_pillars, 3}));

    // tensor initialization
    pillars_.tensor<float, 4>().setZero();
    indices_.tensor<int, 3>().setZero();
#endif
  }

  void LidarDetection::onInit()
  {
    // get node handles
    ros::NodeHandle &nh = getPrivateNodeHandle();

    // load parameters from parameter server
    ROS_INFO("Lidar Detection Info: load parameters");
    loadParameters(nh);

    // define format
    ROS_INFO("Lidar Detection Info: setup format");
    setupFormat();

    // load model
    ROS_INFO("Lidar Detection Info: setup model");
    setupModel();

    // subscribe and advertise relevant topics.
    ROS_INFO("Lidar Detection Info: setup topics");
    setupTopics(nh);
  }

  LidarDetection::~LidarDetection()
  {
    delete ika_list_creator_;
  }

} // namespace lidar_detection
