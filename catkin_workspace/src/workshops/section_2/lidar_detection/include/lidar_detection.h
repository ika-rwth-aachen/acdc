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

#include <ros/package.h>
#include <nodelet/NodeletUnload.h>
#include <nodelet/nodelet.h>

#include <tensorflow_cpp/model.h>

#include <definitions/IkaObjectList.h>
#include <definitions/utility/object_definitions.h>

#include <definitions.h>
#include <detector.h>
#include <list_creator.h>
#include <pillar_utils.h>
#include <pcl/point_types.h>

#include <chrono>
typedef std::chrono::high_resolution_clock Clock;

namespace lidar_detection
{

  /**
   * @brief
   *
   */
  class LidarDetection : public nodelet::Nodelet
  {

  public:
    /**
     * @brief This is the destructor
     *
     */
    ~LidarDetection();

    /**
     * @brief Mandatory nodelet function. Gets called by the nodelet manager upon
     * start up.
     *
     */
    virtual void onInit();

  private:
    // -- Private Member Functions -- //

    /**
     * @brief This function defines the message output format and creates the corresponding list creator
     *
     */
    void setupFormat();

    /**
     * @brief This function loads the model and sets up all tensors
     *
     */
    void setupModel();

    /**
     * @brief This function subscribes to the topics
     *
     */
    void setupTopics(ros::NodeHandle &nh);

    /**
     * @brief Load parameters from param file and slikaf config
     *
     */
    void loadParameters(ros::NodeHandle &nh);

    /**
     * @brief Callback function for lidar detections of all lidar sensors.
     *
     * @param detection_list Contains a list with all detections made by the
     * sensor.
     */
    void predict(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg);

    // tensorflow model
    tensorflow_cpp::Model model_;
#ifdef MODE_PP
    const std::string SAVED_MODEL_INPUT_NAME_PILLARS = "pillars/input";
    const std::string SAVED_MODEL_INPUT_NAME_INDICES = "pillars/indices";
    const std::string SAVED_MODEL_OUTPUT_NAME_OCCUPANCY = "occupancy/conv2d";
    const std::string SAVED_MODEL_OUTPUT_NAME_LOCATION = "loc/reshape";
    const std::string SAVED_MODEL_OUTPUT_NAME_SIZE = "size/reshape";
    const std::string SAVED_MODEL_OUTPUT_NAME_ANGLE = "angle/conv2d";
    const std::string SAVED_MODEL_OUTPUT_NAME_HEADING = "heading/conv2d";
    const std::string SAVED_MODEL_OUTPUT_NAME_CLASS = "clf/reshape";
    const std::string FROZEN_GRAPH_INPUT_NAME_PILLARS = "input";
    const std::string FROZEN_GRAPH_INPUT_NAME_INDICES = "input_1";
    const std::string FROZEN_GRAPH_OUTPUT_NAME_OCCUPANCY = "model/occupancy/conv2d/Sigmoid";

    // tensorflow tensors
    tensorflow::Tensor pillars_;
    tensorflow::Tensor indices_;
#endif
    // detector for processing bounding boxes
    Detector detector_;

    ikaListCreator *ika_list_creator_;

    // subscriber and publisher
    ros::Subscriber subscriber_;
    ros::Publisher ika_publisher_;
    ros::Publisher pcl_publisher_;

    // transform listener and transform buffer
    tf2_ros::Buffer tf_buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // structs for params and config
    Params params_;
    SlikafConfig config_;
  };

} // namespace lidar_detection
