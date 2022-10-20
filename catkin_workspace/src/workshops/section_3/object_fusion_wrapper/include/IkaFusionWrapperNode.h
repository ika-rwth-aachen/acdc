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

/**
 * @defgroup OBJECT_FUSION_WRAPPER Ros processor for the sensor fusion.
 * @brief Ros package that contains all code to process the sensor fusion.
 * @addtogroup OBJECT_FUSION_WRAPPER
 */
/**@{*/
/**
 * @file IkaFusionWrapperNode.cpp
 * @author Michael Hoss, Simon Schaefer
 * @brief  ROS-wrapper for usage of the object_fusion package as a ROS node.
 * @details Make use of the IkaFusion to fuse IkaObjects.
 */

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <ros/ros.h>

#include <definitions/IkaObjectList.h>
#include <definitions/IkaObject.h>
#include <definitions/utility/ika_utilities.h>

#include <ikaFusLib.h>

class IkaFusionWrapper{
public:
  IkaFusionWrapper(const ros::NodeHandle &node_handle);
  ~IkaFusionWrapper();

  void loadParameters();

  void initiatePublisherAndSubscriber();

private:

  definitions::IkaEgoMotion ego_motion_memory_;

  /**
 * @brief Callback function for input messages of type IkaObjectList.
 */
  void callbackObjectList(const definitions::IkaObjectListConstPtr &msg);

  /**
 * @brief Callback function for input messages of Type IkaEgoMotion.
 */
  void callbackEgoMotion(const definitions::IkaEgoMotionConstPtr &msg);

  void loadVariances(XmlRpc::XmlRpcValue &variance_config, std::map<int, int> &variances);

  void initiateFusionConstants(XmlRpc::XmlRpcValue &fusion_passat_config);

  void loadWeights(XmlRpc::XmlRpcValue &weights_config, std::map<int, float> &sensor_weights);

  void loadMahalanobisPenalty(XmlRpc::XmlRpcValue &config, std::map<std::pair<int, int>, float> &mahalanobis_penalty);

  void initiateKalmanFilter(XmlRpc::XmlRpcValue &kalman_filter_passat_config);

  ros::Publisher *object_list_fused_publisher_ = nullptr;

  ikaFusLib *ika_fusion_library_ = nullptr; ///< @brief Instance of the ika fusion library

  ros::NodeHandle node_handle;

  std::vector<std::string> subscribe_topics;
  std::string subscribe_ego_motion;
  std::vector<ros::Subscriber> subscriber;

  std::string publish_topic_object_list_fused;
  std::string publish_frame_id_object_list_fused;
  bool publish_object_list_fused = false;
};

/**
* @brief ROS main function for ROS initialization and parameter parsing from launch file.
* @details Has to be executed with the given launch file!
*/
int main(int argc, char *argv[]);

/**@}*/
