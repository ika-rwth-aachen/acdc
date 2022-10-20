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

#include <definitions.h>

namespace lidar_detection
{

  void Params::initialize(ros::NodeHandle &nh)
  {
    if (!nh.getParam("input_topic", input_topic))
    {
      ROS_ERROR("Parameter Error: 'input_topic' is missing");
    }

    if (!nh.getParam("model_name", model_name))
    {
      ROS_ERROR("Parameter Error: 'model_name' is missing");
    }
    if (!nh.getParam("model_checkpoint", model_checkpoint))
    {
      ROS_ERROR("Parameter Error: 'model_checkpoint' is missing");
    }

    // read optional parameters
    nh.param<std::string>("static_params/inference_frame", inference_frame, "");
    nh.param<std::string>("static_params/output_frame", output_frame, "");

    nh.param<bool>("static_params/publish_pcl", publish_pcl, false);
    nh.param<float>("static_params/max_length", max_length, 5);
    nh.param<float>("static_params/max_width", max_width, 5);
    nh.param<float>("static_params/max_height", max_height, 5);

    // read variance
    XmlRpc::XmlRpcValue variance_tmp;
    if (!nh.getParam("static_params/variance", variance_tmp))
    {
      ROS_WARN("Parameter Warning: 'variance' is missing");
    }

    try
    {
      for (int i = 0; i < variance_tmp.size(); i++)
      {
        double f = static_cast<double>(variance_tmp[i]);
        variance.push_back(f);
      }
    }
    catch (...)
    {
      ROS_WARN("Parameter Error: Check that all entries in the 'variance' array are double values. (e.g. convert 1 to 1.0). Skip 'variance'");
    }
  }

  void SlikafConfig::initialize(ros::NodeHandle &nh)
  {
    nh.param<int>("slikaf/intensity_threshold",
                  intensity_threshold, 10000);

#ifdef MODE_PP

    // read grid parameters
    nh.param<float>("slikaf/x_min", x_min,
                    -40.96f);
    nh.param<float>("slikaf/x_max", x_max,
                    40.96f);
    nh.param<float>("slikaf/delta_x", delta_x,
                    0.16f);

    nh.param<float>("slikaf/y_min", y_min,
                    -40.96f);
    nh.param<float>("slikaf/y_max", y_max,
                    40.96f);
    nh.param<float>("slikaf/delta_y", delta_y,
                    0.16f);

    nh.param<float>("slikaf/z_min", z_min,
                    -3.0f);
    nh.param<float>("slikaf/z_max", z_max,
                    1.0f);

    // read network parameters
    nh.param<int>(
        "slikaf/max_points_per_pillar",
        max_points_per_pillar, 100);
    nh.param<int>("slikaf/max_pillars",
                  max_pillars, 10000);
    nh.param<int>("slikaf/n_features",
                  n_features, 7);
    nh.param<int>("slikaf/n_channels",
                  n_channels, 64);
    nh.param<float>("slikaf/downscaling",
                    downscaling, 2.0);

    nh.param<float>("slikaf/nms_max_objects",
                    nms_max_objects, 50);
    nh.param<float>("slikaf/nms_iou_thresh",
                    nms_iou_thresh, 0.1);

    // get class_names
    XmlRpc::XmlRpcValue names;
    if (!nh.getParam("slikaf/class_names", names))
    {
      ROS_ERROR("Slikaf Config Error: 'class_names' not found");
    }

    for (int i = 0; i < names.size(); i++)
    {
      class_names.push_back(static_cast<std::string>(names[i]));
    }

    // get score_thresh
    XmlRpc::XmlRpcValue scores;
    if (!nh.getParam("slikaf/score_thresh", scores))
    {
      ROS_ERROR("Slikaf Config Error: 'score_thresh' not found");
    }

    for (int i = 0; i < scores.size(); i++)
    {
      double score = static_cast<double>(scores[i]);
      if (score < minimum_thresh)
        minimum_thresh = score;
      score_thresh.push_back(score);
    }

    // get anchors
    XmlRpc::XmlRpcValue anchors;
    if (!nh.param("slikaf/anchors", anchors, anchors))
    {
      ROS_ERROR("Slikaf Config Error: 'anchors' not found");
    }

    // check if multiplier of 5
    if (anchors.size() % 5 != 0)
    {
      ROS_ERROR("Slikaf Config Error: 'anchors' has to be a multiplier of 5");
    }

    try
    {
      for (int i = 0; i < int(anchors.size() / 5); i++)
      {
        Anchor anchor;
        anchor.length = static_cast<double>(anchors[i * 5 + 0]);
        anchor.width = static_cast<double>(anchors[i * 5 + 1]);
        anchor.height = static_cast<double>(anchors[i * 5 + 2]);
        anchor.z_center = static_cast<double>(anchors[i * 5 + 3]);
        anchor.orientation = static_cast<double>(anchors[i * 5 + 4]);

        anchor_boxes.push_back(anchor);
        anchor_diagonals.emplace_back(std::sqrt(std::pow(anchor.length, 2) + std::pow(anchor.width, 2)));
      }
    }
    catch (...)
    {
      ROS_ERROR("Slikaf Config Error: Check that all entries in the 'anchors' array are double values. (e.g. convert 1 to 1.0)");
    }

    // check dimensions
    if (score_thresh.size() != class_names.size())
    {
      ROS_ERROR("Slikaf Config Error: 'class_names' and 'score_thresh' has to have the same size");
    };
#endif
  }

} // namespace lidar_detection