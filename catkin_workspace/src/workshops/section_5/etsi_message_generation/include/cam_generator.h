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
#include <tf/tf.h>
#include <tf2_ros/buffer_interface.h>

#include <definitions/utility/ika_utilities.h>

#include <definitions/FlatlandVehicleState.h>
#include <definitions/v2x_CAM.h>


namespace etsi_message_generation {

class cam_generator {

  public:
    cam_generator();

  private:
    void wheelSensorCallback(const definitions::FlatlandVehicleState& msg);
    definitions::v2x_CAM generateCAM(const float vel);

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;
    tf2_ros::TransformListener *listener;
    tf2_ros::Buffer *buffer_;

    ros::Publisher pub_cam_;
    ros::Subscriber sub_wheelSensor_;

    definitions::v2x_CAM cam_;

    int parameter_stationID_;
    std::string parameter_cam_topic_;

};


}  // end of namespace etsi_message_generation
