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

#include <cam_generator.h>


namespace etsi_message_generation {


cam_generator::cam_generator() : node_handle_(), private_node_handle_("~") {
  ROS_INFO("cam_generator starting...");

  // generate random stationID (uint32)
  srand((unsigned) time(NULL)); 
  parameter_stationID_ = rand() % 65535 + 1; //random uint16
  private_node_handle_.setParam("stationID", parameter_stationID_);
  ROS_WARN("stationID is random set to %d.", parameter_stationID_);

  // load parameters
  if(!private_node_handle_.param<std::string>("CAM_Topic_Name", parameter_cam_topic_, "/CAM")) ROS_WARN("CAM_Topic_Name not set, defaulting to %s.", parameter_cam_topic_.c_str());
  ROS_INFO("Config: parameter_stationID_ = %d, parameter_cam_topic_ = %s", parameter_stationID_, parameter_cam_topic_.c_str());

  // init tf2 buffer and listener
  buffer_ = new tf2_ros::Buffer();
  listener = new tf2_ros::TransformListener(*buffer_);

  // setup publisher and subscriber
  pub_cam_ = private_node_handle_.advertise<definitions::v2x_CAM>(parameter_cam_topic_, 1);
  sub_wheelSensor_ = private_node_handle_.subscribe("/vehicle/wheel_sensor", 1, &cam_generator::wheelSensorCallback, this);

  ros::spin();
}

void cam_generator::wheelSensorCallback(const definitions::FlatlandVehicleState& msg) {

  cam_ = generateCAM(msg.velocity);
  pub_cam_.publish(cam_);
}

definitions::v2x_CAM cam_generator::generateCAM(const float vel) {

  // get pose of vehicle in flatland simulation via tf
  geometry_msgs::TransformStamped transform;
  try{
     transform = buffer_->lookupTransform("map", "vehicle_body", ros::Time(0), ros::Duration(0));
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }

  tf::Quaternion q(transform.transform.rotation.x,
                  transform.transform.rotation.y,
                  transform.transform.rotation.z,
                  transform.transform.rotation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // -----------FILL CAM--------------------------

  definitions::v2x_CAM v2x_cam;
  v2x_cam.header_protocolVersion = 2;
  v2x_cam.header_messageID = 2; // CAM
  v2x_cam.header_stationID = parameter_stationID_;
  v2x_cam.cam_generationDeltaTime = ros::WallTime::now().toSec();
  
  definitions::v2x_CAM_basic_container basic_container;
  basic_container.stationType = 5; // StationType_passengerCar
  basic_container.referencePosition_latitude = transform.transform.translation.y;   // map-frame
  basic_container.referencePosition_longitude = transform.transform.translation.x;  // map-frame
  v2x_cam.basic_container = basic_container;

  definitions::v2x_CAM_high_freq_container high_freq_container;
  high_freq_container.heading_headingValue = yaw;
  high_freq_container.speed_speedValue = vel; // m/s
  high_freq_container.vehicleLength_vehicleLengthValue = 5; // dummy
  high_freq_container.vehicleWidth = 2;                     // dummy
  v2x_cam.high_freq_container = high_freq_container;

  return v2x_cam;  
}


}  // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cam_generator");

  etsi_message_generation::cam_generator node;

  return 0;
}

