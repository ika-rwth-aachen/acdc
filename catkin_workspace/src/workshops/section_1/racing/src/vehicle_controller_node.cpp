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

#include "VehicleController.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

/* Declare variables outside of functions, such that they can be used
 * both in main, and in the callback function.
 */
ros::Publisher *publisher_actions = nullptr;
ros::Subscriber *subscriber_sensor_data = nullptr;
VehicleController *vehicle_controller = nullptr;

/**
 * @brief Callback function that is automatically triggered when a new Lidar scan is available
 * @param msg A pointer to message object that contains the new Lidar scan
 */
void callbackLaserSensor(const sensor_msgs::LaserScanPtr &msg) {
  // START TASK 2 CODE



  // END TASK 2 CODE

  // Copy argument data to local variable
  float distances[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  for(size_t i=0; i<5;i++){
    distances[i] = msg->ranges[i];
  }

  // Interface calls to the VehicleController instance
  vehicle_controller->overwriteLidarDistances(distances);
  vehicle_controller->computeTargetValues();
  double linear_velocity = vehicle_controller->getTargetVelocity();
  double steering_angle = vehicle_controller->getTargetSteeringAngle();

  // Convert local variables to a geometry_msgs::Twist message for publishing.
  geometry_msgs::Twist new_action;
  geometry_msgs::Vector3 steering;
  geometry_msgs::Vector3 velocity;
  steering.z = steering_angle;
  velocity.x = linear_velocity;
  new_action.linear = velocity;
  new_action.angular = steering;

  // Publish the newly computed actuator command to the topic
  publisher_actions->publish(new_action);
}

int main(int argc, char* argv[]) {
  // Initialize the ROS node
  ros::init(argc, argv, "vehicle_controller");
  ros::NodeHandle node_handle;
  ROS_INFO("Vehicle controller started.");

  // Declare local variables for subscribe and publish topics
  std::string subscribe_topic_sensors;
  std::string publish_topic_actuators;

  // Write publish and subscribe topics from parameter server into local variables
  node_handle.getParam("vehicle/sensor_topic", subscribe_topic_sensors);
  node_handle.getParam("vehicle/actuator_topic", publish_topic_actuators);

  ROS_INFO("Vehicle controller subscribes to: %s", subscribe_topic_sensors.c_str());
  ROS_INFO("Vehicle controller publishes to: %s", publish_topic_actuators.c_str());

  // Initialize / allocate dynamic memory
  vehicle_controller = new VehicleController;
  subscriber_sensor_data = new ros::Subscriber;
  publisher_actions = new ros::Publisher;

  // Connect subscriber and publisher to their respective topics and callback function
  *subscriber_sensor_data = node_handle.subscribe(subscribe_topic_sensors, 10, callbackLaserSensor);
  *publisher_actions = node_handle.advertise<geometry_msgs::Twist>(publish_topic_actuators, 10);

  // Enter a loop to keep the node running while looking for messages on the subscribed topic
  ROS_INFO("Vehicle controller is running...");

  // START TASK 1 CODE


  ros::spin();


  // END TASK 1 CODE

  // Clean dynamic memory
  delete publisher_actions;
  delete subscriber_sensor_data;
  delete vehicle_controller;
  return 0;
}
