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
 * @file vehicle_controller_node.h
 */

#include "VehicleController.h"

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace sample_package_cpp {

// Class representing the Vehicle Controller Node in ROS.
class VehicleControllerNode : public rclcpp::Node {
public:
    // Static constants for parameter names and default topic values
    static const std::string kSensorTopicParam;
    static const std::string kActuatorTopicParam;

    static const std::string kDefaultSensorTopic;
    static const std::string kDefaultActuatorTopic;

    // Constructor and Destructor
    VehicleControllerNode();
    ~VehicleControllerNode();

private:
    // Method to declare parameters used in the ROS node
    void declareParameters();

    // Method to load the parameters from the parameter server
    void loadParameters();

    // Setup method initializes publishers, subscribers, and other members
    void setup();

    // Callback method that is invoked upon receiving Lidar data on the subscribed topic
    void callbackLaserSensor(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    // Private member variables
    std::string sensor_topic_;
    std::string actuator_topic_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_actions_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_sensor_data_;
    VehicleController *vehicle_controller = nullptr;
};

// Static member variable initialization
const std::string VehicleControllerNode::kSensorTopicParam{"vehicle.sensor_topic"};
const std::string VehicleControllerNode::kActuatorTopicParam{"vehicle.actuator_topic"};
const std::string VehicleControllerNode::kDefaultSensorTopic{"/vehicle/lidar_measurements"};
const std::string VehicleControllerNode::kDefaultActuatorTopic{"/vehicle/actuator_commands"};

VehicleControllerNode::VehicleControllerNode() : rclcpp::Node("vehicle_controller_node") {
    // Initialize parameters, load them and set up the node components
    declareParameters();
    loadParameters();
    setup();
}

VehicleControllerNode::~VehicleControllerNode() {
    delete vehicle_controller;
}

void VehicleControllerNode::declareParameters() {
    // Declare ROS parameters with their default values
    this->declare_parameter(kSensorTopicParam, kDefaultSensorTopic);
    this->declare_parameter(kActuatorTopicParam, kDefaultActuatorTopic);
}

void VehicleControllerNode::loadParameters() {
    // Load ROS parameters into member variables
    sensor_topic_ = this->get_parameter(kSensorTopicParam).as_string();
    actuator_topic_ = this->get_parameter(kActuatorTopicParam).as_string();
}

void VehicleControllerNode::setup() {
    // Initialize the Vehicle Controller object
    vehicle_controller = new VehicleController;
    
    // Logging info about subscribed and published topics
    RCLCPP_INFO(this->get_logger(), "Vehicle controller subscribes to: %s", sensor_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Vehicle controller publishes to: %s", actuator_topic_.c_str());
    
    // Create a subscriber for the sensor data and bind its callback function
    subscriber_sensor_data_ = this->create_subscription<sensor_msgs::msg::LaserScan>(sensor_topic_, 10, 
        std::bind(&VehicleControllerNode::callbackLaserSensor, this, std::placeholders::_1));

    // Create a publisher to publish actuator commands
    publisher_actions_ = this->create_publisher<geometry_msgs::msg::Twist>(actuator_topic_, 10);
}

void VehicleControllerNode::callbackLaserSensor(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Process the received sensor data and compute actuator commands
    float distances[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    for(size_t i=0; i<5; i++){
        distances[i] = msg->ranges[i];
    }

    // Utilize the vehicle controller to determine actions based on the sensor data
    vehicle_controller->overwriteLidarDistances(distances);
    vehicle_controller->computeTargetValues();

    //Convert local variables to a geometry_msgs::msg::Twist message for publishing.
    geometry_msgs::msg::Twist new_action;
    new_action.angular.z = vehicle_controller->getTargetSteeringAngle();
    new_action.linear.x = vehicle_controller->getTargetVelocity();

    // Publish the computed actuator command
    publisher_actions_->publish(new_action);
}

}  // namespace sample_package_cpp

// Main function to initialize and run the ROS node
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sample_package_cpp::VehicleControllerNode>();
    RCLCPP_INFO(node->get_logger(), "Vehicle controller started.");
    RCLCPP_INFO(node->get_logger(), "Vehicle controller is running...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}