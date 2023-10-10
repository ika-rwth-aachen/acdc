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
 * @author Simon Schaefer, Michael Hoss
 * @file vehicle_controller_node.h
 */

#include "VehicleController.h"

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class VehicleControllerNode : public rclcpp::Node{
public:
    VehicleControllerNode(): Node("vehicle_controller_node"){
        
        // Declare and load the parameters
        declareParameters();
        loadParameters();
        RCLCPP_INFO(this->get_logger(), "Vehicle controller subscribes to: %s", subscribe_topic_sensors.c_str());
        RCLCPP_INFO(this->get_logger(), "Vehicle controller publishes to: %s", publish_topic_actuators.c_str());

        // Initialize / allocate dynamic memory
        vehicle_controller = new VehicleController;

        // Connect subscriber and publisher to their respective topics and callback function
        subscriber_sensor_data_ = this->create_subscription<sensor_msgs::msg::LaserScan>(subscribe_topic_sensors, 10,
                                                                                         std::bind(&VehicleControllerNode::callbackLaserSensor, this, _1));

        publisher_actions_ = this->create_publisher<geometry_msgs::msg::Twist>(publish_topic_actuators, 10);
    }

    ~VehicleControllerNode(){
        // Clean dynamic memory
        delete vehicle_controller;
    }

private:
    /**
 * @brief Callback function that is automatically triggered when a new Lidar scan is available
 * @param msg A pointer to message object that contains the new Lidar scan
 */

    void callbackLaserSensor(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Copy argument data to local variable
        float distances[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        for(size_t i=0; i<5;i++){
            distances[i] = msg->ranges[i];
        }

        // Interface calls to the VehicleController instance
        vehicle_controller->overwriteLidarDistances(distances);
        vehicle_controller->computeTargetValues();

        // Convert local variables to a geometry_msgs::msg::Twist message for publishing.
        geometry_msgs::msg::Twist new_action;
        new_action.angular.z = vehicle_controller->getTargetSteeringAngle();
        new_action.linear.x = vehicle_controller->getTargetVelocity();

        // Publish the newly computed actuator command to the topic
        publisher_actions_->publish(new_action);
    }

    void declareParameters()
    {
        // Declare the parameters with a default value
        this->declare_parameter<std::string>("vehicle.sensor_topic", "/vehicle/lidar_measurements");
        this->declare_parameter<std::string>("vehicle.actuator_topic", "/vehicle/actuator_commands");
    }

    void loadParameters()
    {
        // Write publish and subscribe topics from parameter server into local variables
        subscribe_topic_sensors = this->get_parameter("vehicle.sensor_topic").as_string( );
        publish_topic_actuators = this->get_parameter("vehicle.actuator_topic").as_string();

    }

    /* Declare variables outside of functions, such that they can be used
    * both in main, and in the callback function.
    */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_actions_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_sensor_data_;
    VehicleController *vehicle_controller = nullptr;

    // Declare variables for subscribe and publish topics
     std::string subscribe_topic_sensors;
     std::string publish_topic_actuators;

};

int main(int argc, char* argv[]) {
    // Initialize the ROS node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleControllerNode>();
    RCLCPP_INFO(node->get_logger(), "Vehicle controller started.");

    // Enter a loop to keep the node running while looking for messages on the subscribed topic
    RCLCPP_INFO(node->get_logger(), "Vehicle controller is running...");
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}