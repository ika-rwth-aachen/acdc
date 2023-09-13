// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Necessary standard library headers
#include <chrono>
#include <memory>

// ROS2 client library for C++
#include "rclcpp/rclcpp.hpp"

// Message type used for publishing
#include "std_msgs/msg/string.hpp"

// Using directive for the chrono literals (like 500ms)
using namespace std::chrono_literals;

/* 
 * MinimalPublisher class inherits from rclcpp::Node.
 * It's a simple example of how to create a ROS 2 publisher node.
 */
class MinimalPublisher : public rclcpp::Node
{
public:
  // Constructor
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)  // Initialize the node with the name "minimal_publisher" and set count_ to 0
  {
    // Decleare and Load the parameters
    declareParameters();
    loadParameters();

    // Create a publisher with the topic name "topic", using String messages, and a queue size of 10
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
    
    // Create a timer that fires every 500ms and binds it to the timer_callback function
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:

  void declareParameters()
  {
    // Declare the parameters with a default value
    this->declare_parameter<std::string>("base_message", "default_message");
    this->declare_parameter<std::string>("topic", "default_topic");
  }

  void loadParameters()
  {
    // Load the parameter values
    this->get_parameter("base_message", base_message);
    this->get_parameter("topic", topic_name);
  }

  // This function is called every 500ms by the timer
  void timer_callback()
  {
    // Create a new String message
    auto message = std_msgs::msg::String();
    
    // Set the message data to be the base message + the current count
    message.data = base_message + " " + std::to_string(count_++);
    
    // Log the published message for the user to see
    RCLCPP_INFO(this->get_logger(), "Publishing to: '%s', Message: '%s'", publisher_->get_topic_name(), message.data.c_str());

    // Publish the message
    publisher_->publish(message);
  }
  
  // Private member variables
  rclcpp::TimerBase::SharedPtr timer_;  // Timer triggering the publishing
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // Publisher object
  size_t count_;  // Counter for the messages
  std::string base_message; // Base message to be published
  std::string topic_name; // Topic name to publish to
};

// Main function
int main(int argc, char * argv[])
{
  // Initialize the ROS 2 communication
  rclcpp::init(argc, argv);

  // Create an instance of MinimalPublisher and keep it responsive to callbacks
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  
  // Clean up and shut down the ROS 2 communication
  rclcpp::shutdown();
  
  return 0;
}