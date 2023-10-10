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
    base_message = this->get_parameter("base_message").as_string();
    topic_name = this->get_parameter("topic").as_string();
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