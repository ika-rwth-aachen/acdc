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

// Including the necessary standard library header for shared pointers
#include <memory>

// ROS 2 C++ client library
#include "rclcpp/rclcpp.hpp"

// The message type used for subscribing
#include "std_msgs/msg/string.hpp"

// For using the placeholder in the std::bind function
using std::placeholders::_1;

/* 
 * MinimalSubscriber class inherits from rclcpp::Node.
 * It's a simple example of how to create a ROS 2 subscriber node.
 */
class MinimalSubscriber : public rclcpp::Node
{
public:
    // Constructor
    MinimalSubscriber()
    : Node("minimal_subscriber")  // Initialize the node with the name "minimal_subscriber"
    {

    // Decleare and Load the parameters
    declareParameters();
    loadParameters();

    // Create a subscription to the "topic" with a queue size of 10.
    // The std::bind function is used to bind the topic_callback function 
    // to this subscription, with the placeholder _1 being replaced by the received message.
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        topic_name, 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    }

private:

    void declareParameters()
    {
        // Declare the parameters with a default value
        this->declare_parameter<std::string>("topic", "default_topic");
    }

    void loadParameters()
    {
        // Load the parameter values
        this->get_parameter("topic", topic_name);
    }

    // Callback function for when a message is received on the subscribed topic
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
            // Log the received message for the user to see
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    
    // Private member variable to store the subscription object
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    std::string topic_name; // Topic name to publish to
};

// Main function
int main(int argc, char * argv[])
{
    // Initialize the ROS 2 communication
    rclcpp::init(argc, argv);
    
    // Create an instance of MinimalSubscriber and keep it responsive to callbacks
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    
    // Clean up and shut down the ROS 2 communication
    rclcpp::shutdown();
    
    return 0;
}
