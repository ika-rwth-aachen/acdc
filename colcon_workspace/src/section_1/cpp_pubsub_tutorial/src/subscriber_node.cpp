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
        topic_name = this->get_parameter("topic").as_string();
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
