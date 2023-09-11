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
 * @author Simon Schaefer
 * @date 11.12.2019
 * @file vehicle_timer_node.h
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;

class VehicleTimerNode : public rclcpp::Node{
public:
    VehicleTimerNode(): Node("vehicle_timer_node"){
        this->declare_parameter<std::string>("vehicle.position_topic", "/odometry/ground_truth");

        // Get subscribe topic from parameter server
        std::string subscribe_topic_position;
        this->get_parameter("vehicle.position_topic", subscribe_topic_position);
        RCLCPP_INFO(this->get_logger(), "Vehicle timer subscribes to: %s", subscribe_topic_position.c_str());

        // Initialize / allocate dynamic memory
        time_buffer = new rclcpp::Time;
        *time_buffer = this->get_clock()->now();

        // Connect subscriber and to the respective topics and callback function
        subscriber_position_data = this->create_subscription<nav_msgs::msg::Odometry>(subscribe_topic_position, 10,
                                                                                           std::bind(&VehicleTimerNode::callbackPosition, this, _1));
    }

    ~VehicleTimerNode(){
        delete time_buffer;
    }


private:
    /**
 * @brief Function that maps a position to a map quadrant.
 */
    void computeQuadrants(const double& x, const double& y) {
        if (x > 0 && y < 0 ) {
            quadrants[0] = true;
        } else if (x < 0 && y < 0 && quadrants[0]) {
            quadrants[1] = true;
        } else if (x < 0 && y > 0 && quadrants[1]) {
            quadrants[2] = true;
        } else if (x > 0 && y > 0 && quadrants[2]) {
            quadrants[3] = true;
        }
    }

    /**
 * @brief Function that checks if the vehicle is passing the line in this time step.
 */
    bool justPassedFinishLine(const double& x, const double& y) {
        return quadrants[3] && x > 0 && y < 0;
    }

    /**
 * @brief Print time on the terminal if lap is completed.
 */
    void printInfo() {
        rclcpp::Duration diff = this->get_clock()->now() - *time_buffer;

        if (firstFinishLinePassing) {
            RCLCPP_INFO(this->get_logger(), "Lap time measurement started...");
            firstFinishLinePassing = false;
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Lap time: %fs", diff.seconds());
        }
    }

    /**
 * @brief Function that resets the data.
 */
    void resetData() {
        for (bool & quadrant : quadrants) {
            quadrant = false;
        }
        *time_buffer = this->get_clock()->now();
    }

    /**
 * @brief Callback for the incomming positon.
 */
    void callbackPosition(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        computeQuadrants(x,y);

        if (justPassedFinishLine(x,y)) {
            printInfo();
            resetData();
        }
    }

    // Flags to check there the vehicle is in the moment
    bool quadrants[4] = {false, false, false, true}; // start in quadrants[3]
    // Flag to enable a flying start
    bool firstFinishLinePassing = true;

    // Subscriber to odometry data
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_position_data;

    // Buffer to store the current time
    rclcpp::Time *time_buffer = nullptr;

};

int main(int argc, char *argv[]) {
    // Initialize the ROS node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VehicleTimerNode>();
    RCLCPP_INFO(node->get_logger(), "Vehicle timer started.");

    // Enter a loop to keep the node running while looking for messages on the subscribed topic
    RCLCPP_INFO(node->get_logger(), "Vehicle timer is running...");
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
