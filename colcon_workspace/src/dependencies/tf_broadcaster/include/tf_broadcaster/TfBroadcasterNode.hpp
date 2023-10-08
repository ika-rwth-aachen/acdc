#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace localization
{
class TfBroadcasterNode : public rclcpp::Node
{
public:
  explicit TfBroadcasterNode();

protected:
  // constants
  static const std::string kInputTopic;
  static const double kOriginCarlaMapLat;
  static const double kOriginCarlaMapLon;

private:
  void setup();
  void poseCallback(nav_msgs::msg::Odometry::UniquePtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

}  // namespace localization