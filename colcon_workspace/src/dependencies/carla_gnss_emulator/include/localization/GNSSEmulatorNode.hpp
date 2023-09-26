#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>


namespace localization
{

class GNSSEmulatorNode : public rclcpp::Node
{

public:
  explicit GNSSEmulatorNode();

protected:
  // constants
  static const std::string kInputTopicOdometry;
  static const std::string kOutputTopicGNSS;

  // parameter names
  static const std::string kPublishPeriodParam;
  static const std::string kUTMZoneParam;
  static const std::string kNorthernHemisphereParam;

private:
  void setup();

  void odometryCallback(nav_msgs::msg::Odometry::UniquePtr msg);

  bool projectToWGS84(const geometry_msgs::msg::Point& utm_point, double& latitude, double& longitude);
  bool transformPoint(const geometry_msgs::msg::PointStamped& input_point, geometry_msgs::msg::PointStamped& output_point, const std::string& output_frame);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_gnss_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  double period_;
  int zone_;
  bool northp_;
  rclcpp::Time last_pub_;
};

}  // namespace localization