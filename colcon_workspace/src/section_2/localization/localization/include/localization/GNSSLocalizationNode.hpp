#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>


namespace localization
{

class GNSSLocalizationNode : public rclcpp::Node
{

public:
  explicit GNSSLocalizationNode();

protected:
  // constants
  static const std::string kInputTopicOdometry;
  static const std::string kInputTopicGNSS;
  static const std::string kOutputTopicGNSSPoint;
  static const std::string kOutputTopicGNSSPose;
  static const std::string kOutputTopicPredictedPose;

private:
  void setup();

  void odometryCallback(nav_msgs::msg::Odometry::UniquePtr msg);
  void gnssCallback(sensor_msgs::msg::NavSatFix::UniquePtr msg);

  bool projectToUTM(const double& latitude, const double& longitude, geometry_msgs::msg::PointStamped& utm_point);
  bool transformPoint(const geometry_msgs::msg::PointStamped& input_point, geometry_msgs::msg::PointStamped& output_point, const std::string& output_frame);
  void estimateGNSSHeading(const geometry_msgs::msg::PointStamped& current_point, const geometry_msgs::msg::PointStamped& last_point, geometry_msgs::msg::PoseStamped& output_pose);
  bool getIncrementalMovement(const nav_msgs::msg::Odometry& current_odometry, const nav_msgs::msg::Odometry& previous_odometry, geometry_msgs::msg::Vector3& delta_translation, tf2::Quaternion& delta_rotation);
  void setInitialPose(geometry_msgs::msg::PoseStamped& initial_pose);
  void posePrediction(geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Vector3& delta_translation, const tf2::Quaternion& delta_rotation);
  void getYawFromQuaternion(double& yaw, const tf2::Quaternion& quaternion);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_gnss_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_gnss_point_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_gnss_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_predicted_pose_;

  std::shared_ptr<geometry_msgs::msg::PointStamped> last_gnss_map_point_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> gnss_map_pose_;
  bool new_gnss_pose_ = false;

  std::shared_ptr<nav_msgs::msg::Odometry> last_odometry_;

  std::shared_ptr<geometry_msgs::msg::PoseStamped> predicted_map_pose_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace localization