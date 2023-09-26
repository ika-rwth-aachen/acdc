#include <tf_broadcaster/TfBroadcasterNode.hpp>

#include <GeographicLib/UTMUPS.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

/**
 * @brief Namespace for tf broadcaster node
 *
 */
namespace localization
{

// constants
const std::string TfBroadcasterNode::kInputTopic{"/ground_truth/pose"};
const double TfBroadcasterNode::kOriginCarlaMapLat{0.0};
const double TfBroadcasterNode::kOriginCarlaMapLon{0.0};


/**
 * @brief Creates the node inheriting from the Node class.
 *
 */
TfBroadcasterNode::TfBroadcasterNode() : rclcpp::Node("tf_broadcaster_node")
{
  setup();
}

/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void TfBroadcasterNode::setup()
{
  // create a transform broadcaster, buffer, and listener
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // create a subscriber for handling incoming messages
  subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    kInputTopic, 1, std::bind(&TfBroadcasterNode::poseCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_->get_topic_name());

  // create and broadcast an static transform utm --> carla_map
  static tf2_ros::StaticTransformBroadcaster static_tf_broadcaster(this);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now();
  tf.header.frame_id = "utm";
  tf.child_frame_id = "carla_map";

  // get utm position of carla_map origin
  double utm_x, utm_y;
  int zone;
  bool northp;
  GeographicLib::UTMUPS::Forward(kOriginCarlaMapLat, kOriginCarlaMapLon, zone, northp, utm_x, utm_y);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "UTM Origin of 'carla_map'-Frame x: " << utm_x << " y: " << utm_y);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "UTM Zone: " << zone << " Is northern hemisphere: " << northp);
  tf.transform.translation.x = utm_x;
  tf.transform.translation.y = utm_y;
  tf.transform.translation.z = 0.0;

  tf2::Quaternion q;
  // no rotational offset between both frames
  q.setRPY(0, 0, 0);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  // send the transform
  static_tf_broadcaster.sendTransform(tf);

}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input
 */
void TfBroadcasterNode::poseCallback(nav_msgs::msg::Odometry::UniquePtr msg)
{
  // create and broadcast a dynamic transform based on the incoming odometry msg
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = msg->header.stamp;
  tf.header.frame_id = msg->header.frame_id;
  tf.child_frame_id = msg->child_frame_id;
  tf.transform.translation.x = msg->pose.pose.position.x;
  tf.transform.translation.y = msg->pose.pose.position.y;
  tf.transform.translation.z = msg->pose.pose.position.z;
  tf.transform.rotation = msg->pose.pose.orientation;
  // send the transform
  tf_broadcaster_->sendTransform(tf);
}

}  // namespace localization


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto broadcaster = std::make_shared<localization::TfBroadcasterNode>();
  rclcpp::spin(broadcaster);
  rclcpp::shutdown();
  return 0;
}
