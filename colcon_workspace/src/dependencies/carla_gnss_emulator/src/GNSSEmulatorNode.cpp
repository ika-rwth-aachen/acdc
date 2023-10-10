#include <localization/GNSSEmulatorNode.hpp>

#include <GeographicLib/UTMUPS.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/**
 * @brief Namespace for gnss_localization_node
 *
 */
namespace localization
{

// constants
const std::string GNSSEmulatorNode::kInputTopicOdometry{"/ground_truth/pose"};
const std::string GNSSEmulatorNode::kOutputTopicGNSS{"/gnss/navsatfix"};

// parameter names
const std::string GNSSEmulatorNode::kPublishPeriodParam{"gnss_publish_period"};
const std::string GNSSEmulatorNode::kUTMZoneParam{"utm_zone"};
const std::string GNSSEmulatorNode::kNorthernHemisphereParam{"is_northern_hemisphere"};


/**
 * @brief Creates the node inheriting from the Node class.
 *
 */
GNSSEmulatorNode::GNSSEmulatorNode() : rclcpp::Node("gnss_emulator_node")
{
  setup();
}

/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void GNSSEmulatorNode::setup()
{
  // create a transform buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Load parameters
  // set parameter description
  rcl_interfaces::msg::ParameterDescriptor period_desc;
  period_desc.description = "Period between published messages";

  // set allowed parameter range
  rcl_interfaces::msg::FloatingPointRange period_range;
  period_range.set__from_value(0.1).set__to_value(10.0).set__step(0.1);
  period_desc.floating_point_range = {period_range};

  // Publish Period
  this->declare_parameter(kPublishPeriodParam, rclcpp::ParameterType::PARAMETER_DOUBLE, period_desc);
  try {
    period_ = this->get_parameter(kPublishPeriodParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException &) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kPublishPeriodParam.c_str());
    exit(EXIT_FAILURE);
  } 
  // UTM Zone
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = "UTM Zone of utm-frame defined by carla-map -> utm tf";
  this->declare_parameter(kUTMZoneParam, rclcpp::ParameterType::PARAMETER_INTEGER, desc);
  try {
    zone_ = this->get_parameter(kUTMZoneParam).as_int();
  } catch (rclcpp::exceptions::ParameterUninitializedException &) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kUTMZoneParam.c_str());
    exit(EXIT_FAILURE);
  } 
  // Is northern hemisphere
  desc.description = "Bool indicating if utm-frame defined by carla-map -> utm tf is in northern hemisphere";
  this->declare_parameter(kNorthernHemisphereParam, rclcpp::ParameterType::PARAMETER_BOOL, desc);
  try {
    northp_ = this->get_parameter(kNorthernHemisphereParam).as_bool();
  } catch (rclcpp::exceptions::ParameterUninitializedException &) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kPublishPeriodParam.c_str());
    exit(EXIT_FAILURE);
  } 

  // create subscriber for handling incoming messages
  subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    kInputTopicOdometry, 1, std::bind(&GNSSEmulatorNode::odometryCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_odom_->get_topic_name());

  // create a publisher for publishing messages
  publisher_gnss_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(kOutputTopicGNSS, 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_gnss_->get_topic_name());
  last_pub_=now();
}

/**
 * @brief This callback is invoked when the subscriber receives an odometry message
 *
 * @param[in] msg   input
 */
void GNSSEmulatorNode::odometryCallback(nav_msgs::msg::Odometry::UniquePtr msg)
{
  // Check for duration
  if((now()-last_pub_).seconds()>period_)
  {
    geometry_msgs::msg::PointStamped utm_point;
    geometry_msgs::msg::PointStamped carla_map_point;
    carla_map_point.header = msg->header;
    carla_map_point.point = msg->pose.pose.position;
    // transform the point within the carla_map frame into the utm-frame
    // the corresponding transform is published by the tf_broadcaster_node
    if(!transformPoint(carla_map_point, utm_point, "utm")) return;
    double latitude, longitude;
    // perform the projection to get WGS84 coordinates from a given utm point
    if(!projectToWGS84(utm_point.point, latitude, longitude)) return;
    // write ROS NavSatFix message
    sensor_msgs::msg::NavSatFix navsatfix;
    navsatfix.header.stamp = msg->header.stamp;
    navsatfix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    navsatfix.altitude = utm_point.point.z;
    navsatfix.latitude = latitude;
    navsatfix.longitude = longitude;
    // publish the message
    publisher_gnss_->publish(navsatfix);
    last_pub_=now();
    return;
  }
  else return;
}

/**
 * @brief Get the WGS84 Position defined by the given utm point coordinates
 * The position is transformed into WGS84 by using GeographicLib::UTMUPS
 * 
 * @param[in] geometry_msgs::msg::Point utm point
 * @param[out] latitude latitude coordinate
 * @param[out] longitude longitude coordinate
 * @return bool indicating if projection was successfully
 */
bool GNSSEmulatorNode::projectToWGS84(const geometry_msgs::msg::Point& utm_point, double& latitude, double& longitude)
{
  try {
    GeographicLib::UTMUPS::Reverse(zone_, northp_, utm_point.x, utm_point.y, latitude, longitude);
    return true;
  } catch (GeographicLib::GeographicErr& e) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Tranformation from UTM to WGS84 failed: " << e.what());
    return false;
  }
}

/**
 * @brief Transform a geometry_msgs::PointStamped into a defined frame
 * 
 * @param input_point 
 * @param output_point 
 * @param output_frame 
 * @return bool indicating if the transformation was successfull 
 */
bool GNSSEmulatorNode::transformPoint(const geometry_msgs::msg::PointStamped& input_point, geometry_msgs::msg::PointStamped& output_point, const std::string& output_frame)
{
  try {
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(output_frame, input_point.header.frame_id, input_point.header.stamp);
    tf2::doTransform(input_point, output_point, tf);
    return true;
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Tranformation from '" << input_point.header.frame_id << "' to '" << output_frame << "' is not available!");
    return false;
  }
}

}  // namespace localization


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto emulation = std::make_shared<localization::GNSSEmulatorNode>();
  rclcpp::spin(emulation);
  rclcpp::shutdown();
  return 0;
}
