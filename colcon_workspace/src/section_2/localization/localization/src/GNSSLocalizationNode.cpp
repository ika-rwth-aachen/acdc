#include <localization/GNSSLocalizationNode.hpp>

#include <GeographicLib/UTMUPS.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/**
 * @brief Namespace for gnss_localization_node
 *
 */
namespace localization
{

// constants
const std::string GNSSLocalizationNode::kInputTopicOdometry{"/odometry"};
const std::string GNSSLocalizationNode::kInputTopicGNSS{"/gnss/navsatfix"};
const std::string GNSSLocalizationNode::kOutputTopicGNSSPoint{"/gnss/map_transformed_point"};
const std::string GNSSLocalizationNode::kOutputTopicGNSSPose{"/gnss/map_transformed_pose"};
const std::string GNSSLocalizationNode::kOutputTopicPredictedPose{"/localization/predicted_pose"};

/**
 * @brief Creates the node inheriting from the Node class.
 *
 */
GNSSLocalizationNode::GNSSLocalizationNode() : rclcpp::Node("gnss_localization_node")
{
  setup();
}

/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void GNSSLocalizationNode::setup()
{
  // create a transform buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // create subscriber for handling incoming messages
  subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    kInputTopicOdometry, 1, std::bind(&GNSSLocalizationNode::odometryCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_odom_->get_topic_name());
  subscriber_gnss_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    kInputTopicGNSS, 1, std::bind(&GNSSLocalizationNode::gnssCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_gnss_->get_topic_name());

  // create a publishers for publishing messages
  publisher_gnss_point_ = this->create_publisher<geometry_msgs::msg::PointStamped>(kOutputTopicGNSSPoint, 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_gnss_point_->get_topic_name());
  publisher_gnss_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(kOutputTopicGNSSPose, 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_gnss_pose_->get_topic_name());
  publisher_predicted_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(kOutputTopicPredictedPose, 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_predicted_pose_->get_topic_name());

}

/**
 * @brief This callback is invoked when the subscriber receives a gnss message
 *
 * @param[in] msg input
 */
void GNSSLocalizationNode::gnssCallback(sensor_msgs::msg::NavSatFix::UniquePtr msg)
{
  geometry_msgs::msg::PointStamped utm_point;
  // apply projection to get utm position from the received WGS84 position
  if(!projectToUTM(msg->latitude, msg->longitude, utm_point)) return;
  utm_point.header.stamp=msg->header.stamp;
  geometry_msgs::msg::PointStamped map_point;
  // transform the utm-position into the carla_map-frame
  // the corresponding transform from utm to carla_map is provided by the tf_broadcaster_node
  if(!transformPoint(utm_point, map_point, "carla_map")) return;
  // publish the gps point as message
  publisher_gnss_point_->publish(map_point);

  // Estimate the yaw angle from two gnss-points within the map-frame
  if(last_gnss_map_point_!=nullptr) // We need two gnss-points to estimate the yaw angle --> check if the last_gnss_map_point_ is available
  {
    geometry_msgs::msg::PoseStamped map_pose;
    estimateGNSSYawAngle(map_point, *last_gnss_map_point_, map_pose);
    // store the map_pose in a member variable
    gnss_map_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(map_pose);
    publisher_gnss_pose_->publish(*gnss_map_pose_);
    new_gnss_pose_ = true; // flag indicating if a new gnss_pose is available
  }
  // Set the current map_point to the last_gnss_map_point_ for the next iteration
  last_gnss_map_point_ = std::make_shared<geometry_msgs::msg::PointStamped>(map_point);
}

/**
 * @brief Get the UTM Position defined by the given latitude and longitude coordinates
 * The position is transformed into UTM by using GeographicLib::UTMUPS
 * 
 * @param[in] latitude latitude coordinate in decimal degree
 * @param[in] longitude longitude coordinate in decimal degree
 * @param[out] geometry_msgs::msg::PointStamped indicating the position in the utm system
 * @return bool indicating if projection was succesful
 */
bool GNSSLocalizationNode::projectToUTM(const double& latitude, const double& longitude, geometry_msgs::msg::PointStamped& utm_point)
{
  try {
    // START TASK 2 CODE HERE
    int zone;
    bool northp;
    utm_point.header.frame_id="utm";
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_point.point.x, utm_point.point.y);
    // return true if succesful
    return true;
    // END TASK 2 CODE HERE
  } catch (GeographicLib::GeographicErr& e) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Tranformation from WGS84 to UTM failed: " << e.what());
    return false;
  }
}

/**
 * @brief This function transforms a given geometry_msgs::msg::PointStamped into a given frame if tf is available
 * 
 * @param[in] input_point 
 * @param[out] output_point 
 * @param[in] output_frame the frame to transform input_point to
 * @return bool indicating if transformation was succesful
 */
bool GNSSLocalizationNode::transformPoint(const geometry_msgs::msg::PointStamped& input_point, geometry_msgs::msg::PointStamped& output_point, const std::string& output_frame)
{
  try {
    // START TASK 3 CODE HERE
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(output_frame, input_point.header.frame_id, input_point.header.stamp);
    tf2::doTransform(input_point, output_point, tf);
    // return true if succesful
    return true;
    // END TASK 3 CODE HERE
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Tranformation from '" << input_point.header.frame_id << "' to '" << output_frame << "' is not available!");
    return false;
  }
}

/**
 * @brief This function estimates the yaw-angle of the vehicle with respect to two given point-measurements
 * 
 * @param[in] current_point the current GNSS Point
 * @param[in] last_point the previous GNSS Point
 * @param[out] output_pose geometry_msgs::msg::Pose including the current_point and an additional 2D orientation
 */
void GNSSLocalizationNode::estimateGNSSYawAngle(const geometry_msgs::msg::PointStamped& current_point, const geometry_msgs::msg::PointStamped& last_point, geometry_msgs::msg::PoseStamped& output_pose)
{
    // START TASK 4 CODE HERE
    // calculate the yaw angle from two sequential gnss-points



    // use header from input point

    // use the position provided through the input point

    // generate a quaternion using the calculated yaw angle



    // END TASK 4 CODE HERE
}

/**
 * @brief This callback is invoked when the subscriber receives an odometry message
 *
 * @param[in] msg input
 */
void GNSSLocalizationNode::odometryCallback(nav_msgs::msg::Odometry::UniquePtr msg)
{
  // store the incoming message in a local object
  nav_msgs::msg::Odometry current_odometry = *msg;
  if(last_odometry_!=nullptr) // We need at least two odometry measurements
  {
    // derive the incremental movement of the vehicle inbetween two odometry measurements
    geometry_msgs::msg::Vector3 delta_translation;
    tf2::Quaternion delta_rotation;
    if(!getIncrementalMovement(current_odometry, *last_odometry_, delta_translation, delta_rotation)) return;
    geometry_msgs::msg::PoseStamped pose;
    // get the initial pose either from gnss or from the previous iteration
    setInitialPose(pose);
    // predict the corresponding vehicle pose
    posePrediction(pose, delta_translation, delta_rotation);
    // Set timestamp to current odometry stamp
    pose.header.stamp = current_odometry.header.stamp;
    // Save predicted pose for next iteration
    predicted_map_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(pose);
    // Publish predicted pose
    publisher_predicted_pose_->publish(*predicted_map_pose_);
  }
  // Save the current odometry as "last_odometry" for next iteration
  last_odometry_ = std::make_shared<nav_msgs::msg::Odometry>(current_odometry);
}

/**
 * @brief This function derives the incremental movement of the vehicle within two sequential odometry measurements
 * 
 * @param current_odometry the current odometry measurement
 * @param previous_odometry the previous odometry measurement
 * @param delta_translation the translation of the vehicle to move from the previous odometry pose to the current odometry pose
 * @param delta_rotation the rotation of the vehicle to move from the previous odometry pose to the current odometry pose
 * @return bool indicating if function call was successful
 */
bool GNSSLocalizationNode::getIncrementalMovement(const nav_msgs::msg::Odometry& current_odometry, const nav_msgs::msg::Odometry& previous_odometry, geometry_msgs::msg::Vector3& delta_translation, tf2::Quaternion& delta_rotation)
{
  try {
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(previous_odometry.child_frame_id, previous_odometry.header.stamp, current_odometry.child_frame_id, current_odometry.header.stamp, previous_odometry.header.frame_id);
    delta_translation = tf.transform.translation;
    tf2::fromMsg(tf.transform.rotation, delta_rotation);
    return true;
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    return false;
  }
}

/**
 * @brief this function sets the initial pose for the current odometry step
 * the function either returns a new gnss-based pose estimate,
 * or the pose derived by the previous iteration
 * 
 * @param[out] initial_pose the initial pose
 */
void GNSSLocalizationNode::setInitialPose(geometry_msgs::msg::PoseStamped& initial_pose)
{
  // use gnss pose if new measurement is available or no pose from previous iteration is available
  if((predicted_map_pose_==nullptr || new_gnss_pose_) && gnss_map_pose_!=nullptr)
  {
    initial_pose = *gnss_map_pose_;
    new_gnss_pose_=false;
  }
  else if(predicted_map_pose_!=nullptr)
  {
    initial_pose = *predicted_map_pose_;
  }
}

/**
 * @brief this function performs the actual prediction of the vehicle pose
 * 
 * @param[out] pose the pose on that delta_translation and delta_rotation is applied
 * @param[in] delta_translation the incremental translation of the vehicle (in vehicle coordinates)
 * @param[in] delta_rotation the incremental rotation of the vehicle (in vehicle coordinates)
 */
void GNSSLocalizationNode::posePrediction(geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Vector3& delta_translation, const tf2::Quaternion& delta_rotation)
{
  // The delta values are given in a vehicle centered frame --> we need to transform them into the map frame
  // First apply delta orientation to the pose
  tf2::Quaternion orientation;
  tf2::fromMsg(pose.pose.orientation, orientation);
  orientation*=delta_rotation; // the multiplication of two quaternions represents two sequential rotations
  pose.pose.orientation = tf2::toMsg(orientation);
  // now perform the transformation of the translation into map coordinates, by using the yaw of the vehicle in map coordinates
  double initial_yaw;
  getYawFromQuaternion(initial_yaw, orientation);
  // START TASK 5 CODE HERE





  // Apply dx and dy (in map coordinates) to the position


  // END TASK 5 CODE HERE
}

/**
 * @brief this function derives the yaw angle from a given quaternion
 * 
 * @param[out] yaw angle (in radians)
 * @param[in] quaternion to derive the yaw angle from 
 */
void GNSSLocalizationNode::getYawFromQuaternion(double& yaw, const tf2::Quaternion& quaternion)
{
  tf2::Matrix3x3 m(quaternion);
  double roll, pitch;
  m.getRPY(roll, pitch, yaw);
}

}  // namespace localization


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto localization = std::make_shared<localization::GNSSLocalizationNode>();
  rclcpp::spin(localization);
  rclcpp::shutdown();
  return 0;
}
