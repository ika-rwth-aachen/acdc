#ifndef FLATLAND_IKA_PLUGINS_LASER_SCANNER_H
#define FLATLAND_IKA_PLUGINS_LASER_SCANNER_H

#include <ros/ros.h>

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thirdparty/ThreadPool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

#include <random>
#include <thread>

#include <definitions/IkaObject.h>
#include <definitions/IkaObjectList.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace flatland_ika_plugins {

class LaserCallback;

/**
 * @brief This class implements the model plugin class and provides laser scanner with lane marking extraction features.
 * @details This laser scanner only uses two laser rays! Every other configuration is invalid. This laser scanner will
 * not intersect with any object that has not a 2d or 3d name prefix. Both intersected object have to have a name
 * containing left or right to be a valid lane marking.
 */
class LaserScanner : public flatland_server::ModelPlugin {
 public:
  // Intensities used by this laser scanner
  enum INTENSITIES {
    LEFT_LANE = 90,
    RIGHT_LANE = 110,
    ENVIRONMENT = 100
  };

 private:
  std::string left_border_file_ = ""; ///< File that contains the left border data
  std::string right_border_file_ = ""; ///< File that contains the right border data
  boost::filesystem::path yaml_dir_ = ""; ///< Directory to look for files mentioned above

  std::vector<tf::Point> left_border_buffer_; ///< Buffer for the left border points
  std::vector<tf::Point> right_border_buffer_; ///< Buffer for the right border points

  std::string laser_scan_topic_ = ""; ///< Topic were the laser scan should be published
  std::string marker_topic_ = ""; ///< Topic were the marker array should be published
  std::unique_ptr<ros::Publisher> laser_scan_publisher_; ///< Publisher for the laser scans
  std::unique_ptr<ros::Publisher> marker_publisher_; ///< Publisher for the marker array

  visualization_msgs::Marker marker_left_lane_; ///< Current left lane marker buffer
  visualization_msgs::Marker marker_right_lane_; ///< Current right lane marker buffer
  visualization_msgs::Marker marker_debug_; ///< Current right lane marker buffer

  size_t number_of_laser_rays_ = 0; ///< Number of rays in this scanner
  sensor_msgs::LaserScan laser_scan_; ///< Current laser scan buffer
  std::vector<LaserCallback> laser_results_; ///< Buffer for the real laser scan of the simulation

  std::string scanner_body_name_ = ""; ///< Flatland body name of scanner
  flatland_server::Body *scanner_body_ = nullptr; ///< Flatland body of the laser scanner
  flatland_server::Pose scanner_origin_ = {0, 0, 0}; ///< Origin of the laser scanner

  double max_range_ = 0.0; ///< Maximum range of the laser rays
  double max_angle_ = 0.0; ///< Maximum angle of the laser scanner
  double min_angle_ = 0.0; ///< Minimum angle of the laser scanner
  double increment_angle_ = 0.0; ///< Increment of the laser scanner
  double update_rate_ = 0.0; ///< Update rate of the laser scanner

  int max_lane_marking_iterations_ = 500; ///< Max number of iterations that are performed to extract a lane marking
  float max_lane_marking_length_ = 50.0f; ///< Max meters traveled to extract a lane marking

  std::string scanner_frame_id_ = "laser_front_ref";  ///< Frame id of the laser scanner
  std::string world_frame_id_ = "map";  ///< Frame id of the world

  std::unique_ptr<tf::TransformBroadcaster> transform_broadcaster_; ///< Transform broadcaster
  geometry_msgs::TransformStamped laser_to_body_transform_;  ///< Transform for laser to body frame

  std::unique_ptr<tf::TransformListener> transform_listener_scanner_world_;
  tf::StampedTransform scanner_to_map_transform_;  ///< Transform for body to world frame

  double noise_std_dev_ = 0.0; ///< Std. deviation of the noise
  std::unique_ptr<std::default_random_engine> random_number_generator_; ///< Generator for random numbers
  std::unique_ptr<std::normal_distribution<double>> random_noise_generator_; ///< Gaussian noise generator

  std::vector<tf::Point> locale_laser_target_points_; ///< Buffer for the pre-calculated laser scanner target points
  std::vector<float> locale_laser_angles_; ///< Buffer for the angle of the laser scanner rays

  flatland_plugins::UpdateTimer update_timer_; ///< Time to keep update rate

  /**
   * @brief Will parse all configurations from the yaml file.
   * @param config Content of yaml file.
   */
  void parse_parameter_from_node(const YAML::Node &config);

  /**
   * @breif Will check parameter for consistency.
   */
  void check_parameter();

  /**
   * @brief Calculates new laser data from simulation.
   * @param timekeeper Current simulation time.
   */
  void computer_laser_results(const flatland_server::Timekeeper &timekeeper);

  /**
   * @brief Extract laser scan from simulation results.
   */
  void extract_laser_scan_from_laser_results();

  /**
   * @brief Extraxt lane markings from simulation results.
   * @param lane_buffer Buffer to read from.
   * @param lane_intensity Intensity to search rays for.
   * @param marker Buffer to write in
   */
  void extract_lanes_from_laser_scan(std::vector<tf::Point> lane_buffer, float lane_intensity, visualization_msgs::Marker& marker);

 public:

  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Called when just before physics update
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const flatland_server::Timekeeper &timekeeper) override;

  /**
   * @brief Getter for max range of laser scanner.
   */
  double getMaxRange();

};
};

#endif // FLATLAND_IKA_PLUGINS_LASER_SCANNER_H
