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

#include "sensors/LaserScanner.h"
#include "sensors/LaserCallback.h"

#include <flatland_server/collision_filter_registry.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>

#include <cmath>
#include <limits>
#include <bitset>
#include <memory>
#include <utility>
#include <fstream>

#include <geometry_msgs/Point.h>

namespace flatland_ika_plugins {

std::vector<tf::Point> read_first_column_of_csv_file(const std::string &file,
                                                     const std::string &separator,
                                                     float resolution) {
  // Assert all pre conditions
  assert(resolution != 0);
  assert(!separator.empty());
  assert(!file.empty());

  // Allocate buffer to read a file
  std::ifstream in_stream(file);
  std::string line_buffer;
  std::vector<tf::Point> point_buffer;

  // Check if stream is valid
  if (in_stream.is_open()) {
    // Iterate through all lines in this file
    while (std::getline(in_stream, line_buffer)) {
      // Split lines at separator
      std::vector<std::string> parts;
      boost::split(parts, line_buffer, boost::is_any_of(separator));
      // Assert correct file format
      assert(parts.size() >= 2);
      point_buffer.emplace_back(tf::Point(boost::lexical_cast<float>(parts.at(0)) * resolution,
                                          boost::lexical_cast<float>(parts.at(1)) * resolution,
                                          1.0));
    }
  } else {
    // Stream wasn't valid returning empty buffer
    //TODO Throw exception here
    ROS_WARN("Csv file: %s could not be parsed.", file.c_str());
  }

  // Close file and return buffer
  in_stream.close();
  return point_buffer;
}

void LaserScanner::parse_parameter_from_node(const YAML::Node &config) {
  ROS_INFO("Laser scanner %s is parsing parameter.", GetName().c_str());
  // Read parameter from Yaml file
  flatland_server::YamlReader reader(config);
  scanner_body_name_ = reader.Get<std::string>("body");
  laser_scan_topic_ = reader.Get<std::string>("laser_scan_topic", "scan");
  marker_topic_ = reader.Get<std::string>("marker_topic", "marker");
  scanner_frame_id_ = reader.Get<std::string>("frame", scanner_frame_id_);
  world_frame_id_ = reader.Get<std::string>("world_frame_id", world_frame_id_);
  update_rate_ = reader.Get<double>("update_rate", std::numeric_limits<double>::infinity());
  scanner_origin_ = reader.GetPose("origin", flatland_server::Pose(0, 0, 0));
  max_range_ = reader.Get<double>("range");
  noise_std_dev_ = reader.Get<double>("noise_std_dev", 0);

  // Read from sub node angle
  flatland_server::YamlReader angle_reader = reader.Subnode("angle", flatland_server::YamlReader::MAP);
  min_angle_ = angle_reader.Get<double>("min");
  max_angle_ = angle_reader.Get<double>("max");
  increment_angle_ = angle_reader.Get<double>("increment");
  // Ensure that no none read angle property is left
  angle_reader.EnsureAccessedAllKeys();
  left_border_file_ = reader.Get<std::string>("left_border", "left_border");
  right_border_file_ = reader.Get<std::string>("right_border", "right_border");
  max_lane_marking_iterations_ = reader.Get<int>("lane_marking_iterations", max_lane_marking_iterations_);
  max_lane_marking_length_ = reader.Get<float>("lane_marking_distance", max_lane_marking_length_);
  //Ensure that none read global property is left
  reader.EnsureAccessedAllKeys();

  yaml_dir_ = boost::filesystem::path(GetModel()->plugins_reader_.file_path_).parent_path();
  //auto right_border_buffer = readCSV(right_border);
}

void LaserScanner::check_parameter() {
  ROS_INFO("Laser scanner %s is checking parameter.", GetName().c_str());

  // Check if angles are defined well
  if (max_angle_ < min_angle_) throw flatland_server::YAMLException("Invalid angles defined.");
  if (increment_angle_ <= 0) throw flatland_server::YAMLException("Invalid angles defined.");
  // Only two rays are allowed
  number_of_laser_rays_ = std::lround((max_angle_ - min_angle_) / increment_angle_) + 1;
  if (number_of_laser_rays_ != 2) throw flatland_server::YAMLException("Invalid angles defined.");

  // Check if body name is valid
  auto body_ = GetModel()->GetBody(scanner_body_name_);
  if (!body_) throw flatland_server::YAMLException("Invalid body name.");
}

void LaserScanner::OnInitialize(const YAML::Node &config) {
  ROS_INFO("Laser scanner %s initialises.", GetName().c_str());

  parse_parameter_from_node(config);
  check_parameter();

  ROS_INFO("Laser scanner %s creates publisher.", GetName().c_str());
  // Initialise publisher
  laser_scan_publisher_ = std::make_unique<ros::Publisher>();
  *laser_scan_publisher_ = nh_.advertise<sensor_msgs::LaserScan>(laser_scan_topic_, 1);
  marker_publisher_ = std::make_unique<ros::Publisher>();
  *marker_publisher_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 1);

  ROS_INFO("Laser scanner %s creates laser scan.", GetName().c_str());

  // Initialise laser scan message

  laser_results_.resize(number_of_laser_rays_, LaserCallback(this, 0));

  laser_scan_.angle_min = min_angle_;
  laser_scan_.angle_max = max_angle_;
  laser_scan_.angle_increment = increment_angle_;
  laser_scan_.time_increment = 0.0;
  laser_scan_.scan_time = 0.0;
  laser_scan_.range_min = 0.0;
  laser_scan_.range_max = max_range_;
  laser_scan_.ranges.resize(number_of_laser_rays_);
  laser_scan_.intensities.resize(number_of_laser_rays_);

  laser_scan_.header.seq = 0;
  laser_scan_.header.frame_id = tf::resolve("", GetModel()->NameSpaceTF(scanner_frame_id_));

  ROS_INFO("Laser scanner %s grabs body from simulation.", GetName().c_str());

  // Initialise flatland body
  scanner_body_ = GetModel()->GetBody(scanner_body_name_);

  ROS_INFO("Laser scanner %s creates transform broadcaster.", GetName().c_str());

  // Initialise transform broadcaster
  transform_broadcaster_ = std::make_unique<tf::TransformBroadcaster>();
  transform_listener_scanner_world_ = std::make_unique<tf::TransformListener>();

  ROS_INFO("Laser scanner %s creates laser to body transformation.", GetName().c_str());
  // Initialise transform between the body and laser
  tf::Quaternion q;
  q.setRPY(0, 0, scanner_origin_.theta);
  laser_to_body_transform_.header.frame_id = tf::resolve("", GetModel()->NameSpaceTF(scanner_body_->GetName()));
  laser_to_body_transform_.child_frame_id = tf::resolve("", GetModel()->NameSpaceTF(scanner_frame_id_));
  laser_to_body_transform_.transform.translation.x = scanner_origin_.x;
  laser_to_body_transform_.transform.translation.y = scanner_origin_.y;
  laser_to_body_transform_.transform.translation.z = 0;
  laser_to_body_transform_.transform.rotation.x = q.x();
  laser_to_body_transform_.transform.rotation.y = q.y();
  laser_to_body_transform_.transform.rotation.z = q.z();
  laser_to_body_transform_.transform.rotation.w = q.w();

  ROS_INFO("Laser scanner %s creates noise generator.", GetName().c_str());

  // Init random noise generator
  std::random_device rd;
  random_number_generator_ = std::make_unique<std::default_random_engine>(rd());
  random_noise_generator_ = std::make_unique<std::normal_distribution<double>>(0.0, noise_std_dev_);

  ROS_INFO("Laser scanner %s performes pre-calculations.", GetName().c_str());

  // Pre-calculate the laser points w.r.t to the laser frame, since this never changes
  for (unsigned int i = 0; i < number_of_laser_rays_; i++) {
    double angle = min_angle_ + i * increment_angle_;
    double x = max_range_ * std::cos(angle);
    double y = max_range_ * std::sin(angle);

    tf::Point locale_target_point(x, y, 1);
    locale_laser_target_points_.push_back(locale_target_point);
    locale_laser_angles_.push_back(angle);
  }

  ROS_INFO("Laser scanner %s create clock.", GetName().c_str());
  // Initialise time for update rate
  update_timer_.SetRate(update_rate_);

  ROS_INFO("Laser scanner %s initialising lane markings.", GetName().c_str());
  // Initialise ros marker
  marker_left_lane_.header.frame_id = tf::resolve("", GetModel()->NameSpaceTF(scanner_frame_id_));;
  marker_left_lane_.ns = "lane_markings";
  marker_left_lane_.action = visualization_msgs::Marker::ADD;
  marker_left_lane_.pose.orientation.w = 1.0;
  marker_left_lane_.id = 0;

  marker_left_lane_.type = visualization_msgs::Marker::LINE_STRIP;
  marker_left_lane_.scale.x = 0.4;

  marker_left_lane_.color.r = 1.0;
  marker_left_lane_.color.a = 0.5;

  marker_right_lane_ = marker_left_lane_;
  marker_right_lane_.id = 1;

  marker_debug_ = marker_left_lane_;
  marker_debug_.header.frame_id = tf::resolve("", "map");
  marker_debug_.type = visualization_msgs::Marker::POINTS;
  marker_debug_.scale.x = 100.0;
  marker_debug_.scale.y = 100.0;
  marker_debug_.color.r = 0.0;
  marker_debug_.color.b = 1.0;
  marker_debug_.id = 2;

  ROS_INFO("Laser scanner %s parsing boundary files.", GetName().c_str());
  // Parse lane boundary files
  auto left_border_path = yaml_dir_ / left_border_file_;
  left_border_buffer_ = read_first_column_of_csv_file(left_border_path.c_str(), "\t", 0.1f);
  auto right_border_path = yaml_dir_ / right_border_file_;
  right_border_buffer_ = read_first_column_of_csv_file(right_border_path.c_str(), "\t", 0.1f);

}

void LaserScanner::extract_laser_scan_from_laser_results() {
  // Extract all callbacks and apply noise
  for (unsigned int i = 0; i < laser_results_.size(); ++i) {
    auto result = laser_results_[i];
    laser_scan_.ranges[i] = result.getDistance() + (*this->random_noise_generator_)(*this->random_number_generator_);
    laser_scan_.intensities[i] = result.getIntensity();
  }
}

void LaserScanner::extract_lanes_from_laser_scan(std::vector<tf::Point> lane_buffer,
                                                 float lane_intensity,
                                                 visualization_msgs::Marker &marker) {
  // Check scanner orientation against left/right assumption
  int index = -1;
  if (laser_scan_.intensities[0] == lane_intensity) {
    index = 0;
  } else if (laser_scan_.intensities[1] == lane_intensity) {
    index = 1;
  }
  // Getting current orientation failed.
  if (index == -1) {
    ROS_WARN("No lanes found!");
    marker.points.clear();
    return;
  }
  // Get the laser scan that correlates to the current lane marking
  float range = laser_scan_.ranges[index];
  float angle = locale_laser_angles_.at(index);
  tf::Point laser_point(range * std::cos(angle), range * std::sin(angle), 1.0f);

  // Transform the current lane marking buffer in local frame
  auto transformed_lane_buffer = std::move(lane_buffer);
  int transformed_lane_buffer_length = transformed_lane_buffer.size();
  for (auto &value : transformed_lane_buffer) {
    value = scanner_to_map_transform_.inverse()(value);
  }

  // Find starting point in the lane marking buffer
  float min_distance = std::numeric_limits<float>::infinity();
  int min_distance_index = std::numeric_limits<size_t>::quiet_NaN();
  for (size_t i = 0; i < transformed_lane_buffer.size(); i++) {
    auto value = transformed_lane_buffer.at(i);
    float distance = (laser_point - value).length();
    if (distance < min_distance) {
      min_distance = distance;
      min_distance_index = i;
    }
  }

  // Clear lane markings and push starting point (laser point)
  marker.points.clear();
  geometry_msgs::Point p;
  p.x = laser_point.x();
  p.y = laser_point.y();
  marker.points.push_back(p);

  auto current_point = transformed_lane_buffer.at(min_distance_index);
  int shift = 1;
  // Check of the positive walking direction on the buffer is current driving direction
  {
    int next_index_in_positive_direction = min_distance_index + 1;
    if (next_index_in_positive_direction >= transformed_lane_buffer_length) next_index_in_positive_direction = 0;
    auto next_point_in_positive_direction = transformed_lane_buffer.at(next_index_in_positive_direction);
    auto shift_to_next_point_in_positive_direction = next_point_in_positive_direction - current_point;
    double word_direction_of_next_point_in_positive_direction =
      std::atan2(shift_to_next_point_in_positive_direction.y(), shift_to_next_point_in_positive_direction.x());
    if (std::abs(word_direction_of_next_point_in_positive_direction) > M_PI_2) shift = -shift;
  }
  // Shifting by one times shift is no the driving direction

  // Store the traveled distance on the line
  float break_distance = 0;
  // Emergency breaking counter to prevent inf loop
  for (size_t i = 0; i < max_lane_marking_iterations_; i++) {
    // Make a cycling list for next index
    int next_index = min_distance_index + shift;
    if (next_index >= transformed_lane_buffer_length) next_index = 0;
    if (next_index < 0) next_index = transformed_lane_buffer_length + shift;

    auto next_point = transformed_lane_buffer.at(next_index);

    float distance_traveled = (next_point - current_point).length();
    break_distance += distance_traveled;
    current_point = next_point;
    // Push next point
    p.x = next_point.x();
    p.y = next_point.y();
    marker.points.push_back(p);

    // Make a cycling list for current index
    min_distance_index = min_distance_index + shift;
    if (min_distance_index >= transformed_lane_buffer_length) min_distance_index = 0;
    if (min_distance_index < 0) min_distance_index = transformed_lane_buffer_length + shift;

    // Alternative breaking condition of lane reached max length
    if (break_distance > max_lane_marking_length_) {
      break;
    }
  }
}

void LaserScanner::computer_laser_results(const flatland_server::Timekeeper &timekeeper) {
  // Convert local origin to global origin
  tf::Point local_laser_origin = {0.0, 0.0, 1.0};
  tf::Point world_laser_origin = scanner_to_map_transform_(local_laser_origin);

  // Convert the origin to physics engine data type
  b2Vec2 laser_origin_point(world_laser_origin.x(), world_laser_origin.y());

  // Dequeue all the results from future objects
  for (unsigned int i = 0; i < laser_results_.size(); ++i) {
    // Convert the target point to physics engine data type
    tf::Point world_laser_target_point = scanner_to_map_transform_(locale_laser_target_points_.at(i));
    b2Vec2 laser_point(world_laser_target_point.x(), world_laser_target_point.y());
    // Allocate the callback buffer to store it
    flatland_ika_plugins::LaserCallback callback(this, locale_laser_angles_.at(i));
    // Get physics engine world to shoot ray
    b2World *world = GetModel()->GetPhysicsWorld();
    // Shoot ray
    world->RayCast(&callback, laser_origin_point, laser_point);
    laser_results_.at(i) = callback;
  }
}

void LaserScanner::BeforePhysicsStep(const flatland_server::Timekeeper &timekeeper) {
  // Update transform as fast as possible
  laser_to_body_transform_.header.stamp = timekeeper.GetSimTime();
  transform_broadcaster_->sendTransform(laser_to_body_transform_);

  // Cancel if update rate is not reached yet
  // This will limit this
  if (!update_timer_.CheckUpdate(timekeeper)) return;
  try {
    transform_listener_scanner_world_->lookupTransform(world_frame_id_,
                                                       scanner_frame_id_,
                                                       ros::Time(0),
                                                       scanner_to_map_transform_);
  }catch (const std::exception& e) {return;}
  // Compute now laser scan
  computer_laser_results(timekeeper);
  extract_laser_scan_from_laser_results();
  // Compute new lane markings
  extract_lanes_from_laser_scan(left_border_buffer_, INTENSITIES::LEFT_LANE, marker_left_lane_);
  extract_lanes_from_laser_scan(right_border_buffer_, INTENSITIES::RIGHT_LANE, marker_right_lane_);

  // Publish laser scan
  laser_scan_.header.stamp = timekeeper.GetSimTime();
  laser_scan_publisher_->publish(laser_scan_);

  // Publish lane markings
  marker_left_lane_.header.stamp = timekeeper.GetSimTime();
  marker_right_lane_.header.stamp = timekeeper.GetSimTime();
  marker_publisher_->publish(marker_left_lane_);
  marker_publisher_->publish(marker_right_lane_);

}

double LaserScanner::getMaxRange() {
  return max_range_;
}

};

PLUGINLIB_EXPORT_CLASS(flatland_ika_plugins::LaserScanner, flatland_server::ModelPlugin)