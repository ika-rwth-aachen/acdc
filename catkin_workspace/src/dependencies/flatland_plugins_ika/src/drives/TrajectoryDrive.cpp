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

#include "drives/TrajectoryDrive.h"

// C++
#include <random>
#include <algorithm>

// ROS
#include <tf/tf.h>
#include <angles/angles.h>

// Flatland
#include <pluginlib/class_list_macros.h>

// Eigen
#include <Eigen/Geometry>
#include <Eigen/Dense>

void flatland_ika_plugins::TrajectoryDrive::OnInitialize(const YAML::Node &config) {
  flatland_server::YamlReader config_reader(config);

  // load body names
  std::string vehicle_body_name = config_reader.Get<std::string>("vehicle_body");
  std::string front_left_tire_body_name = config_reader.Get<std::string>("front_left_tire_body");
  std::string front_right_tire_body_name = config_reader.Get<std::string>("front_right_tire_body");
  std::string rear_left_tire_body_name = config_reader.Get<std::string>("rear_left_tire_body");
  std::string rear_right_tire_body_name = config_reader.Get<std::string>("rear_right_tire_body");

  // load joint names
  std::string front_left_tire_joint_name = config_reader.Get<std::string>("front_left_tire_joint");
  std::string front_right_tire_joint_name = config_reader.Get<std::string>("front_right_tire_joint");
  std::string rear_left_tire_joint_name = config_reader.Get<std::string>("rear_left_tire_joint");
  std::string rear_right_tire_joint_name = config_reader.Get<std::string>("rear_right_tire_joint");

  // load odom frame id
  std::string odom_frame_id = config_reader.Get<std::string>("odom_frame_id", "odom");

  // load topics
  std::string target_gps_topic = config_reader.Get<std::string>("target_gps_topic", "vehicle/gps");
  std::string target_ego_motion_topic = config_reader.Get<std::string>("target_ego_motion_topic", "vehicle/ego_motion");

  std::string odom_topic = config_reader.Get<std::string>("odom_pub", "odometry/filtered");
  std::string marker_topic = config_reader.Get<std::string>("marker_topic", "vehicle/marker");

  // load update rate
  auto update_rate = config_reader.Get<double>("update_rate", std::numeric_limits<double>::infinity());
  update_timer_.SetRate(update_rate);

  config_reader.EnsureAccessedAllKeys();

  // get bodies and joints
  vehicle_body_ = GetModel()->GetBody(vehicle_body_name);
  if (vehicle_body_ == nullptr) {
    throw flatland_server::YAMLException("Body with name " + flatland_server::Q(vehicle_body_name) + " does not exist");
  }

  tire_bodies_[0] = GetModel()->GetBody(front_right_tire_body_name);
  if (tire_bodies_[0] == nullptr) {
    throw flatland_server::YAMLException(
        "Body with name " + flatland_server::Q(front_right_tire_body_name) + " does not exist");
  }

  tire_bodies_[1] = GetModel()->GetBody(rear_right_tire_body_name);
  if (tire_bodies_[1] == nullptr) {
    throw flatland_server::YAMLException(
        "Body with name " + flatland_server::Q(rear_right_tire_body_name) + " does not exist");
  }

  tire_bodies_[2] = GetModel()->GetBody(rear_left_tire_body_name);
  if (tire_bodies_[2] == nullptr) {
    throw flatland_server::YAMLException(
        "Body with name " + flatland_server::Q(rear_left_tire_body_name) + " does not exist");
  }

  tire_bodies_[3] = GetModel()->GetBody(front_left_tire_body_name);
  if (tire_bodies_[3] == nullptr) {
    throw flatland_server::YAMLException(
        "Body with name " + flatland_server::Q(front_left_tire_body_name) + " does not exist");
  }

  tire_joints_[0] = GetModel()->GetJoint(front_right_tire_joint_name);
  if (tire_joints_[0] == nullptr) {
    throw flatland_server::YAMLException(
        "Joint with name " + flatland_server::Q(front_right_tire_joint_name) + " does not exist");
  }

  tire_joints_[1] = GetModel()->GetJoint(rear_right_tire_joint_name);
  if (tire_joints_[1] == nullptr) {
    throw flatland_server::YAMLException(
        "Joint with name " + flatland_server::Q(rear_right_tire_joint_name) + " does not exist");
  }

  tire_joints_[2] = GetModel()->GetJoint(rear_left_tire_joint_name);
  if (tire_joints_[2] == nullptr) {
    throw flatland_server::YAMLException(
        "Joint with name " + flatland_server::Q(rear_left_tire_joint_name) + " does not exist");
  }

  tire_joints_[3] = GetModel()->GetJoint(front_left_tire_joint_name);
  if (tire_joints_[3] == nullptr) {
    throw flatland_server::YAMLException(
        "Joint with name " + flatland_server::Q(front_left_tire_joint_name) + " does not exist");
  }

  // publish and subscribe to topics
  target_ego_sub_ = nh_.subscribe(target_ego_motion_topic, 1, &TrajectoryDrive::EgoMotionCallback, this);
  target_gps_sub_ = nh_.subscribe(target_gps_topic, 1, &TrajectoryDrive::GpsCallback, this);

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 1);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic, 1);

  // init the values for the messages
  odom_msg_.header.frame_id = odom_frame_id;
  odom_msg_.child_frame_id = tf::resolve("", GetModel()->NameSpaceTF(vehicle_body_->name_));

  odom_msg_.twist.covariance.fill(0);
  odom_msg_.pose.covariance.fill(0);

  /*for (auto *f = vehicle_body_->physics_body_->GetFixtureList(); f; f = f->GetNext()) {
    auto filter = f->GetFilterData();
    filter.categoryBits = 0x000F;
    f->SetFilterData(filter);
  }*/

  line_strip.header.frame_id = odom_frame_id;
  line_strip.ns = "points_and_lines";
  line_strip.action = visualization_msgs::Marker::ADD;

  line_strip.pose.orientation.w = 1.0;

  line_strip.id = 0;

  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  line_strip.scale.x = 0.05;

  line_strip.color.r = 1.0;
  line_strip.color.g = 1.0;
  line_strip.color.b = 0.3;
  line_strip.color.a = 0.5;


  // Set a fix reference point
  this->reference_gps_.fUTMNorth = 5630663;
  this->reference_gps_.fUTMEast = 291911;
  this->gps_referenced = true;
}

void flatland_ika_plugins::TrajectoryDrive::AfterPhysicsStep(const flatland_server::Timekeeper &timekeeper) {
  bool publish = update_timer_.CheckUpdate(timekeeper);
  if (!publish) return;

  b2Body *b2body = vehicle_body_->physics_body_;
  b2Vec2 vehicle_position = b2body->GetPosition();
  b2Vec2 vehicle_velocity = b2body->GetLinearVelocity();
  float vehicle_angle = b2body->GetAngle();
  float vehicle_angular_velocity = b2body->GetAngularVelocity();

  odom_msg_.header.stamp = timekeeper.GetSimTime();
  odom_msg_.pose.pose.position.x = vehicle_position.x;
  odom_msg_.pose.pose.position.y = vehicle_position.y;
  odom_msg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(vehicle_angle);
  odom_msg_.twist.twist.linear.x = vehicle_velocity.x;
  odom_msg_.twist.twist.linear.y = vehicle_velocity.y;
  odom_msg_.twist.twist.angular.z = vehicle_angular_velocity;

  odom_pub_.publish(odom_msg_);
}
void flatland_ika_plugins::TrajectoryDrive::GpsCallback(const definitions::IkaGPS &msg) {
  this->current_gps_ = msg;

  // Use first message as reference if not referenced before
  if (!gps_referenced) {
    this->reference_gps_ = msg;
    this->gps_referenced = true;
  }

  // Reference the new gps position relative to the reference point
  Eigen::Vector2d reference_position = {this->reference_gps_.fUTMEast, this->reference_gps_.fUTMNorth};
  Eigen::Vector2d target_position = {this->current_gps_.fUTMEast, this->current_gps_.fUTMNorth};
  target_position -= reference_position;

  // Get new position
  double x = target_position.x();
  double y = target_position.y();
  double target_heading = angles::from_degrees(current_gps_.fUTMHeading);
  b2Vec2 new_position(x, y);

  // Force all sub elements of the vehicle to the new position
  b2JointEdge *edge = vehicle_body_->physics_body_->GetJointList();
  while (edge != nullptr) {
    b2Joint *joint = edge->joint;
    b2Body *body = joint->GetBodyA();
    b2Vec2 anchor = vehicle_body_->physics_body_->GetLocalPoint(joint->GetAnchorB());
    Eigen::Vector2f anchor_eigen = {anchor.x, anchor.y};
    Eigen::Rotation2D<float> eigen_rotation(target_heading);
    anchor_eigen = eigen_rotation * anchor_eigen;
    anchor = {anchor_eigen.x(), anchor_eigen.y()};
    body->SetTransform(new_position + anchor, body->GetAngle());

    edge = edge->next;
  }
  // Force main body to new position
  this->vehicle_body_->physics_body_->SetTransform(new_position, target_heading);

  // Set velocity values
  b2Vec2 new_velocity = {static_cast<float>(current_gps_.fVelEast), static_cast<float>(current_gps_.fVelNorth)};
  double new_angular_velocity = current_gps_.fAngRateZ;

  vehicle_body_->physics_body_->SetLinearVelocity({0,0});
  vehicle_body_->physics_body_->SetAngularVelocity(0);

  // Get the gps position and creat a marker
  // What happens if the buffer of arrays gets very big?
  geometry_msgs::Point p;
  p.x = new_position.x;
  p.y = new_position.y;
  p.z = 0.0;
  // Push marker to array
  line_strip.points.push_back(p);
  line_strip.header.stamp = ros::Time::now();
  // Publish marker array
  marker_pub_.publish(line_strip);
}

void flatland_ika_plugins::TrajectoryDrive::EgoMotionCallback(const definitions::IkaEgoMotion &msg) {
  this->current_ego_motion_ = msg;

  // Set steering angle of the visual model
  auto *front_right_wheel_joint = dynamic_cast<b2RevoluteJoint *>(tire_joints_[0]->physics_joint_);
  auto *front_left_wheel_joint = dynamic_cast<b2RevoluteJoint *>(tire_joints_[3]->physics_joint_);

  auto new_steering_angle = static_cast<float>(this->current_ego_motion_.fSteeringAngle * 2.0);
  front_right_wheel_joint->EnableLimit(true);
  front_right_wheel_joint->SetLimits(-new_steering_angle, -new_steering_angle);
  front_left_wheel_joint->EnableLimit(true);
  front_left_wheel_joint->SetLimits(-new_steering_angle, -new_steering_angle);
}

PLUGINLIB_EXPORT_CLASS(flatland_ika_plugins::TrajectoryDrive, // NOLINT(cert-err58-cpp,readability-container-size-empty)
                       flatland_server::ModelPlugin)
