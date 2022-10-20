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
 * @date 27.02.2020
 * @dfile SimulatedDrive.cpp
 */

#include "drives/SimulatedDrive.h"
#include <pluginlib/class_list_macros.h>

double clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

void flatland_ika_plugins::SimulatedDrive::OnInitialize(const YAML::Node &config) {
  // Open yaml reader
  flatland_server::YamlReader config_reader(config);

  // Load body names from the yaml file
  std::string vehicle_body_name = config_reader.Get<std::string>("vehicle_body");
  std::string front_left_tire_body_name = config_reader.Get<std::string>("front_left_tire_body");
  std::string front_right_tire_body_name = config_reader.Get<std::string>("front_right_tire_body");
  std::string rear_left_tire_body_name = config_reader.Get<std::string>("rear_left_tire_body");
  std::string rear_right_tire_body_name = config_reader.Get<std::string>("rear_right_tire_body");

  // Load joint names from the yaml file
  std::string front_left_tire_joint_name = config_reader.Get<std::string>("front_left_tire_joint");
  std::string front_right_tire_joint_name = config_reader.Get<std::string>("front_right_tire_joint");
  std::string rear_left_tire_joint_name = config_reader.Get<std::string>("rear_left_tire_joint");
  std::string rear_right_tire_joint_name = config_reader.Get<std::string>("rear_right_tire_joint");

  // Load odom frame id from the yaml file
  std::string odom_frame_id = config_reader.Get<std::string>("odom_frame_id", "odom");
  std::string state_frame_id = config_reader.Get<std::string>("state_frame_id", "vehicle_body");

  // Load topics from the yaml file
  std::string
      target_twist_topic = config_reader.Get<std::string>("target_twist_subscribe_topic", "vehicle/actuator_commands");
  std::string
      current_twist_topic = config_reader.Get<std::string>("current_twist_publish_topic", "vehicle/wheel_sensors");
  std::string ground_truth_topic = config_reader.Get<std::string>("ground_truth_pub", "odometry/ground_truth");

  // Load update rate from the yaml file
  auto update_rate = config_reader.Get<double>("update_rate", std::numeric_limits<double>::infinity());
  // Set update rate of the simulation
  update_timer_.SetRate(update_rate);

  // Load default vehicle parameter from yaml file
  maximum_steering_rate_ = config_reader.Get<double>("max_steering_angle_rate", maximum_steering_rate_);
  if (maximum_steering_rate_ < 0.0) {
    throw flatland_server::YAMLException(
        "Property " + flatland_server::Q("max_angular_velocity") + " must be positive");
  }
  maximum_steering_angle_ = config_reader.Get<double>("max_steering_angle", maximum_steering_angle_);
  if (maximum_steering_angle_ < 0.0) {
    throw flatland_server::YAMLException("Property " + flatland_server::Q("max_steering_angle") + " must be positive");
  }

  maximum_forward_velocity_ = config_reader.Get<double>("max_forward_velocity", maximum_forward_velocity_);
  if (maximum_forward_velocity_ < 0.0) {
    throw flatland_server::YAMLException(
        "Property " + flatland_server::Q("max_forward_velocity") + " must be positive");
  }
  maximum_backward_velocity_ = config_reader.Get<double>("max_backward_velocity", maximum_backward_velocity_);
  if (maximum_backward_velocity_ < 0.0) {
    throw flatland_server::YAMLException(
        "Property " + flatland_server::Q("max_backward_velocity") + " must be positive");
  }

  maximum_front_acceleration_ = config_reader.Get<double>("max_front_acceleration", maximum_front_acceleration_);
  if (maximum_front_acceleration_ < 0.0) {
    throw flatland_server::YAMLException("Property " + flatland_server::Q("max_front_acceleration") + " must be positive");
  }
  maximum_front_deceleration_ = config_reader.Get<double>("max_front_deceleration", maximum_front_deceleration_);
  if (maximum_front_deceleration_ < 0.0) {
    throw flatland_server::YAMLException("Property " + flatland_server::Q("max_front_deceleration") + " must be positive");
  }
  maximum_rear_acceleration_ = config_reader.Get<double>("max_rear_acceleration", maximum_rear_acceleration_);
  if (maximum_rear_acceleration_ < 0.0) {
    throw flatland_server::YAMLException(
        "Property " + flatland_server::Q("max_rear_drive_force") + " must be positive");
  }
  maximum_rear_deceleration_ = config_reader.Get<double>("max_rear_deceleration", maximum_rear_deceleration_);
  if (maximum_rear_deceleration_ < 0.0) {
    throw flatland_server::YAMLException(
        "Property " + flatland_server::Q("max_raer_deceleration") + " must be positive");
  }

  maximum_rear_lateral_impulse_ = config_reader.Get<double>("max_rear_lateral_impulse", maximum_rear_lateral_impulse_);
  if (maximum_rear_lateral_impulse_ < 0.0) {
    throw flatland_server::YAMLException(
        "Property " + flatland_server::Q("max_rear_lateral_impulse") + " must be positive");
  }
  maximum_front_lateral_impulse_ =
      config_reader.Get<double>("max_front_lateral_impulse", maximum_front_lateral_impulse_);
  if (maximum_front_lateral_impulse_ < 0.0) {
    throw flatland_server::YAMLException(
        "Property " + flatland_server::Q("max_front_lateral_impulse") + " must be positive");
  }

  // Make sure that all parameter where read
  config_reader.EnsureAccessedAllKeys();

  // Get bodies and joints from the simulation
  vehicle_body_ = GetModel()->GetBody(vehicle_body_name);
  if (vehicle_body_ == nullptr) {
    throw flatland_server::YAMLException("Body with name " + flatland_server::Q(vehicle_body_name) + " does not exist");
  }

  // Front right body
  tire_bodies_[0] = GetModel()->GetBody(front_right_tire_body_name);
  if (tire_bodies_[0] == nullptr) {
    throw flatland_server::YAMLException(
        "Body with name " + flatland_server::Q(front_right_tire_body_name) + " does not exist");
  }

  // Rear right body
  tire_bodies_[1] = GetModel()->GetBody(rear_right_tire_body_name);
  if (tire_bodies_[1] == nullptr) {
    throw flatland_server::YAMLException(
        "Body with name " + flatland_server::Q(rear_right_tire_body_name) + " does not exist");
  }

  // Rear left body
  tire_bodies_[2] = GetModel()->GetBody(rear_left_tire_body_name);
  if (tire_bodies_[2] == nullptr) {
    throw flatland_server::YAMLException(
        "Body with name " + flatland_server::Q(rear_left_tire_body_name) + " does not exist");
  }

  // Front left body
  tire_bodies_[3] = GetModel()->GetBody(front_left_tire_body_name);
  if (tire_bodies_[3] == nullptr) {
    throw flatland_server::YAMLException(
        "Body with name " + flatland_server::Q(front_left_tire_body_name) + " does not exist");
  }

  // Front right joint
  tire_joints_[0] = GetModel()->GetJoint(front_right_tire_joint_name);
  if (tire_joints_[0] == nullptr) {
    throw flatland_server::YAMLException(
        "Joint with name " + flatland_server::Q(front_right_tire_joint_name) + " does not exist");
  }

  // Rear right joint
  tire_joints_[1] = GetModel()->GetJoint(rear_right_tire_joint_name);
  if (tire_joints_[1] == nullptr) {
    throw flatland_server::YAMLException(
        "Joint with name " + flatland_server::Q(rear_right_tire_joint_name) + " does not exist");
  }

  // Rear left joint
  tire_joints_[2] = GetModel()->GetJoint(rear_left_tire_joint_name);
  if (tire_joints_[2] == nullptr) {
    throw flatland_server::YAMLException(
        "Joint with name " + flatland_server::Q(rear_left_tire_joint_name) + " does not exist");
  }

  // Front left joint
  tire_joints_[3] = GetModel()->GetJoint(front_left_tire_joint_name);
  if (tire_joints_[3] == nullptr) {
    throw flatland_server::YAMLException(
        "Joint with name " + flatland_server::Q(front_left_tire_joint_name) + " does not exist");
  }

  // Initialise tires with there driving characteristics
  simulated_tires_[0].Initialise(tire_bodies_[0]->physics_body_,
                                 maximum_front_acceleration_,
                                 maximum_front_deceleration_,
                                 std::max(maximum_forward_velocity_, maximum_backward_velocity_),
                                 maximum_front_lateral_impulse_);
  simulated_tires_[3].Initialise(tire_bodies_[3]->physics_body_,
                                 maximum_front_acceleration_,
                                 maximum_front_deceleration_,
                                 std::max(maximum_forward_velocity_, maximum_backward_velocity_),
                                 maximum_front_lateral_impulse_);
  simulated_tires_[1].Initialise(tire_bodies_[1]->physics_body_,
                                 maximum_rear_acceleration_,
                                 maximum_rear_deceleration_,
                                 std::max(maximum_forward_velocity_, maximum_backward_velocity_),
                                 maximum_front_lateral_impulse_);
  simulated_tires_[2].Initialise(tire_bodies_[2]->physics_body_,
                                 maximum_rear_acceleration_,
                                 maximum_rear_deceleration_,
                                 std::max(maximum_forward_velocity_, maximum_backward_velocity_),
                                 maximum_front_lateral_impulse_);

  // Get front tire joints and bodies from the physic simulation
  b2_front_right_wheel_joint_ = dynamic_cast<b2RevoluteJoint *>(tire_joints_[0]->physics_joint_);
  b2_front_left_wheel_joint_ = dynamic_cast<b2RevoluteJoint *>(tire_joints_[3]->physics_joint_);
  b2_front_right_wheel_body_ = tire_bodies_[0]->physics_body_;
  b2_front_left_wheel_body_ = tire_bodies_[3]->physics_body_;
  // Get vehicle body from the simulation
  b2_vehicle_body_ = vehicle_body_->physics_body_;

  // Calculate wheelbase front right joint
  b2Vec2 anchor = b2_front_right_wheel_body_->GetLocalPoint(b2_front_right_wheel_joint_->GetAnchorB());
  this->wheel_base_ = anchor.x;

  // Publish to and subscribe from topics
  target_state_subscriber_ = nh_.subscribe(target_twist_topic, 1, &SimulatedDrive::FlatlandVehicleStateCallback, this);
  current_state_publisher_ = nh_.advertise<definitions::FlatlandVehicleState>(current_twist_topic, 1);
  ground_truth_motion_data_publisher_ = nh_.advertise<nav_msgs::Odometry>(ground_truth_topic, 1);

  // Init the values for the messages
  ground_truth_motion_data_.header.frame_id = odom_frame_id;
  ground_truth_motion_data_.child_frame_id = tf::resolve("", GetModel()->NameSpaceTF(vehicle_body_->name_));
  ground_truth_motion_data_.twist.covariance.fill(0);
  ground_truth_motion_data_.pose.covariance.fill(0);

  current_state_.header.frame_id = state_frame_id;

  // Set the category bits to filter later on in the laser scanner
  for (auto *f = vehicle_body_->physics_body_->GetFixtureList(); f; f = f->GetNext()) {
    auto filter = f->GetFilterData();
    filter.categoryBits = 0x000F;
    f->SetFilterData(filter);
  }

  // Calculate vehicle mass
  //double mass = 0.0;
  //for (size_t i = 0; i < 4; i++) {
  //  mass += tire_bodies_[i]->physics_body_->GetMass();
  //}
  //mass += vehicle_body_->physics_body_->GetMass();
  //this->vehicle_mass_ = mass;
  //ROS_INFO("Mass: %f", mass);
}

void flatland_ika_plugins::SimulatedDrive::BeforePhysicsStep(const flatland_server::Timekeeper &timekeeper) {
  // Lock the target state so that it is not overwritten while calculations are running
  std::lock_guard<std::mutex> guard(this->target_state_mutex_);

  // Get the time of this integration [s]
  auto step_size = timekeeper.GetStepSize();
  // Adapt tire forces to match new situation
  DriveWheels(this->target_state_.acceleration, step_size);

  // Calculate steering angle

  // Possibility to drive calculate from a given yaw rate
  //double target_steering_angle = CalculateSteeringAngleFromTargetYawRate();

  double target_steering_angle = this->target_state_.steering_angle;
  // Adapt the joint that steer the vehicle
  TurnWheels(target_steering_angle, step_size);
}

Eigen::Vector2d flatland_ika_plugins::SimulatedDrive::GetVelocityVectorInLocalCoordinateSystem(b2Body *body) {
  // Get Angle of the body in global coords
  double body_angle = body->GetAngle();

  // Get velocity in global coords
  Eigen::Vector2d velocity = {body->GetLinearVelocity().x, body->GetLinearVelocity().y};
  // Allocate rotation matrix
  Eigen::Rotation2Dd rotation(-body_angle);
  // Rotate global velocity into local velocity
  velocity = rotation * velocity;

  return velocity;
}

double flatland_ika_plugins::SimulatedDrive::CalculateFrontSlipAngle() {
  // Get velocity from both front tires
  Eigen::Vector2d
    front_right_tire_velocity_vector = GetVelocityVectorInLocalCoordinateSystem(b2_front_right_wheel_body_);
  Eigen::Vector2d front_left_tire_velocity_vector = GetVelocityVectorInLocalCoordinateSystem(b2_front_left_wheel_body_);
  // Calculate mean
  Eigen::Vector2d
    front_tire_velocity_vector = 0.5 * (front_right_tire_velocity_vector + front_left_tire_velocity_vector);

  // Return slipping angle of the front mean
  return std::atan2(front_tire_velocity_vector.y(), front_tire_velocity_vector.x());
}

double flatland_ika_plugins::SimulatedDrive::CalculateSteeringAngleFromTargetYawRate(){
  // Current velocity
  Eigen::Vector2d current_linear_velocity_vector = GetVelocityVectorInLocalCoordinateSystem(this->b2_vehicle_body_);
  double current_linear_velocity = current_linear_velocity_vector.norm();

  // Calculate slip angles
  double front_slip_angle = CalculateFrontSlipAngle();
  double slip_angle = std::atan2(current_linear_velocity_vector.y(), current_linear_velocity_vector.x());

  // The factor of 0.6 results from the weight distribution between wheels and body
  double target_steering_angle =
    front_slip_angle - slip_angle + 0.5 * wheel_base_ * target_state_.yaw_rate / current_linear_velocity;
  target_steering_angle = clip(target_steering_angle, -this->maximum_steering_angle_, this->maximum_steering_angle_);
}

void flatland_ika_plugins::SimulatedDrive::DriveWheels(double desired_acceleration, double step_size) {
  // Loop over all tires
  for (auto &simulated_tire : simulated_tires_) {
    // Update tires
    simulated_tire.UpdateTire(desired_acceleration, step_size);
  }
}

void flatland_ika_plugins::SimulatedDrive::TurnWheels(double target_steering_angle, double step_size) {
  Eigen::Vector2d current_linear_velocity_vector = GetVelocityVectorInLocalCoordinateSystem(this->b2_vehicle_body_);
  double current_linear_velocity = current_linear_velocity_vector.norm();

  // Do not turn if the vehicle is not moving
  if (current_linear_velocity < 0.01) return;

  // Integrate max rate over time step
  double max_angle_step = maximum_steering_rate_ * step_size;

  // Check how much the tire has to be turned
  double angle_to_turn = target_steering_angle - current_steering_angle_;
  // Clip to max values
  angle_to_turn = clip(angle_to_turn, -max_angle_step, max_angle_step);

  // Calculate new steering angle
  this->current_steering_angle_ = (current_steering_angle_ + angle_to_turn);
  // Clip to max values
  this->current_steering_angle_ =
      clip(this->current_steering_angle_, -this->maximum_steering_angle_, this->maximum_steering_angle_);

  // Rotate front joints
  b2_front_right_wheel_joint_->EnableLimit(true);
  b2_front_right_wheel_joint_->SetLimits(current_steering_angle_, current_steering_angle_);
  b2_front_left_wheel_joint_->EnableLimit(true);
  b2_front_left_wheel_joint_->SetLimits(current_steering_angle_, current_steering_angle_);
}

void flatland_ika_plugins::SimulatedDrive::AfterPhysicsStep(const flatland_server::Timekeeper &timekeeper) {
  // Check if a new update has to be pushed, if not skip
  bool publish = update_timer_.CheckUpdate(timekeeper);
  if (!publish) return;

  // Get vehicle state
  b2Vec2 vehicle_position = b2_vehicle_body_->GetPosition();
  float vehicle_angle = b2_vehicle_body_->GetAngle();

  float vehicle_angular_velocity = -b2_vehicle_body_->GetAngularVelocity();
  b2Vec2 vehicle_linear_velocity_vector = b2_vehicle_body_->GetLinearVelocity();
  double vehicle_linear_velocity = vehicle_linear_velocity_vector.Length();
  if(GetVelocityVectorInLocalCoordinateSystem(this->b2_vehicle_body_)[0] < 0.0) vehicle_linear_velocity *= -1;

  // Push the state to the buffer
  ground_truth_motion_data_.header.stamp = timekeeper.GetSimTime();
  ground_truth_motion_data_.pose.pose.position.x = vehicle_position.x;
  ground_truth_motion_data_.pose.pose.position.y = vehicle_position.y;
  ground_truth_motion_data_.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(vehicle_angle);
  ground_truth_motion_data_.twist.twist.linear.x = vehicle_linear_velocity_vector.x;
  ground_truth_motion_data_.twist.twist.linear.y = vehicle_linear_velocity_vector.y;
  ground_truth_motion_data_.twist.twist.angular.z = vehicle_angular_velocity;

  // Push data to ros network
  ground_truth_motion_data_publisher_.publish(ground_truth_motion_data_);

  double acceleration = (vehicle_linear_velocity - this->old_state_.velocity) / (timekeeper.GetSimTime()
      - this->old_state_.header.stamp).toSec();

  // Push data to buffer
  current_state_.velocity = vehicle_linear_velocity;
  current_state_.acceleration = acceleration;
  current_state_.yaw_rate = vehicle_angular_velocity;
  current_state_.steering_angle = this->current_steering_angle_;

  current_state_.header.stamp = timekeeper.GetSimTime();
  //current_state_.header.seq = sequence;
  this->old_state_ = this->current_state_;

  // Push to ros network
  current_state_publisher_.publish(current_state_);
}

void flatland_ika_plugins::SimulatedDrive::FlatlandVehicleStateCallback(const definitions::FlatlandVehicleState &msg) {
  // Lock the target state so that it is not overwritten during calculations
  std::lock_guard<std::mutex> guard(this->target_state_mutex_);
  // copy new target state
  this->target_state_ = msg;
}

PLUGINLIB_EXPORT_CLASS(flatland_ika_plugins::SimulatedDrive, // NOLINT(cert-err58-cpp,readability-container-size-empty)
                       flatland_server::ModelPlugin
)
