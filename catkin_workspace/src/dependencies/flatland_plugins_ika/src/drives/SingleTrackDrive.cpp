/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	tricycle_drive.cpp
 * @brief   tricycle plugin
 * @author  Mike Brousseau, Chunshang Li
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Avidbots Corp.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Avidbots Corp. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "drives/SingleTrackDrive.h"

#include <flatland_server/model_plugin.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/tf.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
namespace flatland_ika_plugins {

void SingleTrackDrive::OnInitialize(const YAML::Node& config) {
  flatland_server::YamlReader r(config);

  // load all the parameters
  std::string body_name = r.Get<std::string>("body");
  std::string front_wj_name = r.Get<std::string>("front_wheel_joint");
  std::string rear_left_wj_name = r.Get<std::string>("rear_left_wheel_joint");
  std::string rear_right_wj_name = r.Get<std::string>("rear_right_wheel_joint");
  std::string odom_frame_id = r.Get<std::string>("odom_frame_id", "odom");

  std::string target_twist_topic = r.Get<std::string>("target_twist_sub", "cmd_vel");
  std::string current_twist_topic = r.Get<std::string>("current_twist_pub", "cmd_vel");
  std::string odom_topic = r.Get<std::string>("odom_pub", "odometry/filtered");
  std::string ground_truth_topic =
      r.Get<std::string>("ground_truth_pub", "odometry/ground_truth");

  // noise are in the form of linear x, linear y, angular variances
  std::vector<double> odom_twist_noise =
      r.GetList<double>("odom_twist_noise", {0, 0, 0}, 3, 3);
  std::vector<double> odom_pose_noise =
      r.GetList<double>("odom_pose_noise", {0, 0, 0}, 3, 3);

  auto pub_rate = r.Get<double>("pub_rate", std::numeric_limits<double>::infinity());
  update_timer_.SetRate(pub_rate);

  // by default the covariance diagonal is the variance of actual noise
  // generated, non-diagonal elements are zero assuming the noises are
  // independent, we also don't care about linear z, angular x, and angular y
  std::array<double, 36> odom_pose_covar_default = {0};
  odom_pose_covar_default[0] = odom_pose_noise[0];
  odom_pose_covar_default[7] = odom_pose_noise[1];
  odom_pose_covar_default[35] = odom_pose_noise[2];

  std::array<double, 36> odom_twist_covar_default = {0};
  odom_twist_covar_default[0] = odom_twist_noise[0];
  odom_twist_covar_default[7] = odom_twist_noise[1];
  odom_twist_covar_default[35] = odom_twist_noise[2];

  auto odom_twist_covar =
      r.GetArray<double, 36>("odom_twist_covariance", odom_twist_covar_default);
  auto odom_pose_covar =
      r.GetArray<double, 36>("odom_pose_covariance", odom_pose_covar_default);

  // Default max_angular_velocity=0 means "unbounded"
  max_angular_velocity_ = r.Get<double>("max_angular_velocity", 0.0);
  max_steering_angle_ = r.Get<double>("max_steering_angle", 0.0);
  target_wheel_angle_ = 0.0;
  theta_f_ = 0.0;

  // Default max_linear_acceleration_=0 means "unbounded"
  max_forward_velocity_ = r.Get<double>("max_forward_velocity", 0.0);
  max_backward_velocity_ = r.Get<double>("max_backward_velocity", 0.0);
  max_linear_acceleration_ = r.Get<double>("max_linear_acceleration", 0.0);
  max_linear_deceleration_ = r.Get<double>("max_linear_deceleration", 0.0);

  r.EnsureAccessedAllKeys();

  // Get the bodies and joints from names, throw if not found
  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw flatland_server::YAMLException("Body with name " + flatland_server::Q(body_name) + " does not exist");
  }

  front_wj_ = GetModel()->GetJoint(front_wj_name);
  if (front_wj_ == nullptr) {
    throw flatland_server::YAMLException("Joint with name " + flatland_server::Q(front_wj_name) +
                        " does not exist");
  }

  rear_left_wj_ = GetModel()->GetJoint(rear_left_wj_name);
  if (rear_left_wj_ == nullptr) {
    throw flatland_server::YAMLException("Joint with name " + flatland_server::Q(rear_left_wj_name) +
                        " does not exist");
  }

  rear_right_wj_ = GetModel()->GetJoint(rear_right_wj_name);
  if (rear_right_wj_ == nullptr) {
    throw flatland_server::YAMLException("Joint with name " + flatland_server::Q(rear_right_wj_name) +
                        " does not exist");
  }

  // validate the that joints fits the assumption of the robot model and
  // calculate rear wheel separation and wheel base
  ComputeJoints();

  // publish and subscribe to topics
  target_twist_sub_ =
      nh_.subscribe(target_twist_topic, 1, &SingleTrackDrive::TwistCallback, this);
  current_twist_pub_ = nh_.advertise<geometry_msgs::Twist>(current_twist_topic, 1);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 1);
  ground_truth_pub_ = nh_.advertise<nav_msgs::Odometry>(ground_truth_topic, 1);

  // init the values for the messages
  ground_truth_msg_.header.frame_id = odom_frame_id;
  ground_truth_msg_.child_frame_id =
      tf::resolve("", GetModel()->NameSpaceTF(body_->name_));

  ground_truth_msg_.twist.covariance.fill(0);
  ground_truth_msg_.pose.covariance.fill(0);
  odom_msg_ = ground_truth_msg_;

  // copy from array to boost array
  for (unsigned int i = 0; i < 36; i++) {
    odom_msg_.twist.covariance[i] = odom_twist_covar[i];
    odom_msg_.pose.covariance[i] = odom_pose_covar[i];
  }

  // init the random number generators
  std::random_device rd;
  rng_ = std::default_random_engine(rd());
  for (unsigned int i = 0; i < 3; i++) {
    // variance is standard deviation squared
    noise_gen_[i] = std::normal_distribution<double>(0.0, sqrt(odom_pose_noise[i]));
  }

  for (unsigned int i = 0; i < 3; i++) {
    noise_gen_[i + 3] =
        std::normal_distribution<double>(0.0, sqrt(odom_twist_noise[i]));
  }

  ROS_DEBUG_NAMED(
      "SingleTrackDrive",
      "Initialized with params body(%p %s) front_wj(%p %s) "
      "rear_left_wj(%p %s) rear_right_wj(%p %s) "
      "odom_frame_id(%s) target_twist_sub(%s) odom_pub(%s) "
      "ground_truth_pub(%s) odom_pose_noise({%f,%f,%f}) "
      "odom_twist_noise({%f,%f,%f}) pub_rate(%f)\n",
      body_, body_->GetName().c_str(), front_wj_, front_wj_->GetName().c_str(),
      rear_left_wj_, rear_left_wj_->GetName().c_str(), rear_right_wj_,
      rear_right_wj_->GetName().c_str(), odom_frame_id.c_str(),
      target_twist_topic.c_str(), odom_topic.c_str(), ground_truth_topic.c_str(),
      odom_pose_noise[0], odom_pose_noise[1], odom_pose_noise[2],
      odom_twist_noise[0], odom_twist_noise[1], odom_twist_noise[2], pub_rate);
}

void SingleTrackDrive::ComputeJoints() {
  auto get_anchor = [&](flatland_server::Joint* joint, bool* is_inverted = nullptr) {

    b2Vec2 wheel_anchor;  ///< wheel anchor point, must be (0,0)
    b2Vec2 body_anchor;   ///< body anchor point
    bool inv = false;

    // ensure one of the body is the main body for the odometry
    if (joint->physics_joint_->GetBodyA()->GetUserData() == body_) {
      wheel_anchor = joint->physics_joint_->GetAnchorB();
      body_anchor = joint->physics_joint_->GetAnchorA();
    } else if (joint->physics_joint_->GetBodyB()->GetUserData() == body_) {
      wheel_anchor = joint->physics_joint_->GetAnchorA();
      body_anchor = joint->physics_joint_->GetAnchorB();
      inv = true;
    } else {
      throw flatland_server::YAMLException("Joint " + flatland_server::Q(joint->GetName()) +
                          " does not anchor on body " + flatland_server::Q(body_->GetName()));
    }

    // convert anchor is global coordinates to local body coordinates
    wheel_anchor = body_->physics_body_->GetLocalPoint(wheel_anchor);
    body_anchor = body_->physics_body_->GetLocalPoint(body_anchor);

    // ensure the joint is anchored at (0,0) of the wheel_body
    if (fabs(wheel_anchor.x) > 1e-5 || fabs(wheel_anchor.y) > 1e-5) {
      throw flatland_server::YAMLException("Joint " + flatland_server::Q(joint->GetName()) +
                          " must be anchored at (0, 0) on the wheel");
    }

    if (is_inverted) {
      *is_inverted = inv;
    }

    return body_anchor;
  };

  // joints must be of expected type
  if (front_wj_->physics_joint_->GetType() != e_revoluteJoint) {
    throw flatland_server::YAMLException("Front wheel joint must be a revolute joint");
  }

  if (rear_left_wj_->physics_joint_->GetType() != e_weldJoint) {
    throw flatland_server::YAMLException("Rear left wheel joint must be a weld joint");
  }

  if (rear_right_wj_->physics_joint_->GetType() != e_weldJoint) {
    throw flatland_server::YAMLException("Rear right wheel joint must be a weld joint");
  }

  // enable limits for the front joint
  auto* j = dynamic_cast<b2RevoluteJoint*>(front_wj_->physics_joint_);
  j->EnableLimit(true);

  // positive joint angle goes counter clockwise from the perspective of BodyA,
  // if body_ is not BodyA, we need flip the steering angle for visualization
  b2Vec2 front_anchor = get_anchor(front_wj_, &invert_steering_angle_);
  b2Vec2 rear_left_anchor = get_anchor(rear_left_wj_);
  b2Vec2 rear_right_anchor = get_anchor(rear_right_wj_);

  // the front wheel must be at (0,0) of the body
  if (fabs(front_anchor.x) > 1e-5 || fabs(front_anchor.y) > 1e-5) {
    throw flatland_server::YAMLException(
        "Front wheel joint must have its body anchored at (0, 0)");
  }

  // calculate the wheelbase and axeltrack. We also need to verify that
  // the rear_center is at the perpendicular intersection between the rear axel
  // and the front wheel anchor
  rear_center_ = 0.5 * (rear_left_anchor + rear_right_anchor);

  // find the perpendicular intersection between line segment given by (x1, y1)
  // and (x2, y2) and a point (x3, y3).
  double x1 = rear_left_anchor.x, y1 = rear_left_anchor.y,
         x2 = rear_right_anchor.x, y2 = rear_right_anchor.y,
         x3 = front_anchor.x, y3 = front_anchor.y;

  double k = ((y2 - y1) * (x3 - x1) - (x2 - x1) * (y3 - y1)) /
             ((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
  double x4 = x3 - k * (y2 - y1);
  double y4 = y3 + k * (x2 - x1);

  // check (x4, y4) equals to rear_center_
  if (fabs(x4 - rear_center_.x) > 1e-5 || fabs(y4 - rear_center_.y) > 1e-5) {
    throw flatland_server::YAMLException(
        "The mid point between the rear wheel anchors on the body must equal "
        "the perpendicular intersection between the rear axel (line segment "
        "between rear anchors) and the front wheel anchor");
  }

  // track is the separation between the rear two wheels, which is simply the
  // distance between the rear two wheels
  axel_track_ = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

  // wheel base is the perpendicular distance between the rear axel and the
  // front wheel
  wheelbase_ = sqrt((x4 - x3) * (x4 - x3) + (y4 - y3) * (y4 - y3));
}

void SingleTrackDrive::BeforePhysicsStep(const flatland_server::Timekeeper& timekeeper) {
  bool publish = update_timer_.CheckUpdate(timekeeper);

  b2Body* b2body = body_->physics_body_;

  b2Vec2 position = b2body->GetPosition();
  float angle = b2body->GetAngle();

  // get the state of the body and publish the data
  float angular_vel = b2body->GetAngularVelocity();
  b2Vec2 linear_vel_local =
      b2body->GetLinearVelocityFromLocalPoint(b2Vec2(0, 0));

  double speed = std::sqrt(std::pow(linear_vel_local.x,2) + std::pow(linear_vel_local.y,2));

  if (publish) {
    ground_truth_msg_.header.stamp = timekeeper.GetSimTime();
    ground_truth_msg_.pose.pose.position.x = position.x;
    ground_truth_msg_.pose.pose.position.y = position.y;
    ground_truth_msg_.pose.pose.position.z = 0;
    ground_truth_msg_.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(angle);
    ground_truth_msg_.twist.twist.linear.x = linear_vel_local.x;
    ground_truth_msg_.twist.twist.linear.y = linear_vel_local.y;
    ground_truth_msg_.twist.twist.linear.z = 0;
    ground_truth_msg_.twist.twist.angular.x = 0;
    ground_truth_msg_.twist.twist.angular.y = 0;
    ground_truth_msg_.twist.twist.angular.z = angular_vel;

    // add the noise to odom messages
    odom_msg_.header.stamp = timekeeper.GetSimTime();
    odom_msg_.pose.pose = ground_truth_msg_.pose.pose;
    odom_msg_.twist.twist = ground_truth_msg_.twist.twist;
    odom_msg_.pose.pose.position.x += noise_gen_[0](rng_);
    odom_msg_.pose.pose.position.y += noise_gen_[1](rng_);
    odom_msg_.pose.pose.orientation =
        tf::createQuaternionMsgFromYaw(angle + noise_gen_[2](rng_));
    odom_msg_.twist.twist.linear.x += noise_gen_[3](rng_);
    odom_msg_.twist.twist.linear.y += noise_gen_[4](rng_);
    odom_msg_.twist.twist.angular.z += noise_gen_[5](rng_);

    ground_truth_pub_.publish(ground_truth_msg_);
    odom_pub_.publish(odom_msg_);
  }

  // twist message contains the speed and angle of the front wheel
  double v_f = target_twist_msg_.linear.x;            // velocity at front wheel
  target_wheel_angle_ = target_twist_msg_.angular.z;  // front wheel steering angle
  double theta = angle;                        // angle of the robot

  if (max_angular_velocity_ == 0.0) {  // Infinite angular velocity
    theta_f_ = target_wheel_angle_;
  } else {  // If angular velocity is bounded, bound it
    double max_angle_step = max_angular_velocity_ * timekeeper.GetStepSize();

    if (target_wheel_angle_ > theta_f_) {
      theta_f_ +=
          std::min<double>(max_angle_step, target_wheel_angle_ - theta_f_);
    } else {
      theta_f_ -=
          std::min<double>(max_angle_step, theta_f_ - target_wheel_angle_);
    }
  }

  if(max_steering_angle_ != 0.0){
    if(theta_f_ > max_steering_angle_){
      theta_f_ = max_steering_angle_;
    }else if(theta_f_ < -max_steering_angle_){
      theta_f_ = -max_steering_angle_;
    }
  }

  if (max_linear_acceleration_ != 0.0) {
    double max_vel_step = max_linear_acceleration_ * timekeeper.GetStepSize();
    if (v_f > speed)
      v_f = speed + std::min<double>(max_vel_step, v_f - speed);
  }

  if (max_linear_acceleration_ != 0.0) {
    double max_vel_step = max_linear_deceleration_ * timekeeper.GetStepSize();
    if (v_f < speed)
      v_f = speed - std::min<double>(max_vel_step, speed - v_f);
  }

  if (max_forward_velocity_ != 0.0) {
      v_f = std::min<double>(max_forward_velocity_, v_f);
  }
  if (max_backward_velocity_ != 0.0) {
    v_f = std::max<double>(-max_backward_velocity_, v_f);
  }
  // change angle of the front wheel for visualization

  auto* j = dynamic_cast<b2RevoluteJoint*>(front_wj_->physics_joint_);
  j->EnableLimit(true);
  if (invert_steering_angle_) {
    j->SetLimits(-theta_f_, -theta_f_);
  } else {
    j->SetLimits(theta_f_, theta_f_);
  }

  // calculate the desired velocity using the bicycle model in the world frame
  // looking at the rear center, formulas obtained from avidbots robot systems
  // confluence page
  double v_x = v_f * cos(theta_f_) * cos(theta);  // x velocity in world
  double v_y = v_f * cos(theta_f_) * sin(theta);  // y velocity in world
  double w = v_f * sin(theta_f_) / wheelbase_;    // angular velocity

  // Now we would like the rear center to move at v_x, v_y, and w, since Box2D
  // applies velocities at center of mass, we must use rigid body kinematics
  // to transform the velocities
  b2Vec2 linear_vel(v_x, v_y);

  // V_cm = V_rc + W x r_cm/rc
  // velocity at center of mass equals to the velocity at the rear center plus,
  // angular velocity cross product the displacement from the rear center to the
  // center of mass

  // r is the vector from rear center to CM in world frame
  b2Vec2 r = b2body->GetWorldCenter() - b2body->GetWorldPoint(rear_center_);
  b2Vec2 linear_vel_cm = linear_vel + w * b2Vec2(-r.y, r.x);

  b2body->SetLinearVelocity(linear_vel_cm);

  // angular velocity is the same at any point in body
  b2body->SetAngularVelocity(w);

if (publish) {
    current_twist_msg_.linear.x = v_f;
    current_twist_msg_.angular.z = theta_f_;
    current_twist_pub_.publish(current_twist_msg_);
  }

}

void SingleTrackDrive::TwistCallback(const geometry_msgs::Twist& msg) {
  target_twist_msg_ = msg;
}
}

PLUGINLIB_EXPORT_CLASS(flatland_ika_plugins::SingleTrackDrive, // NOLINT(cert-err58-cpp,readability-container-size-empty)
                       flatland_server::ModelPlugin)
