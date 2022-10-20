/**
 * @author Simon Schaefer
 * @date 27.02.2020
 * @dfile SimulatedDrive.h
 */

#ifndef FLATLAND_PLUGINS_IKA_SIMULATED_DRIVE_H
#define FLATLAND_PLUGINS_IKA_SIMULATED_DRIVE_H

// C++ imports
#include <mutex>

// Eigen3 import
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Flatland imports
#include <Box2D/Box2D.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>

// Ros imports
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// Ika imports
#include <definitions/FlatlandVehicleState.h>

// Local imports
#include "SimulatedTire.h"

namespace flatland_ika_plugins {

/**
 * @brief A simulated vehicle for the box 2d simulation framework.
 * @details This vehicle is model for a top down 2D environment. The vehicle uses acceleration and yaw rate as input
 * and pushed velocity and yaw rate as outputs.
 */
class SimulatedDrive : public flatland_server::ModelPlugin {
 private:

  /**
   * @brief Four wheels that will simulate the driving behaviour on a force bases.
   * @details Stored in clockwise order with the front right tire.
   */
  SimulatedTire simulated_tires_[4];

  /**
   * @brief Flatland bodies of the tires. Represent the physical bodies in the simulation.
   * @details Will be managed by flatland. This pointer is only to get information from the simulation.
   */
  flatland_server::Body *tire_bodies_[4] = {nullptr, nullptr, nullptr, nullptr};

  /**
   * @brief Flatland joints the connect the tires to the vehicle. Represent the physical connections in the simulation.
   * @details Will be managed by flatland. This pointer is only to get information from the simulation.
   */
  flatland_server::Joint *tire_joints_[4] = {nullptr, nullptr, nullptr, nullptr};

  /**
   * @brief Short access to the front right wheel joint.
   * @details Joint of the physic simulation.
   */
  b2RevoluteJoint *b2_front_right_wheel_joint_ = nullptr;
  /**
   * @brief Short access to the front LEFT wheel joint.
   * @details Joint of the physic simulation.
   */
  b2RevoluteJoint *b2_front_left_wheel_joint_ = nullptr;
  /**
   * @brief Short access to the front right wheel body.
   * @details Body of the physic simulation.
   */
  b2Body *b2_front_right_wheel_body_ = nullptr;
  /**
   * @brief Short access to the front left wheel body.
   * @details Body of the physic simulation.
   */
  b2Body *b2_front_left_wheel_body_ = nullptr;
  /**
   * @brief Short access to the vehicle body.
   * @details Body of the physic simulation.
   */
  b2Body *b2_vehicle_body_ = nullptr;

  /**
   * @brief Flatland bodies of the tires. Represent the physical bodies in the simulation.
   * @details Will be managed by flatland. This pointer is only to get information from the simulation.
   */
  flatland_server::Body *vehicle_body_ = nullptr;

  /**
   * @brief Wheelbase of the vehicle.
   * @details Will be pares form the front right wheel joint.
   */
  double wheel_base_ = 0.0;

  /**
   * @brief Parameter that clamps the steering angle rate to a maximum value. [rad/s]
   * @details Will clamp the absolute change of the steering angle to a max value.
   */
  double maximum_steering_rate_ = 0.0;
  /**
   * @brief Parameter that clamps the steering angle to a maximum value. [rad]
   * @details Will clamp the absolute steering angle to a max value.
   */
  double maximum_steering_angle_ = 0.0;

  /**
   * @brief Parameter that clamps the forward velocity to a maximum value. [m/s]
   * @details Will clamp the forward velocity to a max value.
   */
  double maximum_forward_velocity_ = 0.0;
  /**
   * @brief Parameter that clamps the backward velocity to a maximum value. [m/s]
   * @details Will clamp the backward velocity to a max value. Value has to be positive.
   */
  double maximum_backward_velocity_ = 0.0;

  /**
   * @brief Parameter that clamps the acceleration to a maximum value. [m/s^2]
   * @details Will clamp the acceleration to a max value.
   */
  double maximum_front_acceleration_ = 0.0;
  /**
   * @brief Parameter that clamps the deceleration to a maximum value. [m/s^2]
   * @details Will clamp the deceleration to a max value. Value must be positive.
   */
  double maximum_front_deceleration_ = 0.0;

  /**
   * @brief Parameter that clamps the rear drive force to a maximum value. [N]
   * @details Will clamp the drive force of the rear tires to a max value.
   */
  double maximum_rear_acceleration_ = 0.0;
  /**
   * @brief Parameter that clamps the front drive force to a maximum value. [N]
   * @details Will clamp the drive force of the front tires to a max value.
   */
  double maximum_rear_deceleration_ = 0.0;

  /**
   * @brief Parameter that clamps the rear lateral impulse to a maximum value. [Ns]
   * @details Will clamp the lateral impulse of the rear tires to a max value.
   */
  double maximum_rear_lateral_impulse_ = 0.0;
  /**
   * @brief Parameter that clamps the front lateral impulse to a maximum value. [Ns]
   * @details Will clamp the lateral impulse of the front tires to a max value.
   */
  double maximum_front_lateral_impulse_ = 0.0;

  /**
   * @brief State to which this drive is wanted to converge.
   * @details Will be receiver from the controller.
   * @see target_state_subscriber_
   */
  definitions::FlatlandVehicleState target_state_;

  /**
   * @brief State where the drive is currently in.
   * @details Will be used to calculate the current actions.
   * @see target_state_
   * @see current_state_publisher_
   */
  definitions::FlatlandVehicleState current_state_;

  /**
   * @brief Subscribes to the new driving instructions.
   * @details Will update the target state.
   * @see target_state_
   */
  ros::Subscriber target_state_subscriber_;
  std::mutex target_state_mutex_;

  /**
   * @brief Publishes to the current driving behaviour.
   * @details Will send out the current state.
   * @see current_state_
   */
  ros::Publisher current_state_publisher_;

  /**
   * @breif Motions sensor data of the vehicle.
   * @details Not used for any calculation inside the model.
   * @see ground_truth_motion_data_publisher
   */
  nav_msgs::Odometry ground_truth_motion_data_;

  /**
   * @brief Publishes the current motions sensor data.
   * @see ground_truth_motion_data_
   */
  ros::Publisher ground_truth_motion_data_publisher_;

  /**
   * @brief Flatland timer the keeps track of the update rate of this model.
   * @detials A fixed update rate is used for both performance on low power machines and realtime capabilities.
   */
  flatland_plugins::UpdateTimer update_timer_;

  /**
   * @brief Stores the current steering angle. [rad]
   * @details This is used as a buffer so that value does not have to be read from the physics model.
   */
  double current_steering_angle_ = 0.0;

  /**
   * @brief Velocity from the last published message
   * @details Will be use to calculate the acceleration since the last time step.
   */
  definitions::FlatlandVehicleState old_state_;

  /**
   * @brief Will calculate the velocity vector as a local quantity.
   * @details All vector in the physics engine are defined in global coords.
   * @param body Physics body from that you want the velocity.
   * @return Velocity in local coords.
   */
  static Eigen::Vector2d GetVelocityVectorInLocalCoordinateSystem(b2Body *body);

  /**
   * @brief Will calculate the slipping angle of the front axis.
   * @details Will calculate the mean from both front joints.
   * @return Front slipping angle.
   */
  double CalculateFrontSlipAngle();

  /**
   * @brief Will turn the front tire to the target angle if possible.
   * @details Will only rotate as much as possible in one step. Defined by max rate and step time.
   * @param target_steering_angle Steering angle that wants to be archived.
   * @param step_size Time in seconds for the next step.
   */
  void TurnWheels(double target_steering_angle, double step_size);

  /**
   * @brief Will drive all front tires to the target linear velocity.
   * @details Will drive the wheels only in the physical boundaries defined by the max values.
   * @param desired_acceleration Target linear velocity.
   * @param step_size Time in seconds for the next step.
   */
  void DriveWheels(double desired_acceleration, double step_size);

 public:

  /**
   * @brief Initialize the model by parsing the model configuration.
   * @details Grabs model information from the physics model and initialises publisher and subscriber.
   * @param config Yaml configuration file.
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Pre simulation loop. Will be called before the physic model updates it data.
   * @param timekeeper Contains information of the current time step.
   */
  void BeforePhysicsStep(const flatland_server::Timekeeper &timekeeper) override;

  /**
   * @brief Post simulation loop. Will be called after the physic model updated data.
   * @details Will publish the current motion state.
   * @param timekeeper Contains information of the current time step.
   */
  void AfterPhysicsStep(const flatland_server::Timekeeper &timekeeper) override;

  /**
   * @brief Callback for the new driving instructions.
   * @details Will clamp all values to its corresponding max values.
   * @param msg New message.
   */
  void FlatlandVehicleStateCallback(const definitions::FlatlandVehicleState &msg);
  double CalculateSteeringAngleFromTargetYawRate();
};

}

#endif //FLATLAND_PLUGINS_IKA_SIMULATED_DRIVE_H
