/**
 * @author Simon Schaefer
 * @date 27.02.2020
 * @dfile TrajectoryDrive.h
 */

#ifndef FLATLAND_PLUGINS_IKA_TRAJECTORY_DRIVE_H
#define FLATLAND_PLUGINS_IKA_TRAJECTORY_DRIVE_H

// Flatland
#include <Box2D/Box2D.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>

// ROS
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

// Ika
#include <definitions/IkaGPS.h>
#include <definitions/IkaEgoMotion.h>

namespace flatland_ika_plugins {

class TrajectoryDrive : public flatland_server::ModelPlugin {
 private:

  /**
   * @brief Flatland bodies of the tires. Represent the physical bodies in the simulation.
   * @details Will be managed by flatland. This pointer is only to get information from the simulation.
   */
  flatland_server::Body *tire_bodies_[4] = {nullptr, nullptr, nullptr, nullptr};;
  /**
   * @brief Flatland joints to connect the tires to the vehicle. Represent the physical connections in the simulation.
   * @details Will be managed by flatland. This pointer is only to get information from the simulation.
   */
  flatland_server::Joint *tire_joints_[4] = {nullptr, nullptr, nullptr, nullptr};
  /**
   * @brief Short access to the vehicle body.
   * @details Body of the physics simulation.
   */
  flatland_server::Body *vehicle_body_ = nullptr;

  /**
   * @brief Subscriber for the recorded vehicle state.
   */
  ros::Subscriber target_gps_sub_;
  /**
   * @brief Subscriber for the recorded steering angle.
   */
  ros::Subscriber target_ego_sub_;

  /**
   * @brief Current steering angle.
   */
  definitions::IkaEgoMotion current_ego_motion_;
  /**
   * @brief Current vehicle state.
   */
  definitions::IkaGPS current_gps_;

  /**
   * @brief Flag to check if the gps was referenced already.
   */
  bool gps_referenced = false;
  /**
   * @brief Reference vehicle position.
   */
  definitions::IkaGPS reference_gps_;

  /**
   * @brief Marker for visualisation of the gps positions.
   */
  visualization_msgs::Marker line_strip;
  /**
   * @brief Publisher of the visual marker.
   */
  ros::Publisher marker_pub_;

  /**
   * @brief Current vehicle state in a generic ros message.
   */
  nav_msgs::Odometry odom_msg_;
  /**
   * @brief Publisher for the state in generic form.
   */
  ros::Publisher odom_pub_;

  /**
   * @brief Flatland timer that keeps track of the update rate of this model.
   * @detials A fixed update rate is used for both performance on low power machines and realtime capabilities.
   */
  flatland_plugins::UpdateTimer update_timer_;


 public:

  /**
   * @brief Initialize the model by parsing the model configuration.
   * @details Grabs model information from the physics model and initialises publisher and subscriber.
   * @param config Yaml configuration file.
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Post simulation loop. Will be called after the physic model updated data.
   * @details Will publish the current motion state.
   * @param timekeeper Contains information of the current time step.
   */
  void AfterPhysicsStep(const flatland_server::Timekeeper &timekeeper) override;

  /**
   * @brief Callback for the new vehicle state recorded from a gps.
   * @param msg New vehicle state.
   */
  void GpsCallback(const definitions::IkaGPS &msg);

  /**
   * @brief Callback for the new steering angle recorded from the vehicle can.
   * @param msg New steering angle.
   */
  void EgoMotionCallback(const definitions::IkaEgoMotion &msg);
};

}

#endif //FLATLAND_PLUGINS_IKA_TRAJECTORY_DRIVE_H
