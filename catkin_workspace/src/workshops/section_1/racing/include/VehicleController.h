/**
 * @author Simon Schaefer, Michael Hoss
 * @file VehicleController.h
 */

/**
 * @brief Controller that computes target vehicle behavior (actuator commands)
 * based on a Lidar scan
 * @details This class does not depend on ROS such that it could be used in
 * another framework as well. This increases code re-usability.
 */
class VehicleController {
 private:

  // member variables
  float sensor_distances_[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; ///< order: right, front, left
  double target_velocity_ = 0.0f; ///< positive for moving forward
  double target_steering_angle_ = 0.0f; ///< positive counterclockwise ("to the left")

  // internal functions
  void computeTargetVelocity(); ///< based on measured front lidar distance
  void computeTargetSteeringAngle(); ///< based on measured left and right lidar distances

 public:

  // interface functions to be called from e.g. a wrapping ROS node

  /**
   * @brief input accessor to overwrite the internally stored Lidar distances
   * @param distances order: right, front, left.
   */
  void overwriteLidarDistances(const float distances[5]);
  void computeTargetValues(); ///< compute target values (actuator commands) based on given sensor data
  double getTargetVelocity(); ///< output accessor
  double getTargetSteeringAngle(); ///< output accessor

};
