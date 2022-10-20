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
