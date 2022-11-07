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

#include "VehicleController.h"

#include <cstdlib>
#include <cmath>

void VehicleController::computeTargetVelocity() {
  // START CODE (improve the longitudinal control)

  const float FACTOR = 2.0f;
  const float FACTOR2 = 8.0f;
  const float& front_distance = this->sensor_distances_[2];

  // The larger the free space in the front, the greater the velocity can be.
  // The tanh function - tuned with parameters - is useful to implement such behavior smoothly.
  this->target_velocity_ = front_distance * FACTOR * std::tanh(front_distance * FACTOR2);

  // END CODE
}

void VehicleController::computeTargetSteeringAngle() {
  // START CODE (improve the lateral control)

  const float FACTOR = 0.5f;

  // Calculate relative lateral position on the road
  // You can use up to 5 lidar points for localization
  const float& right_distance = this->sensor_distances_[0];
  const float& front_distance = this->sensor_distances_[2];
  const float& left_distance = this->sensor_distances_[4];
  float rightShift = left_distance - right_distance;

  // The closer the cart is to the right border, the more it should steer to the left
  // (and vice versa).
  // (Steering to the left = positive steering angle)
  // To achieve this in a smooth way, a parameter-tuned tanh function can be used.
  this->target_steering_angle_ = (FACTOR/front_distance) * rightShift * std::tanh( std::abs(rightShift));
  
  // END CODE
}

void VehicleController::overwriteLidarDistances(const float distances[5]) {
  for(size_t i = 0; i<5; i++){
    this->sensor_distances_[i] = distances[i];
  }
}

void VehicleController::computeTargetValues() {
  computeTargetVelocity();
  computeTargetSteeringAngle();
}

double VehicleController::getTargetVelocity() {
  return this->target_velocity_;
}

double VehicleController::getTargetSteeringAngle() {
  return this->target_steering_angle_;
}





