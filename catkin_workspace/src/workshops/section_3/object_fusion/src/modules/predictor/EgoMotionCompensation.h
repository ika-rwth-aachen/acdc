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

/** @addtogroup FUSION */
/**@{*/

#pragma once

#include "modules/AbstractFusionModule.h"

/**
 * @authors Michael Hoss, Simon Schaefer
 * @brief Transform the pose of global objects due to the motion of the ego-vehicle.
 * @details Compensates the ego motion that was made in between latest global object stamp and current measurement.
 */
class EgoMotionCompensation : public AbstractFusionModule {
 private:
  /**
   * @brief Assumes circular movement of ego vehicle with constant velocity and constant yaw rate
   * @param ego_motion Latest available ego motion
   * @param delta_time_in_seconds
   * @return delta_x, delta_y, delta_heading of ego vehicle
   * @todo possibly apply a more adavanced temporal integration scheme
   */
  static Eigen::Vector3f calculateEgoPositionHeadingChange(
      const definitions::IkaEgoMotion *ego_motion,
      float delta_time_in_seconds);

 public:
  EgoMotionCompensation(std::shared_ptr<Data> data, std::string name);
  void runSingleSensor() override;
};

/**@}*/
