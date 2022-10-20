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

#include "EgoMotionCompensation.h"

#include <cmath>
#include <Eigen/Geometry>
#include <definitions/utility/ika_utilities.h>

	
EgoMotionCompensation::EgoMotionCompensation(std::shared_ptr<Data> data, std::string name)
    : AbstractFusionModule(data, name) {}

void EgoMotionCompensation::runSingleSensor()
{
  Eigen::Vector3f delta_ego_motion =
      calculateEgoPositionHeadingChange(&data_->ego_motion, data_->prediction_gap_in_seconds);

  Eigen::Rotation2Df rotateAgainstDeltaYaw(-delta_ego_motion(2));

  for (auto &object : data_->object_list_fused.objects) { // iterate through object list 
    if (!object.bObjectValid) continue;

    auto position = IkaUtilities::getObjectPosition(object);
    auto velocity = IkaUtilities::getObjectVelocity(object);
    auto acceleration = IkaUtilities::getObjectAcceleration(object);
    Eigen::Vector2f objPos(position[0], position[1]);
    Eigen::Vector2f objVel(velocity[0], velocity[1]);
    Eigen::Vector2f objAcc(acceleration[0], acceleration[1]);

    // rotations   
    auto rotation_matrix = rotateAgainstDeltaYaw.toRotationMatrix();
    objPos = rotation_matrix * objPos;
    objVel = rotation_matrix * objVel;
    objAcc = rotation_matrix * objAcc;
    IkaUtilities::setObjectVelocity(object, objVel[0], objVel[1]);
    IkaUtilities::setObjectAcceleration(object, objAcc[0], objAcc[1]);
    IkaUtilities::setObjectHeading(object, IkaUtilities::getObjectHeading(object) - delta_ego_motion(2));

    // translations
    // use here my position, don't get it again, as not updated yet 
    float newPosX = objPos[0] - delta_ego_motion(0);
    float newPosY = objPos[1] - delta_ego_motion(1);
    IkaUtilities::setObjectPositionXY(object, newPosX, newPosY);


    // reset heading angle to (-PI/2, PI/2]
    while (IkaUtilities::getObjectHeading(object) <= -M_PI_2f32)   
    {
      IkaUtilities::setObjectHeading(object, IkaUtilities::getObjectHeading(object) + M_PIf32);
    }
    while (IkaUtilities::getObjectHeading(object) > M_PI_2f32)
    {
      IkaUtilities::setObjectHeading(object, IkaUtilities::getObjectHeading(object) - M_PIf32);
    }
  }
}


Eigen::Vector3f EgoMotionCompensation::calculateEgoPositionHeadingChange(
    const definitions::IkaEgoMotion *ego_motion,
    float delta_time_in_seconds) {
  float yaw_rate = ego_motion->fYawRate;
  float velocity = ego_motion->fVelocity;
  float delta_theta = yaw_rate * delta_time_in_seconds;
  float delta_x = 0.0f;
  float delta_y = 0.0f;

  if(yaw_rate > 1.0e-4f) {
    float radius = velocity / yaw_rate;
    delta_x = radius * std::sin(delta_theta);
    delta_y = radius * (1.0f - std::cos(delta_theta));
  }else{
    float radius = velocity * delta_time_in_seconds;
    delta_x = radius * std::cos(delta_theta);
    delta_y = radius * std::sin(delta_theta);
  }

  Eigen::Vector3f result;
  result(0) = delta_x;
  result(1) = delta_y;
  result(2) = delta_theta;

  return result;
}
