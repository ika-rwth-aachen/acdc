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

#include "drives/SimulatedTire.h"

void SimulatedTire::Initialise(b2Body *tire_physics_body,
                               double maximum_acceleration,
                               double maximum_deceleration,
                               double maximum_velocity,
                               double maximum_lateral_impulse) {
  // Pointer that allows a cast from a b2Body* to a SimulatedTire object.
  this->tire_physics_body_ = tire_physics_body;
  //this->tire_physics_body_->SetUserData(this);

  // Parameter
  this->maximum_acceleration_ = maximum_acceleration;
  this->maximum_deceleration_ = maximum_deceleration;
  this->maximum_velocity_ = maximum_velocity;
  this->maximum_lateral_impulse_ = maximum_lateral_impulse;
}

b2Vec2 SimulatedTire::GetLateralVelocity() {
  assert(this->tire_physics_body_);

  b2Vec2 right_side_normal = tire_physics_body_->GetWorldVector(b2Vec2(0, 1));
  return b2Dot(right_side_normal, tire_physics_body_->GetLinearVelocity()) * right_side_normal;
}

b2Vec2 SimulatedTire::GetForwardVelocity() {
  assert(this->tire_physics_body_);

  b2Vec2 forward_normal = tire_physics_body_->GetWorldVector(b2Vec2(1, 0));
  return b2Dot(forward_normal, tire_physics_body_->GetLinearVelocity()) * forward_normal;
}

void SimulatedTire::UpdateTire(double desired_acceleration, double step_size) {
  UpdateDragAndFrictionLoss();
  UpdateDriveForces(desired_acceleration, step_size);
}

void SimulatedTire::UpdateDragAndFrictionLoss() {
  assert(this->tire_physics_body_);

  // Cancel some of the angular velocity via friction
  double angular_impulse =
      -0.1 * traction_multiplier_ * tire_physics_body_->GetInertia() * tire_physics_body_->GetAngularVelocity();
  tire_physics_body_->ApplyAngularImpulse(angular_impulse, true);

  //Cancel out some of the linear velocity via drag
  b2Vec2 current_forward_velocity = GetForwardVelocity();
  double current_forward_speed = current_forward_velocity.Normalize();
  double drag_force_multiplier = -0.25 * traction_multiplier_ * drag_multiplier_ * current_forward_speed;

  b2Vec2 force = drag_force_multiplier * current_forward_velocity;
  tire_physics_body_->ApplyForceToCenter(force, true);
}

void SimulatedTire::UpdateDriveForces(double desired_acceleration, double step_size) {
  assert(this->tire_physics_body_);

  // Clamp desired speed to the max udn min values.
  desired_acceleration = std::min(desired_acceleration, maximum_acceleration_);
  desired_acceleration = std::max(desired_acceleration, -maximum_deceleration_);

  // Find current speed in forward direction
  b2Vec2 currentForwardNormal = tire_physics_body_->GetWorldVector(b2Vec2(1, 0));
  double currentSpeed = b2Dot(GetForwardVelocity(), currentForwardNormal);

  // Calculate drive force F = m * a on one of the four wheels
  b2Vec2 drive_force = 2.0 * desired_acceleration * this->tire_physics_body_->GetMass() * currentForwardNormal;

  // The impulse that can be delivered is bigger if the vehicle is faster
  double lateralImpulseAvailable = maximum_lateral_impulse_;
  lateralImpulseAvailable *= currentSpeed / this->maximum_velocity_;
  if (lateralImpulseAvailable < 0.5f * maximum_lateral_impulse_)
    lateralImpulseAvailable = 0.5f * maximum_lateral_impulse_;

  // Check is the max lateral friction impulse is violated
  b2Vec2 lateralFrictionImpulse = -tire_physics_body_->GetMass() * GetLateralVelocity();
  if (lateralFrictionImpulse.Length() > lateralImpulseAvailable)
    lateralFrictionImpulse *= lateralImpulseAvailable / lateralFrictionImpulse.Length();

  // Add both forces and aplly to the body
  b2Vec2 lateral_friction_force = 1.0 / step_size * lateralFrictionImpulse;
  b2Vec2 force = traction_multiplier_ * (lateral_friction_force + drive_force);
  tire_physics_body_->ApplyForceToCenter(force, true);
}

