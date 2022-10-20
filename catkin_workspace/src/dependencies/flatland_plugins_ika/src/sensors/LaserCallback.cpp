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

#include "sensors/LaserCallback.h"

namespace flatland_ika_plugins {

LaserCallback::LaserCallback(LaserScanner *parent, double angle)
  : parent_(parent), angle_(angle) {}

float LaserCallback::ReportFixture(b2Fixture *fixture, const b2Vec2 &point, const b2Vec2 &normal, float fraction) {
  double intensity = 0;
  double distance = 0;

  // Filter not valid hits
  uint16_t category_bits = fixture->GetFilterData().categoryBits;
  if (category_bits == 0xFFFF) return -1.0f;
  if (fixture->IsSensor()) return -1.0f;

  // Check if body is a valid flatland body
  auto *raw_pointer = fixture->GetBody()->GetUserData();
  if (raw_pointer == nullptr)  return -1.0f;

  // Check if valid body is a lane marking
  auto temp_body = static_cast<flatland_server::Body *>(raw_pointer);
  std::string body_name = temp_body->name_;
  if (body_name.find("2d") != 0 && body_name.find("3d") != 0) return -1.0f;
  // Hit is valid
  this->hit_ = true;
  this->body_ = temp_body;
  // Distance were the hit was
  this->distance_ = fraction * parent_->getMaxRange();
  // Box2D body that was hit
  this->physics_body_ = fixture->GetBody();
  // Copy point
  this->point_.setX(this->distance_ * std::cos(this->angle_));
  this->point_.setY(this->distance_ * std::sin(this->angle_));

  // Set intensities
  this->intensity_ = LaserScanner::INTENSITIES::ENVIRONMENT;
  if (body_name.find("left") != std::string::npos) {
    this->intensity_ = LaserScanner::INTENSITIES::LEFT_LANE;
  } else if (body_name.find("right") != std::string::npos) {
    this->intensity_ = LaserScanner::INTENSITIES::RIGHT_LANE;
  }

  return fraction;
}

bool LaserCallback::hasHit() const {
  return this->hit_;
}

double LaserCallback::getDistance() const {
  return this->distance_;
}

double LaserCallback::getIntensity() const {
  return this->intensity_;
}

double LaserCallback::getAngle() const {
  return this->angle_;
}

b2Body *LaserCallback::getPhysicsBody() const {
  return this->physics_body_;
}

flatland_server::Body *LaserCallback::getBody() const {
  return this->body_;
}
tf::Point LaserCallback::getPoint() const {
  return this->point_;
}

};