/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "IkaDetectionVisual.h"

#include <definitions/utility/ika_utilities.h>

namespace rviz_plugin {

float IkaDetectionVisual::calculateAlpha(float amplitude) {

  // scaledAmplitude, should be between 0 and 1 but can exceed
  float scaledAmplitude = (amplitude - MIN_AMPL) / ( MAX_AMPL - MIN_AMPL);

  // raw alpha, should be between MIN_OPACITY and MAX_OPACITY but can exceed
  float rawAlpha = MIN_OPACITY + scaledAmplitude* (MAX_OPACITY - MIN_OPACITY);

  // trimming to the proper range
  rawAlpha = fminf(rawAlpha, MAX_OPACITY);
  rawAlpha = fmaxf(rawAlpha, MIN_OPACITY);

  return rawAlpha;
}

IkaDetectionVisual::IkaDetectionVisual(Ogre::SceneManager *scene_manager,
                                 Ogre::SceneNode *parent_node,
                                 rviz::DisplayContext *context) {
  scene_manager_ = scene_manager;
  // Create reference frames.
  frame_node_ = parent_node->createChildSceneNode();

  // Create geometric objects
  body_box_ = std::make_shared<rviz::Shape>(rviz::Shape::Type::Cube, scene_manager_, frame_node_);

  selection_handler_ = std::make_shared<rviz_plugin::IkaDetectionVisualSelectionHandler>(context);
  selection_handler_->setObject(&detection);
  selection_handler_->addTrackedObjects(frame_node_);
}

IkaDetectionVisual::~IkaDetectionVisual() {
  scene_manager_->destroySceneNode(frame_node_);
}




/**
 * @brief
 * @details
 * @param msg Incoming Message. msg.fAngle is assumed to be in radiants.
 */
void IkaDetectionVisual::setMessage(const definitions::IkaDetection &msg) {
  static Ogre::Vector3 z_axes(0, 0, 1);

  detection = msg;
  frame_node_->setVisible(detection.bStatus, true);

  body_box_->getRootNode()->setVisible(detection.bStatus, true);
  if (!detection.bStatus){
    return;
  }


  auto dPosX = static_cast<float>(detection.fRange * cos(detection.fAngle));
  auto dPosY = static_cast<float>(detection.fRange * sin(detection.fAngle));
  float dPosZ = 0; // 10.f * static_cast<float>(detection.fRangeRate / RANGE_RATE_MAX);


  // Box size and position
  Ogre::Vector3 center_position(dPosX, dPosY, dPosZ + z_shift_);
  Ogre::Vector3 scale_box(BOXSIZE_RANGE, BOXSIZE_ANGLE, BOXSIZE_HEIGHT);

  // Use amplitude probability to choose alpha chanel
  auto amplitude = static_cast<float>(msg.fAmplitude);
  float alpha = calculateAlpha(amplitude);
  geometric_color = Ogre::ColourValue(geometric_color.r, geometric_color.g, geometric_color.b, alpha);


  // set orientation to zero (actually not necessary)
  // Orientation of the box
//  Ogre::Radian angle(0.f);
//  Ogre::Quaternion orientation;
//  orientation.FromAngleAxis(angle, z_axes);


  // Update box
  body_box_->setPosition(center_position);
  body_box_->setScale(scale_box);
  body_box_->setColor(geometric_color);
//  body_box_->setOrientation(orientation);

}

void IkaDetectionVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void IkaDetectionVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

void IkaDetectionVisual::setObjectColor(Ogre::ColourValue color) {
  this->geometric_color = color;
}

void IkaDetectionVisual::setZShift(float z_shift) {
  this->z_shift_ = z_shift;
}

}  // namespace rviz_plugin
