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

#include "IkaObjectVisual.h"

#include <definitions/utility/ika_utilities.h>

namespace rviz_plugin {

float IkaObjectVisual::calculateDisplayExistenceProbability(float real_existence_probability) {
  return static_cast<float>(MIN_OPACITY + real_existence_probability * (MAX_OPACITY - MIN_OPACITY));
}

IkaObjectVisual::IkaObjectVisual(Ogre::SceneManager *scene_manager,
                                 Ogre::SceneNode *parent_node,
                                 rviz::DisplayContext *context) {
  scene_manager_ = scene_manager;
  // Create reference frames.
  frame_node_ = parent_node->createChildSceneNode();
  text_node_ = frame_node_->createChildSceneNode();

  // Create geometric objects
  velocity_arrow_ = std::make_shared<rviz::Arrow>(scene_manager_, frame_node_);
  body_box_ = std::make_shared<rviz::Shape>(rviz::Shape::Type::Cube, scene_manager_, frame_node_);
  type_name_text_ = std::make_shared<rviz::MovableText>("Test Test");

  // Attach text to scene
  type_name_text_->setTextAlignment(rviz::MovableText::H_LEFT, rviz::MovableText::V_BELOW);  
  text_node_->attachObject(type_name_text_.get());

  selection_handler_ = std::make_shared<rviz_plugin::IkaObjectVisualSelectionHandler>(context);
  selection_handler_->setObject(&object);
  selection_handler_->addTrackedObjects(frame_node_);
}

IkaObjectVisual::~IkaObjectVisual() {
  scene_manager_->destroySceneNode(frame_node_);
  scene_manager_->destroySceneNode(text_node_);
}

void IkaObjectVisual::setMessage(const definitions::IkaObject &msg, bool visible) {
  static Ogre::Vector3 z_axes(0, 0, 1);

  object = msg;
  frame_node_->setVisible(object.bObjectValid, true);
  text_node_->setVisible(object.bObjectValid, true);
  body_box_->getRootNode()->setVisible(object.bObjectValid, true);
  if (!object.bObjectValid){
    return;
  }

  // Velocity quantities
  std::vector<float> velo = IkaUtilities::getObjectVelocity(msg);
  Ogre::Vector3 velocity(velo[0], velo[1], 0);
  Ogre::Real velocity_magnitude = velocity.length();
  Ogre::Vector3 scale_arrow_vel(velocity_magnitude*0.5f, 1, 1); // Scale only length of the arrow

  // Orientation of the box
  Ogre::Radian angle(IkaUtilities::getObjectHeading(msg));
  Ogre::Quaternion orientation;
  orientation.FromAngleAxis(angle, z_axes);
  Ogre::Quaternion oppos_orientation;
  oppos_orientation.FromAngleAxis(-angle, z_axes);

  if (orientation.isNaN() || velocity.isNaN()) {
    ROS_ERROR("IkaObjectVisual.cpp: Orientation or velocity is NaN!");
    return;
  }

  // Box size and position
  std::vector<float> pos = IkaUtilities::getObjectPosition(msg);
  std::vector<float> size = IkaUtilities::getObjectSize(msg);
  Ogre::Vector3 center_position(pos[0],pos[1],pos[2]);
  Ogre::Vector3 scale_box(size[0],size[1],size[2]);
  Ogre::Vector3 rotatedBoundingBox = orientation*scale_box;
  Ogre::Vector3 oppos_rotatedBoundingBox = oppos_orientation*scale_box;

  Ogre::Vector3 text_position(pos[0] + 0.5f * std::abs(rotatedBoundingBox.x),
                              pos[1] - 0.6f * std::max(std::abs(oppos_rotatedBoundingBox.y), std::abs(rotatedBoundingBox.y)),
                              pos[2] + 0.5f * std::abs(rotatedBoundingBox.z) + z_shift_ + text_shift_);

  // Use existence probability to choose alpha channel
  float existence = msg.fExistenceProbability;
  existence = calculateDisplayExistenceProbability(existence);
  geometric_color = Ogre::ColourValue(geometric_color.r, geometric_color.g, geometric_color.b, existence);
  text_color = Ogre::ColourValue(text_color.r, text_color.g, text_color.b, 0.8f);

  // Update arrow
  velocity_arrow_->setDirection(velocity);
  velocity_arrow_->setScale(scale_arrow_vel);
  velocity_arrow_->setPosition(center_position);
  velocity_arrow_->setColor(geometric_color);

  // Update box
  body_box_->setPosition(center_position);
  body_box_->setScale(scale_box);
  body_box_->setOrientation(orientation);
  body_box_->setColor(geometric_color);

  // Update text
  type_name_text_->setVisible(msg.bObjectValid && visible);

  std::string msg_str;
  generateObjectInfoString(msg_str, msg);
  
  type_name_text_->setCaption(msg_str);
  type_name_text_->setColor(text_color);
  text_node_->setPosition(text_position);
}

void IkaObjectVisual::generateObjectInfoString(
    std::string& msg_str,
    const definitions::IkaObject& msg)
{
  // some parts of the string commented out because otherwise too much text to read
  msg_str.append(std::to_string(msg.IdInternal));
  msg_str.append("|");
  msg_str.append(IkaUtilities::typeToName(static_cast<uint8_t>(msg.IdType)));
  msg_str.append("|");
  msg_str.append(std::to_string((int)(msg.fExistenceProbability*100)));

  msg_str.append("\n");

  if (msg.IdExternal == definitions::env_model_outputs::FUSED_OBJECTS) {
    bool first = true;
    for (auto& sensorStamp : msg.measHist) {
      std::string sensor_str = IkaUtilities::sourceToLetter(sensorStamp.IdSensor);

      if (first) {
        first = false;
      } else {
        msg_str.append("|");
      }

      msg_str.append(sensor_str);
    }
  }

//  msg_str.append("\n");
//  std::string existenceProbability = std::to_string(msg.fExistenceProbability);
//  existenceProbability.erase(5, existenceProbability.size()); // set existenceProbability precision to 0.XXX
//  msg_str.append(existenceProbability);
}

void IkaObjectVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void IkaObjectVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

void IkaObjectVisual::setObjectColor(Ogre::ColourValue color) {
  this->geometric_color = color;
}

void IkaObjectVisual::setTextColor(Ogre::ColourValue color) {
  this->text_color = color;
}

void IkaObjectVisual::setZShift(float z_shift) {
  this->z_shift_ = z_shift;
}

void IkaObjectVisual::setTextShift(float z_shift) {
  this->text_shift_ = z_shift;
}

}  // namespace rviz_plugin
