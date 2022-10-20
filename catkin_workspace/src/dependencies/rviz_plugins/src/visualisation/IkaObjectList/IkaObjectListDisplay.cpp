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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>

#include "IkaObjectVisual.h"

#include "IkaObjectListDisplay.h"

namespace rviz_plugin {

void IkaObjectListDisplay::processMessage(const definitions::IkaObjectList::ConstPtr &msg) {
  // Check if source id filter is active
  if (source_id_filter_flag_property_->getBool()) {
    int source_id = source_id_filter_value_property_->getInt();
    if (source_id != msg->IdSource) return;
  }

  // Get frame of reference information
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id,
                                                 msg->header.stamp,
                                                 position,
                                                 orientation)) {
    ROS_ERROR("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }

  if (orientation.isNaN()) {
    ROS_ERROR("IkaObjectListDisplay.cpp: Ogre orientation is NaN!");
    return;
  }

  definitions::IkaObjectList object_list = *msg;

  // Reset old objects
  if (object_list.objects.size() < visuals_.size()) {
    for (size_t i = object_list.objects.size(); i < visuals_.size(); i++){
      visuals_[i]->setMessage(definitions::IkaObject(), show_text_->getBool());
      visuals_[i]->setFramePosition(position);
      visuals_[i]->setFrameOrientation(orientation);
    }
  }

  for (size_t i = 0; i < object_list.objects.size(); i++) {
    // Check if the visual was already created
    if (visuals_.size() <= i) {
      // Get current properties
      Ogre::ColourValue geometric_color = geometric_color_property_->getOgreColor();
      Ogre::ColourValue text_color = text_color_property_->getOgreColor();
      float z_axes_shift = z_shift_property_property_->getFloat();
      float text_shift = text_shift_property_property_->getFloat();
      // Add new visual
      visuals_.push_back(std::make_shared<IkaObjectVisual>(context_->getSceneManager(), scene_node_, context_));
      visuals_[i]->setObjectColor(geometric_color);
      visuals_[i]->setTextColor(text_color);
      visuals_[i]->setTextShift(text_shift);
      visuals_[i]->setZShift(z_axes_shift);
    }

    // Update already existing visuals
    definitions::IkaObject object = object_list.objects[i];
    if (!object.bObjectValid) object = definitions::IkaObject();
    visuals_[i]->setMessage(object, show_text_->getBool());
    visuals_[i]->setFramePosition(position);
    visuals_[i]->setFrameOrientation(orientation);
  }
}

IkaObjectListDisplay::IkaObjectListDisplay() {
  // Create properties
  geometric_color_property_ = new rviz::ColorProperty("Geometric Color", QColor(204, 51, 204),
                                                      "Color to draw all geometric objects.",
                                                      this, SLOT(updateColor()));

  text_color_property_ = new rviz::ColorProperty("Text Color", QColor(204, 51, 204),
                                                 "Color to draw texts.",
                                                 this, SLOT(updateColor()));

  source_id_filter_value_property_ = new rviz::IntProperty("Source ID", -1,
                                                           "Source ID value to filter input.",
                                                           this, SLOT(updateSourceIdFilter()));
  source_id_filter_value_property_->setMin(-1);

  source_id_filter_flag_property_ = new rviz::BoolProperty("Filter Source ID", false,
                                                           "Activate or deactivate input filter via source ID.",
                                                           this, SLOT(updateSourceIdFilter()));

  z_shift_property_property_ = new rviz::FloatProperty("Z axes shift.", 0.0f,
                                                       "Will shift all geometric objects about this value.",
                                                       this, SLOT(updateZAxesShift()));
  text_shift_property_property_ = new rviz::FloatProperty("Z axes shift for text.", 0.0f,
                                                          "Will shift all text elements about this value.",
                                                          this, SLOT(updateZAxesShift()));
  show_text_ = new rviz::BoolProperty("Show Text", true, "Activate or deactivate text for objects",
                                                           this, SLOT(updateShowText()));
}

void IkaObjectListDisplay::onInitialize() {
  MFDClass::onInitialize();
  updateSourceIdFilter();
  updateColor();
}

IkaObjectListDisplay::~IkaObjectListDisplay() {
  delete geometric_color_property_;
  delete text_color_property_;
  delete source_id_filter_value_property_;
  delete source_id_filter_flag_property_;
  delete z_shift_property_property_;
  delete text_shift_property_property_;
  delete show_text_;
}

void IkaObjectListDisplay::reset() {
  MFDClass::reset();
  for (auto &visual : visuals_) {
    if (visual) visual->setMessage(definitions::IkaObject(), show_text_->getBool());
  }
}

void IkaObjectListDisplay::updateColor() {
  Ogre::ColourValue geometric_color = geometric_color_property_->getOgreColor();
  Ogre::ColourValue text_color = text_color_property_->getOgreColor();
  for (auto &visual : visuals_) {
    if (visual) {
      visual->setObjectColor(geometric_color);
      visual->setTextColor(text_color);
    }
  }
}

void IkaObjectListDisplay::updateSourceIdFilter() {
  if (source_id_filter_flag_property_->getBool()) {
    source_id_filter_value_property_->show();
    if (source_id_filter_value_property_->getInt() == -1) {
      source_id_filter_value_property_->setValue(0);
    }
    return;
  }
  source_id_filter_value_property_->hide();
}

void IkaObjectListDisplay::updateZAxesShift() {
  float z_axes_shift = z_shift_property_property_->getFloat();
  float text_shift = text_shift_property_property_->getFloat();
  for (auto &visual : visuals_) {
    if (visual) {
      visual->setZShift(z_axes_shift);
      visual->setTextShift(text_shift);
    }
  }
}

void IkaObjectListDisplay::updateShowText() {
    return;
}

}  // namespace rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin::IkaObjectListDisplay, rviz::Display)
