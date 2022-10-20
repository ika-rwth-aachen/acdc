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

#include "IkaDetectionVisual.h"

#include "IkaDetectionListDisplay.h"

namespace rviz_plugin {

void IkaDetectionListDisplay::processMessage(const definitions::IkaDetectionList::ConstPtr &msg) {
  // Check if source id filter is active
  if (source_id_filter_flag_property_->getBool()) {
    int source_id = source_id_filter_value_property_->getInt();
    int internal_id = internal_id_filter_value_property_->getInt();

    // only continue if both source id and internal id do match the user-set values
    if (source_id != msg->IdSource || internal_id != msg->IdInternal) return;
  }

  // Get frame of reference information
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }


  definitions::IkaDetectionList detection_list = *msg;

  // Reset old detections
  if (detection_list.detections.size() < visuals_.size()) {
    for (size_t i = detection_list.detections.size(); i < visuals_.size(); i++){
      visuals_[i]->setMessage(definitions::IkaDetection());
      visuals_[i]->setFramePosition(position);
      visuals_[i]->setFrameOrientation(orientation);
    }
  }

  for (size_t i = 0; i < msg->detections.size(); i++) {
    // Check if the visual was already created
    if (visuals_.size() <= i) {
      // Get current properties
      Ogre::ColourValue geometric_color = geometric_color_property_->getOgreColor();
      float z_axes_shift = z_shift_property_property_->getFloat();

      // Add new visual
      visuals_.push_back(std::make_shared<IkaDetectionVisual>(context_->getSceneManager(), scene_node_, context_));
      visuals_[i]->setObjectColor(geometric_color);
      visuals_[i]->setZShift(z_axes_shift);
    }

    // Update already existence visuals
    definitions::IkaDetection detection = detection_list.detections[i];

    if (!detection.bStatus) detection = definitions::IkaDetection();

    visuals_[i]->setMessage(detection);
    visuals_[i]->setFramePosition(position);
    visuals_[i]->setFrameOrientation(orientation);
  }
}

IkaDetectionListDisplay::IkaDetectionListDisplay() {

  // Create properties
  geometric_color_property_ = new rviz::ColorProperty("Geometric Color", QColor(204, 51, 204),
                                                      "Color to draw all geometric objects.",
                                                      this, SLOT(updateColor()));


  source_id_filter_value_property_ = new rviz::IntProperty("Source ID", -1,
                                                           "Source ID value to filter input.",
                                                           this, SLOT(updateSourceIdFilter()));

  internal_id_filter_value_property_ = new rviz::IntProperty("Internal ID", -1,
                                                           "Internal ID value to filter input.",
                                                           this, SLOT(updateSourceIdFilter()));


  source_id_filter_value_property_->setMin(-1);
  internal_id_filter_value_property_->setMin(-1);

  source_id_filter_flag_property_ = new rviz::BoolProperty("Filter Source ID", false,
                                                           "Activate or deactivate input filter via source ID.",
                                                           this, SLOT(updateSourceIdFilter()));

  z_shift_property_property_ = new rviz::FloatProperty("Z axes shift.", 0.0f,
                                                       "Will shift all geometric objects about this value.",
                                                       this, SLOT(updateZAxesShift()));

}

void IkaDetectionListDisplay::onInitialize() {
  MFDClass::onInitialize();
  updateSourceIdFilter();
  updateColor();
}

IkaDetectionListDisplay::~IkaDetectionListDisplay() {
  delete geometric_color_property_;
  delete source_id_filter_value_property_;
  delete internal_id_filter_value_property_;
  delete source_id_filter_flag_property_;
  delete z_shift_property_property_;

}

void IkaDetectionListDisplay::reset() {
  MFDClass::reset();
  for (auto &visual : visuals_) {
    if (visual) visual->setMessage(definitions::IkaDetection());
  }
}

void IkaDetectionListDisplay::updateColor() {
  Ogre::ColourValue geometric_color = geometric_color_property_->getOgreColor();
  for (auto &visual : visuals_) {
    if (visual) {
      visual->setObjectColor(geometric_color);
    }
  }
}

void IkaDetectionListDisplay::updateSourceIdFilter() {
  if (source_id_filter_flag_property_->getBool()) {

    source_id_filter_value_property_->show();

    internal_id_filter_value_property_->show();

    if (source_id_filter_value_property_->getInt() == -1) {
      source_id_filter_value_property_->setValue(0);
    }

    if (internal_id_filter_value_property_->getInt() == -1) {
      internal_id_filter_value_property_->setValue(0);
    }

    return;
  }
  source_id_filter_value_property_->hide();
  internal_id_filter_value_property_->hide();
}

void IkaDetectionListDisplay::updateZAxesShift() {
  float z_axes_shift = z_shift_property_property_->getFloat();
  for (auto &visual : visuals_) {
    if (visual) {
      visual->setZShift(z_axes_shift);
    }
  }
}

}  // namespace rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin::IkaDetectionListDisplay, rviz::Display)
