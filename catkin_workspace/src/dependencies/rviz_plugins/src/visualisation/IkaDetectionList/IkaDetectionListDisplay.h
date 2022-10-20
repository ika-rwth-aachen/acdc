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

#pragma once

#include <memory>

#include <boost/circular_buffer.hpp>
#include <vector>

#include <rviz/message_filter_display.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/color_property.h>

#include <definitions/IkaDetectionList.h>

#include "IkaDetectionVisual.h"

namespace rviz_plugin {

/**
 * @authors Michael Hoss
 * @date 02.01.2019
 * @brief Display to handle IkaDetectionLists in Rviz.
 * @details This will use a list of IkaDetectionVisuals to display a whole object list in Rviz.
 */
class IkaDetectionListDisplay : public rviz::MessageFilterDisplay<definitions::IkaDetectionList> {
 Q_OBJECT
 private:

  std::vector<std::shared_ptr<IkaDetectionVisual>> visuals_; /**< Pointer to actual visual objects. */

  rviz::ColorProperty *geometric_color_property_ = nullptr; /**< Property to choose geometric color. */
  rviz::IntProperty *source_id_filter_value_property_ = nullptr; /**< Property to choose the source id filter value. */
  rviz::IntProperty *internal_id_filter_value_property_ = nullptr; /**< Property to choose the internal id filter value. */
  rviz::BoolProperty *source_id_filter_flag_property_ = nullptr; /**< Property to toggle the source id filter. */
  rviz::FloatProperty *z_shift_property_property_ = nullptr; /**< Property to shift geometric objects. */

  /**
   * @brief Callback function for incoming messages.
   * @details Will process all elements from an IkaDetectionLists and associate a visual for each element.
   * @param msg Incoming message.
   */
  void processMessage(const definitions::IkaDetectionList::ConstPtr &msg) override;
 public:

  /**
   * @brief Basic constructor that creates the properties.
   * @details The properties are added automatically to rviz do to the inheritance to MessageFilterDisplay.
   */
  IkaDetectionListDisplay();

  /**
   * @brief Basic destructor that frees the properties..
   */
  ~IkaDetectionListDisplay() override;

 protected:
  /**
   * @brief Passes all properties to the visuals.
   * @details Will be called automatically by rviz.
   */
  void onInitialize() override;

  /**
   * @brief Clears the display.
   * @details Will be called automatically by rviz.
   */
  void reset() override;

 private Q_SLOTS:

  /**
   * @brief Slot that will be triggered if one of the color properties were changed.
   */
  void updateColor();

  /**
   * @brief Slot that will be triggered if one of the source id filter properties were changed.
   */
  void updateSourceIdFilter();

  /**
   * @brief Slot that will be triggered if one of the shift properties were changed.
   */
  void updateZAxesShift();

};

}  // namespace rviz_plugin
