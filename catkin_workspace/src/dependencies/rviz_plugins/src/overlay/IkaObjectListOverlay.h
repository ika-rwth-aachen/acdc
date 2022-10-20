// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#pragma once

#include <rviz/display.h>
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreMaterial.h>
#include <std_msgs/ColorRGBA.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>
#include <definitions/IkaObjectList.h>
#include <rviz/message_filter_display.h>
#include <OGRE/OgreMaterialManager.h>
#include <rviz/uniform_string_stream.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <QFontDatabase>
#include <QPainter>
#include <QStaticText>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "IkaOverlayUtils.h"

namespace rviz_plugin {

/**
 * @authors Simon Schaefer
 * @date 10.12.2018
 * @brief Display to handel IkaObjectLists overlay in Rviz.
 * @details This will use a list of IkaOverlayUtils to display an overlay.
 */
class IkaObjectListOverlay : public rviz::MessageFilterDisplay<definitions::IkaObjectList> {
 Q_OBJECT
 private:
  std::shared_ptr<OverlayObject> overlay_;

  int texture_width_;
  int texture_height_;

  bool visible_;
  QColor bg_color_;
  QColor fg_color_;
  int text_size_;
  int line_width_;
  std::string text_;
  QStringList font_families_;
  std::string font_;
  int left_;
  int top_;

  bool require_update_texture_;
  rviz::BoolProperty *visible_property_;
  rviz::IntProperty *top_property_;
  rviz::IntProperty *left_property_;
  rviz::IntProperty *width_property_;
  rviz::IntProperty *height_property_;
  rviz::IntProperty *text_size_property_;
  rviz::IntProperty *line_width_property_;
  rviz::ColorProperty *bg_color_property_;
  rviz::FloatProperty *bg_alpha_property_;
  rviz::ColorProperty *fg_color_property_;
  rviz::FloatProperty *fg_alpha_property_;
  rviz::EnumProperty *font_property_;
  rviz::IntProperty *source_id_filter_value_property_ = nullptr;
  rviz::BoolProperty *source_id_filter_flag_property_ = nullptr;

  /**
   * @brief Callback function for incoming messages.
   * @details Will process all meta information from an IkaObjectLists and displays them in an overlay.
   * @param msg Incoming message.
   */
  void processMessage(const definitions::IkaObjectList::ConstPtr &msg) override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

 public:
  IkaObjectListOverlay();
  ~IkaObjectListOverlay() override;

 protected
  Q_SLOTS:
  void updateVisible();
  void updateTop();
  void updateLeft();
  void updateWidth();
  void updateHeight();
  void updateTextSize();
  void updateFGColor();
  void updateFGAlpha();
  void updateBGColor();
  void updateBGAlpha();
  void updateFont();
  void updateLineWidth();
  void updateSourceIdFilter();
};

}  // namespace rviz_plugin
