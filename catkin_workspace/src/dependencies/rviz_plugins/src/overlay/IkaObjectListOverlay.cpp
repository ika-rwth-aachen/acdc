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

#include "IkaObjectListOverlay.h"
#include <definitions/utility/ika_utilities.h>

namespace rviz_plugin {

void IkaObjectListOverlay::processMessage
    (const definitions::IkaObjectList::ConstPtr &msg) {
  if (!isEnabled()) {
    return;
  }
  if (source_id_filter_flag_property_->getBool()) {
    int source_id = source_id_filter_value_property_->getInt();
    if (source_id != msg->IdSource) return;
  }
  // store message for update method
  std::stringstream ss;
  ss << IkaUtilities::sourceToName(msg->IdSource) << "(" << std::to_string(msg->IdSource) << ")" << std::endl;
  ss << "# valid objects: " << std::to_string(IkaUtilities::numberValidObjects(&*msg)) << std::endl;

  text_ = ss.str();
  require_update_texture_ = true;
}

void IkaObjectListOverlay::onEnable() {
  if (overlay_) {
    overlay_->show();
  }
}

void IkaObjectListOverlay::onDisable() {
  if (overlay_) {
    overlay_->hide();
  }
}

void IkaObjectListOverlay::update(float /*wall_dt*/, float /*ros_dt*/) {
  if (!require_update_texture_) {
    return;
  }
  if (!isEnabled()) {
    return;
  }
  if (!overlay_) {
    return;
  }
  overlay_->updateTextureSize(static_cast<unsigned int>(texture_width_), static_cast<unsigned int>(texture_height_));
  {
    ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage Hud = buffer.getQImage(*overlay_, bg_color_);
    QPainter painter(&Hud);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(fg_color_, line_width_ || 1, Qt::SolidLine));
    unsigned int w = overlay_->getTextureWidth();
    unsigned int h = overlay_->getTextureHeight();

    // font
    if (text_size_ != 0) {
      //QFont font = painter.font();
      QFont font(font_.length() > 0 ? font_.c_str() : "Liberation Sans");
      font.setPointSize(text_size_);
      font.setBold(true);
      painter.setFont(font);
    }
    if (text_.length() > 0) {
      // painter.drawText(0, 0, w, h,
      //                  Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
      //                  text_.c_str());
      std::string color_wrapped_text
          = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
              % text_ % fg_color_.red() % fg_color_.green() % fg_color_.blue() %
              fg_color_.alpha()).str();
      QStaticText static_text(
          boost::algorithm::replace_all_copy(color_wrapped_text, "\n", "<br >").c_str());
      static_text.setTextWidth(w);
      painter.drawStaticText(0, 0, static_text);
    }
    painter.end();
  }
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  overlay_->setPosition(left_, top_);
  require_update_texture_ = false;
}

// only the first time
void IkaObjectListOverlay::onInitialize() {
  MFDClass::onInitialize();
  onEnable();
  updateVisible();
  updateTop();
  updateLeft();
  updateWidth();
  updateHeight();
  updateTextSize();
  updateFGColor();
  updateFGAlpha();
  updateBGColor();
  updateBGAlpha();
  updateFont();
  updateLineWidth();
  updateSourceIdFilter();
  require_update_texture_ = true;
}

IkaObjectListOverlay::IkaObjectListOverlay() {
  visible_property_ = new rviz::BoolProperty("Visibility property.",
                                             false,
                                             "Decides of overlay is visible or not.",
                                             this,
                                             SLOT(updateVisible()));
  top_property_ = new rviz::IntProperty("top", 0, "top position", this, SLOT(updateTop()));
  top_property_->setMin(0);
  left_property_ = new rviz::IntProperty("left", 0, "left position", this, SLOT(updateLeft()));
  left_property_->setMin(0);
  width_property_ = new rviz::IntProperty("width", 128, "width position", this, SLOT(updateWidth()));
  width_property_->setMin(0);
  height_property_ = new rviz::IntProperty("height", 128, "height position", this, SLOT(updateHeight()));
  height_property_->setMin(0);
  text_size_property_ = new rviz::IntProperty("text size", 12, "text size", this, SLOT(updateTextSize()));
  text_size_property_->setMin(0);
  line_width_property_ = new rviz::IntProperty("line width", 2, "line width", this, SLOT(updateLineWidth()));
  line_width_property_->setMin(0);
  fg_color_property_ = new rviz::ColorProperty("Foreground Color",
                                               QColor(25, 255, 240),
                                               "Foreground Color",
                                               this,
                                               SLOT(updateFGColor()));
  fg_alpha_property_ =
      new rviz::FloatProperty("Foreground Alpha", 0.8, "Foreground Alpha", this, SLOT(updateFGAlpha()));
  fg_alpha_property_->setMin(0.0);
  fg_alpha_property_->setMax(1.0);
  bg_color_property_ =
      new rviz::ColorProperty("Background Color", QColor(0, 0, 0), "Background Color", this, SLOT(updateBGColor()));
  bg_alpha_property_ =
      new rviz::FloatProperty("Background Alpha", 0.8, "Background Alpha", this, SLOT(updateBGAlpha()));
  bg_alpha_property_->setMin(0.0);
  bg_alpha_property_->setMax(1.0);

  source_id_filter_value_property_ =
      new rviz::IntProperty("Source ID", -1, "Source ID value to filter input.", this, SLOT(updateSourceIdFilter()));
  source_id_filter_value_property_->setMin(-1);

  source_id_filter_flag_property_ = new rviz::BoolProperty("Filter Source ID",
                                                           false,
                                                           "Activate or deactivate input filter via source ID.",
                                                           this,
                                                           SLOT(updateSourceIdFilter()));

  QFontDatabase database;
  font_families_ = database.families();
  font_property_ = new rviz::EnumProperty("font", "DejaVu Sans Mono", "font", this, SLOT(updateFont()));
  for (size_t i = 0; i < font_families_.size(); i++) {
    font_property_->addOption(font_families_[i], (int) i);
  }
  require_update_texture_ = true;
  static int i = 0;
  overlay_.reset(new rviz_plugin::OverlayObject("Ogre Overlay number " + std::to_string(i)));
  i++;
  overlay_->show();
}

IkaObjectListOverlay::~IkaObjectListOverlay() {
  onDisable();
  //delete overlay_;
  delete top_property_;
  delete left_property_;
  delete width_property_;
  delete height_property_;
  delete text_size_property_;
  delete line_width_property_;
  delete bg_color_property_;
  delete bg_alpha_property_;
  delete fg_color_property_;
  delete fg_alpha_property_;
  delete font_property_;
}

void IkaObjectListOverlay::updateSourceIdFilter() {
  if (source_id_filter_flag_property_->getBool()) {
    source_id_filter_value_property_->show();
    if (source_id_filter_value_property_->getInt() == -1) {
      source_id_filter_value_property_->setValue(0);
    }
    return;
  }
  source_id_filter_value_property_->hide();
}

void IkaObjectListOverlay::updateVisible() {
  visible_ = visible_property_->getBool();
  if (visible_) {
    top_property_->show();
    left_property_->show();
    width_property_->show();
    height_property_->show();
    text_size_property_->show();
  } else {
    top_property_->hide();
    left_property_->hide();
    width_property_->hide();
    height_property_->hide();
    text_size_property_->hide();
  }
}

void IkaObjectListOverlay::updateTop() {
  top_ = top_property_->getInt();
  if (visible_) {
    require_update_texture_ = true;
  }
}

void IkaObjectListOverlay::updateLeft() {
  left_ = left_property_->getInt();
  if (visible_) {
    require_update_texture_ = true;
  }
}

void IkaObjectListOverlay::updateWidth() {
  texture_width_ = width_property_->getInt();
  if (visible_) {
    require_update_texture_ = true;
  }
}

void IkaObjectListOverlay::updateHeight() {
  texture_height_ = height_property_->getInt();
  if (visible_) {
    require_update_texture_ = true;
  }
}

void IkaObjectListOverlay::updateTextSize() {
  text_size_ = text_size_property_->getInt();
  if (visible_) {
    require_update_texture_ = true;
  }
}

void IkaObjectListOverlay::updateBGColor() {
  QColor c = bg_color_property_->getColor();
  bg_color_.setRed(c.red());
  bg_color_.setGreen(c.green());
  bg_color_.setBlue(c.blue());
}

void IkaObjectListOverlay::updateBGAlpha() {
  bg_color_.setAlpha(static_cast<int>(bg_alpha_property_->getFloat() * 255.0f));
}

void IkaObjectListOverlay::updateFGColor() {
  QColor c = fg_color_property_->getColor();
  fg_color_.setRed(c.red());
  fg_color_.setGreen(c.green());
  fg_color_.setBlue(c.blue());
}

void IkaObjectListOverlay::updateFGAlpha() {
  fg_color_.setAlpha(static_cast<int>(fg_alpha_property_->getFloat() * 255.0f));
}

void IkaObjectListOverlay::updateFont() {
  int font_index = font_property_->getOptionInt();
  if (font_index < font_families_.size()) {
    font_ = font_families_[font_index].toStdString();
  } else {
    ROS_FATAL("Unexpected error at selecting font index %d.", font_index);
    return;
  }
}

void IkaObjectListOverlay::updateLineWidth() {
  line_width_ = line_width_property_->getInt();
}

}  // namespace rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin::IkaObjectListOverlay, rviz::Display)
