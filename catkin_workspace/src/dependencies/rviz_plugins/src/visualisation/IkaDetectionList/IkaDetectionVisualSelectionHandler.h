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

#pragma once

#include <rviz/selection/selection_handler.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>

#include <definitions/IkaDetection.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>

namespace rviz_plugin {

class IkaDetectionVisualSelectionHandler : public rviz::SelectionHandler {
 private:
  const definitions::IkaDetection *object_;

  rviz::BoolProperty *bstatus_property_;
  rviz::IntProperty *ivalidlevel_property_;

  rviz::FloatProperty *range_property_;
  rviz::FloatProperty *range_rate_property_;
  rviz::FloatProperty *angle_property_;
  rviz::FloatProperty *amplitude_property_;

  // rviz::StringProperty *type_id_property_; // maybe use this for sourceID?

 public:
  IkaDetectionVisualSelectionHandler(rviz::DisplayContext *context);
  ~IkaDetectionVisualSelectionHandler() override;

  bool getStatus();
  int getValidLevel();

  float getRangeRate();
  float getRange();
  float getAngle();
  float getAmplitude();


  // QString getTypeIdString();


  void createProperties(const rviz::Picked &obj, rviz::Property *parent_property) override;
  void updateProperties() override;

  const definitions::IkaDetection *getObject() const;
  void setObject(const definitions::IkaDetection *object);
};

}  // namespace rviz_plugin
