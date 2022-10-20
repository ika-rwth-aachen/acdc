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

#include <definitions/utility/ika_utilities.h>
#include <QtGui/QtGui>
#include "IkaDetectionVisualSelectionHandler.h"

namespace rviz_plugin {

IkaDetectionVisualSelectionHandler::IkaDetectionVisualSelectionHandler(rviz::DisplayContext *context)
    : rviz::SelectionHandler(context) {}

IkaDetectionVisualSelectionHandler::~IkaDetectionVisualSelectionHandler() {
//  delete bstatus_property_;
//  delete ivalidlevel_property_;
//  delete range_property_;
//  delete range_rate_property_;
//  delete angle_property_;
//  delete amplitude_property_;
}


bool IkaDetectionVisualSelectionHandler::getStatus() {
  if (object_ == nullptr) return false;
  return object_->bStatus;
}

int IkaDetectionVisualSelectionHandler::getValidLevel() {
  if (object_ == nullptr) return 0;
  return object_->iValidLevel;
}


float IkaDetectionVisualSelectionHandler::getRange() {
  if (object_ == nullptr) return 0.f;
  return static_cast<float>(object_->fRange);
}

float IkaDetectionVisualSelectionHandler::getRangeRate() {
  if (object_ == nullptr) return 0.f;
  return static_cast<float>(object_->fRangeRate);
}

float IkaDetectionVisualSelectionHandler::getAngle() {
  if (object_ == nullptr) return 0.f;
  return static_cast<float>(object_->fAngle);
}

float IkaDetectionVisualSelectionHandler::getAmplitude() {
  if (object_ == nullptr) return 0.f;
  return static_cast<float>(object_->fAmplitude);
}


//QString IkaDetectionVisualSelectionHandler::getTypeIdString() {
//  if (object_ == nullptr) return QString::fromStdString(nullptr);
//  return QString::fromStdString(IkaUtilities::typeToName(static_cast<uint8_t>(object_->IdType)));
//}

void IkaDetectionVisualSelectionHandler::createProperties(const rviz::Picked &obj, rviz::Property *parent_property) {
  SelectionHandler::createProperties(obj, parent_property);

  rviz::Property *group =
      new rviz::Property("Detection", QVariant(), "", parent_property);
  properties_.push_back(group);

  bstatus_property_ = new rviz::BoolProperty("Status", getStatus(), "", group);
  bstatus_property_->setReadOnly(true);

  ivalidlevel_property_ = new rviz::IntProperty("Valid Level", getValidLevel(), "", group);
  ivalidlevel_property_->setReadOnly(true);


//  type_id_property_ = new rviz::StringProperty("Type Id", getTypeIdString(), "", group);
//  type_id_property_->setReadOnly(true);
//

  range_property_ = new rviz::FloatProperty("Range", getRange(), "", group);
  range_property_->setReadOnly(true);

  range_rate_property_ = new rviz::FloatProperty("RangeRate", getRangeRate(), "", group);
  range_rate_property_->setReadOnly(true);

  angle_property_ = new rviz::FloatProperty("Angle", getAngle(), "", group);
  angle_property_->setReadOnly(true);

  amplitude_property_ = new rviz::FloatProperty("Amplitude", getAmplitude(), "", group);
  amplitude_property_->setReadOnly(true);


  group->expand();
}

void IkaDetectionVisualSelectionHandler::updateProperties() {
  SelectionHandler::updateProperties();

  bstatus_property_->setBool(getStatus());
  ivalidlevel_property_->setInt(getValidLevel());

  range_property_->setFloat(getRange());
  range_rate_property_->setFloat(getRangeRate());
  angle_property_->setFloat(getAngle());
  amplitude_property_->setFloat(getAmplitude());


  // type_id_property_->setString(getTypeIdString());
}

const definitions::IkaDetection *IkaDetectionVisualSelectionHandler::getObject() const {
  return object_;
}

void IkaDetectionVisualSelectionHandler::setObject(const definitions::IkaDetection *object) {
  object_ = object;
}

}  // namespace rviz_plugin
