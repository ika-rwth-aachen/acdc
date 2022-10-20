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

#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/selection/selection_handler.h>

#include <definitions/IkaObject.h>
#include <definitions/utility/object_definitions.h>

#include "AngleProperty.h"

namespace rviz_plugin {

class IkaObjectVisualSelectionHandler : public rviz::SelectionHandler {
 private:
  const definitions::IkaObject *object_;
  rviz::VectorProperty *position_property_;
  rviz::AngleProperty *heading_angle_property_;
  rviz::VectorProperty *velocity_property_;
  rviz::AngleProperty *velocity_angle_property_;
  rviz::VectorProperty *acceleration_property_;
  rviz::VectorProperty *size_property_;
  rviz::FloatProperty *existence_propability_property_;
  rviz::StringProperty *type_id_property_;
  rviz::StringProperty *meas_hist_property_;
  rviz::BoolProperty *measured_property_;

 public:
  IkaObjectVisualSelectionHandler(rviz::DisplayContext *context);
  ~IkaObjectVisualSelectionHandler() override;

  Ogre::Vector3 getPosition();
  Ogre::Vector3 getVelocity();
  Ogre::Vector3 getAcceleration();
  Ogre::Vector3 getSize();
  float getHeadingAngle();
  float getVelocityAngle();
  float getExistenceProbability();
  QString getTypeIdString();
  QString getSourceIdString();
  uint16_t getInternalId();
  bool getObjectMeasured();

  QString getMeasHistString();

  void createProperties(const rviz::Picked &obj, rviz::Property *parent_property) override;
  void updateProperties() override;

  const definitions::IkaObject *getObject() const;
  void setObject(const definitions::IkaObject *object);
};

}  // namespace rviz_plugin
