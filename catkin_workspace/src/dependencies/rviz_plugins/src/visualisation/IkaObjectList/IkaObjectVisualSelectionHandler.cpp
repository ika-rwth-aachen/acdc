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
#include "IkaObjectVisualSelectionHandler.h"

namespace rviz_plugin {

IkaObjectVisualSelectionHandler::IkaObjectVisualSelectionHandler(rviz::DisplayContext *context)
    : rviz::SelectionHandler(context) {}

IkaObjectVisualSelectionHandler::~IkaObjectVisualSelectionHandler() {
  //  properties_.clear();
  //  delete position_property_;
  //  delete heading_angle_property_;
  //  delete velocity_angle_property_;
  //  delete velocity_property_;
  //  delete acceleration_property_;
  //  delete type_id_property_;
  //  delete size_property_;
  //  delete existence_propability_property_;
}

Ogre::Vector3 IkaObjectVisualSelectionHandler::getPosition() {
  if (object_ == nullptr) return Ogre::Vector3(0, 0, 0);

  float posX, posY, posZ;
  if((object_->IdMotionModel == definitions::motion_model::CV) || 
       (object_->IdMotionModel == definitions::motion_model::CA)) {
    posX = object_->fMean[(int)definitions::ca_model::posX];
    posY = object_->fMean[(int)definitions::ca_model::posY];
    posZ = object_->fMean[(int)definitions::ca_model::posZ];
  } else if((object_->IdMotionModel == definitions::motion_model::CTRV) || 
            (object_->IdMotionModel == definitions::motion_model::CTRA)) {
    posX = object_->fMean[(int)definitions::ctra_model::posX];
    posY = object_->fMean[(int)definitions::ctra_model::posY];
    posZ = object_->fMean[(int)definitions::ctra_model::posZ];
  }
  
  return Ogre::Vector3(posX, posY, posZ);
}

Ogre::Vector3 IkaObjectVisualSelectionHandler::getVelocity() {
  if (object_ == nullptr) return Ogre::Vector3(0, 0, 0);

  float absVelX, absVelY;
  if((object_->IdMotionModel == definitions::motion_model::CV) || 
      (object_->IdMotionModel == definitions::motion_model::CA)) {
    absVelX = object_->fMean[(int)definitions::ca_model::absVelX];
    absVelY = object_->fMean[(int)definitions::ca_model::absVelY];
  } else if((object_->IdMotionModel == definitions::motion_model::CTRV) || 
            (object_->IdMotionModel == definitions::motion_model::CTRA)) {
    absVelX = object_->fMean[(int)definitions::ctra_model::absVel] *
              std::cos(object_->fMean[(int)definitions::ctra_model::heading]);
    absVelY = object_->fMean[(int)definitions::ctra_model::absVel] *
              std::sin(object_->fMean[(int)definitions::ctra_model::heading]);
  }

  return Ogre::Vector3(absVelX, absVelY, 0);
}

Ogre::Vector3 IkaObjectVisualSelectionHandler::getAcceleration() {
  if (object_ == nullptr) return Ogre::Vector3(0, 0, 0);

  float absAccX, absAccY;
  if((object_->IdMotionModel == definitions::motion_model::CV) || 
      (object_->IdMotionModel == definitions::motion_model::CA)) {
    absAccX = object_->fMean[(int)definitions::ca_model::absAccX];
    absAccY = object_->fMean[(int)definitions::ca_model::absAccY];
  } else if((object_->IdMotionModel == definitions::motion_model::CTRV) || 
            (object_->IdMotionModel == definitions::motion_model::CTRA)) {
    absAccX = object_->fMean[(int)definitions::ctra_model::absAcc] *
              std::cos(object_->fMean[(int)definitions::ctra_model::heading]);
    absAccY = object_->fMean[(int)definitions::ctra_model::absAcc] *
              std::sin(object_->fMean[(int)definitions::ctra_model::heading]);
  }

  return Ogre::Vector3(absAccX, absAccY, 0);
}

Ogre::Vector3 IkaObjectVisualSelectionHandler::getSize() {
  if (object_ == nullptr) return Ogre::Vector3(0, 0, 0);

  float length, width, height;
  if((object_->IdMotionModel == definitions::motion_model::CV) || 
      (object_->IdMotionModel == definitions::motion_model::CA)) {
    length = object_->fMean[(int)definitions::ca_model::length];
    width = object_->fMean[(int)definitions::ca_model::width];
    height = object_->fMean[(int)definitions::ca_model::height];
  } else if((object_->IdMotionModel == definitions::motion_model::CTRV) || 
            (object_->IdMotionModel == definitions::motion_model::CTRA)) {
    length = object_->fMean[(int)definitions::ctra_model::length];
    width = object_->fMean[(int)definitions::ctra_model::width];
    height = object_->fMean[(int)definitions::ctra_model::height];
  }

  return Ogre::Vector3(length, width, height);
}

float IkaObjectVisualSelectionHandler::getHeadingAngle() {
  if (object_ == nullptr) return 0.0f;

  float fHeading;
  if((object_->IdMotionModel == definitions::motion_model::CV) || 
      (object_->IdMotionModel == definitions::motion_model::CA)) {
    fHeading = object_->fMean[(int)definitions::ca_model::heading];
  } else if((object_->IdMotionModel == definitions::motion_model::CTRV) || 
            (object_->IdMotionModel == definitions::motion_model::CTRA)) {
    fHeading = object_->fMean[(int)definitions::ctra_model::heading];
  }

  return fHeading;
}

float IkaObjectVisualSelectionHandler::getVelocityAngle() {
  if (object_ == nullptr) return 0.0f;

  float absVelX, absVelY;
  if((object_->IdMotionModel == definitions::motion_model::CV) || 
      (object_->IdMotionModel == definitions::motion_model::CA)) {
    absVelX = object_->fMean[(int)definitions::ca_model::absVelX];
    absVelY = object_->fMean[(int)definitions::ca_model::absVelY];
  } else if((object_->IdMotionModel == definitions::motion_model::CTRV) || 
            (object_->IdMotionModel == definitions::motion_model::CTRA)) {
    absVelX = object_->fMean[(int)definitions::ctra_model::absVel] *
              std::cos(object_->fMean[(int)definitions::ctra_model::heading]);
    absVelY = object_->fMean[(int)definitions::ctra_model::absVel] *
              std::sin(object_->fMean[(int)definitions::ctra_model::heading]);
  }

  return static_cast<float>(-std::atan2(absVelX, absVelY)+M_PI/2);
}

float IkaObjectVisualSelectionHandler::getExistenceProbability() {
  if (object_ == nullptr) return 0.0f;
  return object_->fExistenceProbability;
}

QString IkaObjectVisualSelectionHandler::getTypeIdString() {
  if (object_ == nullptr) return QString::fromStdString(nullptr);
  return QString::fromStdString(IkaUtilities::typeToName(static_cast<uint16_t>(object_->IdType)));
}

QString IkaObjectVisualSelectionHandler::getSourceIdString() {
  if (object_ == nullptr) return QString::fromStdString(nullptr);
  return QString::fromStdString(IkaUtilities::sourceToName(static_cast<uint16_t>(object_->IdExternal)));
}

uint16_t IkaObjectVisualSelectionHandler::getInternalId() {
  if (object_ == nullptr) return 0;
  return object_->IdInternal;
}

QString IkaObjectVisualSelectionHandler::getMeasHistString() {
  QString myString = "";
  if (object_ == nullptr) return myString;

  for (int measHistIter = 0; measHistIter < object_->measHist.size(); measHistIter++) {
    auto& measHistEntry = object_->measHist.at(unsigned(measHistIter));
    QString sensorName = QString::fromStdString(IkaUtilities::sourceToName(static_cast<uint16_t>(measHistEntry.IdSensor)));
    myString.append(sensorName);
//    myString.append("=");
    myString.append(QString::number(measHistEntry.IdObjectWithinSensor));
    myString.append("|");
  }
  myString.chop(1);
  return myString;
}

bool IkaObjectVisualSelectionHandler::getObjectMeasured() {
  if (object_ == nullptr) return false;
  return object_->bObjectMeasured;
}

void IkaObjectVisualSelectionHandler::createProperties(const rviz::Picked &obj, rviz::Property *parent_property) {
  SelectionHandler::createProperties(obj, parent_property);

  uint16_t internal_id = getInternalId();
  QString objectCaption = "Object " + getSourceIdString() + " " + QString::number(internal_id);

  rviz::Property *object_group = new rviz::Property(objectCaption, QVariant(), "", parent_property);
  properties_.push_back(object_group);

  position_property_ = new rviz::VectorProperty("Position", getPosition(), "", object_group);
  position_property_->setReadOnly(true);

  heading_angle_property_ = new rviz::AngleProperty("Heading angle", getHeadingAngle(), "", object_group);
  heading_angle_property_->setReadOnly(true);

  velocity_angle_property_ = new rviz::AngleProperty("Velocity angle", getVelocityAngle(), "", object_group);
  velocity_angle_property_->setReadOnly(true);

  velocity_property_ = new rviz::VectorProperty("Velocity", getVelocity(), "", object_group);
  velocity_property_->setReadOnly(true);

  acceleration_property_ = new rviz::VectorProperty("Acceleration", getAcceleration(), "", object_group);
  acceleration_property_->setReadOnly(true);

  type_id_property_ = new rviz::StringProperty("Type Id", getTypeIdString(), "", object_group);
  type_id_property_->setReadOnly(true);

  size_property_ = new rviz::VectorProperty("Size", getSize(), "", object_group);
  size_property_->setReadOnly(true);

  existence_propability_property_ = new rviz::FloatProperty("Existence probability", getExistenceProbability(), "", object_group);
  existence_propability_property_->setReadOnly(true);

  meas_hist_property_ = new rviz::StringProperty("Meas. Hist", getMeasHistString(), "", object_group);
  meas_hist_property_->setReadOnly(true);

  measured_property_ =
      new rviz::BoolProperty("Measured", getObjectMeasured(), "", object_group);
  measured_property_->setReadOnly(true);

  //  object_group->expand();
}

void IkaObjectVisualSelectionHandler::updateProperties() {
  SelectionHandler::updateProperties();
  position_property_->setVector(getPosition());
  heading_angle_property_->setFloat(getHeadingAngle());
  velocity_property_->setVector(getVelocity());
  velocity_angle_property_->setFloat(getVelocityAngle());
  acceleration_property_->setVector(getAcceleration());
  type_id_property_->setString(getTypeIdString());
  size_property_->setVector(getSize());
  existence_propability_property_->setFloat(getExistenceProbability());
  meas_hist_property_->setString(getMeasHistString());
  measured_property_->setBool(getObjectMeasured());
}



const definitions::IkaObject *IkaObjectVisualSelectionHandler::getObject() const {
  return object_;
}

void IkaObjectVisualSelectionHandler::setObject(const definitions::IkaObject *object) {
  object_ = object;
}

}  // namespace rviz_plugin
