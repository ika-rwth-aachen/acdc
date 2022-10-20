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

#include "modules/manager/ObjectCreator.h"
#include "utility/FusionUtility.h"
#include <definitions/utility/ika_utilities.h>
#include <iostream>

uint16_t ObjectCreator::globalObjectIdCounter = 1; // initialization of static member

ObjectCreator::ObjectCreator(std::shared_ptr<Data> data, std::string name)
  : AbstractFusionModule(std::move(data), name) {}

void ObjectCreator::runSingleSensor() {
  makeSpaceInAssociatedMeasured();
  for (size_t idx_measured = 0; idx_measured < data_->object_list_measured.objects.size(); ++idx_measured) {
    int idx_global_assigned = data_->associated_global(signed(idx_measured));
    unsigned long num_global_objects = data_->object_list_fused.objects.size();
    if (fusutil::successfulAssignment(idx_global_assigned, num_global_objects)) continue;

    const definitions::IkaObject& measured_object = data_->object_list_measured.objects[idx_measured];
    if (!objectSpawnCriterion(measured_object)) continue;

    size_t idx_global = insertNewGlobalObject(measured_object);
    rememberMatchInData(signed(idx_measured), signed(idx_global));
  }

  trimAssociatedMeasured();
}

void ObjectCreator::initializeProperties(definitions::IkaObject& new_global_object) {
  new_global_object.bObjectNew = true;
  new_global_object.bObjectMeasured = true;

  new_global_object.IdInternal = globalObjectIdCounter++;
  if (ObjectCreator::globalObjectIdCounter == USHRT_MAX) { // 65535 or greater
    std::cerr << "[Fusion matcher] Global object counter reached USHRT_MAX=65535. Overflow will make new IDs start at 0!" << std::endl;
  }

  new_global_object.IdExternal = definitions::env_model_outputs::FUSED_OBJECTS;
  new_global_object.birthStamp = new_global_object.header.stamp; // holds at time of birth, which is now

  initializeVariances(new_global_object);
}

void ObjectCreator::initializeVariances(definitions::IkaObject& new_global_object) {
  // remove negative variances from variance vector
  // initialize unknown global object variances to large positive values
  // this is important for Kalman Filter numerics
  auto eigenVarianceVec = IkaUtilities::getEigenVarianceVec(&new_global_object);
  for (long varCount = 0; varCount < eigenVarianceVec.size(); varCount++) {
    if((new_global_object.IdMotionModel == definitions::motion_model::CV) || 
        (new_global_object.IdMotionModel == definitions::motion_model::CA)) {
        if (eigenVarianceVec(varCount) < 0) {
          // Assign realistic max values for the entity's variances, especially
          // for the velocity.
          if (varCount == (int)definitions::ca_model::absVelX ||
              varCount == (int)definitions::ca_model::absVelY) {
            // Assign different velocity variances based on the object type.
            eigenVarianceVec(varCount) = Params::get().cfg.velocity_variances.at(new_global_object.IdType);
          } else if (varCount == (int)definitions::ca_model::absAccX ||
                    varCount == (int)definitions::ca_model::absAccY) {
            // Assign different acceleration variances based on the object type.
            eigenVarianceVec(varCount) = Params::get().cfg.acceleration_variances.at(new_global_object.IdType);
          } else {
            eigenVarianceVec(varCount) = Params::get().cfg.other_state_variables_variances.at(new_global_object.IdType);
          }
        }
      } else if((new_global_object.IdMotionModel == definitions::motion_model::CTRV) || 
              (new_global_object.IdMotionModel == definitions::motion_model::CTRA)) {
          // Assign realistic max values for the entity's variances, especially
          // for the velocity.
          if (varCount == (int)definitions::ctra_model::absVel ||
              varCount == (int)definitions::ctra_model::absVel) {
            // Assign different velocity variances based on the object type.
            eigenVarianceVec(varCount) = Params::get().cfg.velocity_variances.at(new_global_object.IdType);
          } else if (varCount == (int)definitions::ctra_model::absAcc ||
                    varCount == (int)definitions::ctra_model::absAcc) {
            // Assign different acceleration variances based on the object type.
            eigenVarianceVec(varCount) = Params::get().cfg.acceleration_variances.at(new_global_object.IdType);
          } else {
            eigenVarianceVec(varCount) = Params::get().cfg.other_state_variables_variances.at(new_global_object.IdType);
          }

      }
  }
  if (IkaUtilities::getObjectHeadingVar(new_global_object) < 0) {
    IkaUtilities::setObjectHeadingVar(new_global_object, Params::get().cfg.heading_variances.at(new_global_object.IdType));
  }
}

bool ObjectCreator::objectSpawnCriterion(const definitions::IkaObject& measured_object) {
  if (data_->object_list_measured.IdSource == definitions::input_sensors::RADAR) {
    // Object from not reliable sensors such as Radar, can be associated but
    // should not spawn a new object under every condition.
    // Do not allow the Radar to spawn:
    // 1. Cars if existence_prob is < 0.4 // TODO: Outsource such parameters to config file
    // 2. Pedestrians
    if (measured_object.IdType == definitions::ika_object_types::CAR && measured_object.fExistenceProbability < 0.4f) {
      return false;
    } else if (measured_object.IdType == definitions::ika_object_types::PEDESTRIAN) {
      return false;
    }
  }
  return true;
}

size_t ObjectCreator::insertNewGlobalObject(const definitions::IkaObject& measured_object) {
  definitions::IkaObject new_global_object = measured_object;
  initializeProperties(new_global_object);
  size_t idx_global_data = IkaUtilities::calculateFirstInvalidLocationInList(&data_->object_list_fused);
  GlobalObject global_object(new_global_object, IkaUtilities::getEigenVarianceVec(&new_global_object));

  // insert new global object into global object list
  if (idx_global_data >= data_->object_list_fused.objects.size()) {
    data_->object_list_fused.objects.push_back(global_object); // append
  } else {
    data_->object_list_fused.objects[idx_global_data] = global_object;  // overwrite existing gap
  }
  return idx_global_data;
}

void ObjectCreator::rememberMatchInData(int idx_measured, int idx_global) {
  data_->associated_measured[idx_global] = idx_measured;
  data_->associated_global[idx_measured] = idx_global;
}

void ObjectCreator::makeSpaceInAssociatedMeasured() {
  long global_objects_max_num = signed(data_->object_list_fused.objects.size())
                                + signed(data_->object_list_measured.objects.size());
  data_->associated_measured.conservativeResize(global_objects_max_num);
}

void ObjectCreator::trimAssociatedMeasured() {
  long new_num_global_objects = signed(data_->object_list_fused.objects.size());
  data_->associated_measured.conservativeResize(new_num_global_objects);
}
