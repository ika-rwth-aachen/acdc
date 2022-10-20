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

#include <Eigen/Dense>
#include <definitions/IkaObjectList.h>
#include <definitions/utility/object_definitions.h>
#include <definitions/utility/ika_utilities.h>
#include "modules/AbstractFusionModule.h"
#include "data/Params.h"

class ObjectCreator : public AbstractFusionModule
{
public:
  ObjectCreator(std::shared_ptr<Data> data, std::string name);
  void runSingleSensor() override;

private:
  static uint16_t globalObjectIdCounter; /**< @brief counter for unique ID of new global objects */

  bool successfullyAssigned(size_t idx_measured);

  bool objectSpawnCriterion(const definitions::IkaObject& measured_object);

  /**
   * @brief create and insert a new global object into the fusion data storage
   * @details execution for a self-assigned measured object.
   * based on measured_object, the new global object is initialized using @see initializeNewGlobalObject.
   * The new global object is inserted into a gap or appended to the global object list.
   *
   * @param measured_object self-assigned measured object that gives birth to a new global object
   * @return index of new global object within fusion data storage object list
   */
  size_t insertNewGlobalObject(const definitions::IkaObject& measured_object);

  /**
   * @brief modifications such that the new object behaves well as a global object
   * @details set variances of unnknown states to large positive values
   */
  void initializeProperties(definitions::IkaObject& new_global_object);

  /**
   * @brief set variances of unnknown states to large positive values
   */
  void initializeVariances(definitions::IkaObject& new_global_object);

  void rememberMatchInData(int idx_measured, int idx_global);

  /**
   * @brief enlarging data_->associated_measured to host potentially all measured
   * objects also as global objects
   */
  void makeSpaceInAssociatedMeasured();
  /**
   * @brief trimming data_->associated_measured to actual new size of global object list
   */
  void trimAssociatedMeasured();
};
