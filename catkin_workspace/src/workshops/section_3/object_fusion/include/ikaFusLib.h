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

/**
 * @defgroup FUSION Ika Sensor Fusion
 * @brief Module for the sensor data object_fusion.
 * @addtogroup FUSION
 */
/**@{*/

#pragma once

#include "data/Data.h"
#include "data/ConfigParams.h"
#include "modules/AbstractFusionModule.h"

#include <definitions/IkaEgoMotion.h>
#include <definitions/IkaObjectList.h>

#include <definitions/utility/ika_utilities.h>

/**
 * @author Michael Hoss, Simon Schaefer
 * @brief Interfacing class for usage of the ika object_fusion. The user can only see this class, the AbstractFusionModule and
 * the Data class. Any access of data must be pipelined through this interface.
 */
class ikaFusLib {
 private:

  /**
   * @brief Pointer to the actual data storage used by any module.
   * @details This class is using a shared pointer because all modules are using
   * this instance and it shouldn't be deallocated during runtime.
   */
  std::shared_ptr<Data> data_;

  /**
   * @brief Pointers to the actual modules that are executed to do a full object_fusion cycle.
   * @attention All module will be executed in order. First added modules will be executed first.
   */
  std::vector<std::unique_ptr<AbstractFusionModule>> modules_;
 public:
  /**
   * @brief Pushes all modules in the right order into the modules_ list.
   * @details The order of modules_ should not be changed afterwards.
   */
  ikaFusLib();

  /**
   * @return true if no error occurred, false otherwise
   */
  bool initiateKalmanFilterConstants(Eigen::MatrixXf &constant_system_matrix,
                                         Eigen::MatrixXf &time_variant_system_matrix,
                                         Eigen::MatrixXf &process_noise_matrix);

  /**
   * @brief Loads all object_fusion constants from the given parameters.
   * @see FusionConstants::instance
   */
  void initiateFusionConstants(ConfigParams cfg_params);

  /**
   * @brief Fuses one measured object list into the global object list
   * @details Executes a full object_fusion cycle via ordered execution of the modules_.
   */
  definitions::IkaObjectList fuseIntoGlobal(
      const definitions::IkaObjectList& object_list_measured,
      const definitions::IkaEgoMotion& ego_motion);

private:
  void setPredictionGap();

  /**
   * @brief clear global object list if prediction time gap is too large or too negative
   */
  void handleTimeJumps();

  void resetGlobalObjectList();

  definitions::IkaObjectList getFusedOutputList();

  /**
   * @brief filter if a global object should be outputted
   * @return true if a global object should be outputted, false otherwise
   */
  bool outputCriterion(const GlobalObject& global_object);
};
/**@}*/
