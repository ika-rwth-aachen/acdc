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

/** @addtogroup FUSION */
/**@{*/

#pragma once

#include <vector>

#include <Eigen/Dense>

#include <definitions/IkaEgoMotion.h>
#include <definitions/IkaObjectList.h>

#include <definitions/utility/global_object.h>

/**
 * @authors Michael Hoss, Simon Schaefer
 * @brief Global data storage.
 * @details This class stores all object lists and matrices which have to have global accessibility.
 * @todo Is storing all variables public really the best way to do?
 */
class Data {
 public:

  definitions::IkaEgoMotion ego_motion; ///< @brief motion of the ego vehicle that has to be compensated

  definitions::IkaObjectList object_list_measured; ///< @brief input list of measured objects
  GlobalObjectList object_list_fused; ///< @brief output list of fused / global objects

  float prediction_gap_in_seconds = 0.0f;  ///< @brief stamp_measurements - stamp_previous_global in seconds

  Eigen::MatrixXf F_const_; ///< @brief constant part of motion model matrix
  Eigen::MatrixXf F_timevar_; ///< @brief delta-time-variant part of motion model matrix
  Eigen::MatrixXf Q_timevar_; ///< @brief process noise matrix

  /**
   * @brief matched_measured_obj_idx = associated_measured(matched_global_obj_idx)
   */
  Eigen::VectorXi associated_measured;

  /**
   * @brief matched_global_obj_idx = associated_global(matched_measured_obj_idx)
   */
  Eigen::VectorXi associated_global;

 public:
  Data();

};

/**@}*/
