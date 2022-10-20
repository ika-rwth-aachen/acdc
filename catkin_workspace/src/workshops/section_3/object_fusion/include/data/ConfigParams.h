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
#include <map>
#include <utility>

/**
 * @brief Summarizes fusion parameters from external config files into a single struct.
 * @details  This struct definition is also visible at the library interface.
 * Like this, a struct is given to the library instead of invidual variables.
 * --> Shortens function signatures.
 */
struct ConfigParams {
  // PREDICTOR
  float existence_probability_loss_rate;

  // MATCHER
  int chosen_distance_measure;
  float mahalanobis_threshold;
  float iou_overlap_threshold;
  float mahalanobis_global_threshold;

  std::map<int, int> velocity_variances;
  std::map<int, int> acceleration_variances;
  std::map<int, int> other_state_variables_variances;
  std::map<int, int> heading_variances;
  std::map<std::pair<int, int>, float> mahalanobis_penalty;

  // FUSER
  std::map<int, float> sensor_weights_object_existence;

  // MANAGER
  float existence_probability_output_threshold;
  float existence_probability_delete_threshold;
  double measurement_history_threshold_sec;

  // OTHER
  float time_jump_forward_sec;
  float time_jump_backward_sec;
};

/**@}*/
