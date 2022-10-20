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

#include <cassert>
#include <ros/time.h>

/**
 * @brief Utility to define global functions that are used by multiple classes.
 */
namespace fusutil {

  /**
   * @brief Since std::clamp is only available starting from C++17, this is an
   * implementation to clamp a value.
   *
   * @tparam T Type of the value that should be clamped (make sure the < operator
   * is defined)
   * @param v Value to be clamped.
   * @param lo Lower boundary.
   * @param hi Upper boundary. Make sure hi > lo
   * @return constexpr const T& Reference to the solution value of the clamp
   * operation.
   */
  template <class T>
  constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    assert(!(hi < lo));
    return (v < lo) ? lo : (hi < v) ? hi : v;
  }

  uint64_t ROSstamp2unsignedlong_ms(ros::Time ros_stamp);
  bool successfulAssignment(int idx_global_assigned, unsigned long num_global_objects);
}


/**@}*/
