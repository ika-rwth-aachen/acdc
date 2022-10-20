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
#include <ros/ros.h>
#include <Eigen/Dense>
#include <definitions/utility/ika_utilities.h>
#include "data/ConfigParams.h"

/**
 * @authors Michael Hoss, Simon Schaefer
 * @brief Global storage for all configurable values needed by the object_fusion library.
 * @details Follows the singleton design principle / "local static object".
 */
class Params {
 private:
  Params();
 public:
  ConfigParams cfg;

  /**
   * @brief Loads the configuration parameters into the class.
   * @details This function must be only called once on startup.
   */
  void setParamsFromConfig(ConfigParams cfg);

  /**
   * @brief Due to the private constructor, this is the only way to get the single local static instance of this class.
   * @return The single instance of that class.
   */
  static Params &get() { // "static" means that method can be called without class instance
    static Params singleInstance; // "static" means that singleInstance exists only once for all function calls
    return singleInstance;
  }
};
/**@}*/
