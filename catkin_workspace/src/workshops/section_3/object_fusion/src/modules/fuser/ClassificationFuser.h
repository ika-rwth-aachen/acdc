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

#include "modules/AbstractFusionModule.h"

/**
 * @authors Michael Hoss, Simon Schaefer
 * @brief Fuser for the classification of a global object.
 */
class ClassificationFuser : public AbstractFusionModule {
 public:
  ClassificationFuser(std::shared_ptr<Data> data, std::string name);

  /**
   * @brief Overwriting the global classificatoin with the measured classification.
   * @details Keep previous global classification if measured classification is unknown.
   * @todo Improve this with Dempster Shafer theory for classification or similar.
   */
  void runSingleSensor() override;
};

/**@}*/
