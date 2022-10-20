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

#include <memory>

#include "data/Data.h"
#include <definitions/utility/global_object.h>
#include <definitions/utility/object_definitions.h>

/**
 * @author Simon Schaefer
 * @attention If any data should be used by all or many modules in the object_fusion pipeline, add them here!
 * @brief Abstract base class for all modules in the object_fusion pipeline.
 * @details This inheritance simplifies usage of the object_fusion modules:
 * Each module has the same pointer to the global data storage.
 * Also, each module implements their tasks in the same runner function.
 */
class AbstractFusionModule {
 protected:
  /*
   * Place pointers/values that should be available for all modules here!
   */

  /**
   * @brief Shared pointer to data storage for object lists.
   */
  std::shared_ptr<Data> data_ = nullptr;

 public:
  /**
   * @brief Constructor to initiate the data_ storage
   * @attention Will by called by all child classes. If something should be done
   * by all modules in the object_fusion pipeline, do it here!
   * @param data Global data storage of the all modules.
   */
  AbstractFusionModule(std::shared_ptr<Data> data, std::string name);

  /**
   * @brief Module execution function to be implemented by all derived classes / modules.
   * @details The only interface called by the pipeline. Will be executed once upon measured object list arrival.
   * @attention Keep in mind that any other module executed before could have changed the data_ already.
   */
  virtual void runSingleSensor() = 0;

  /**
   * @brief name of this fusion module. Useful for debugging.
   */
  std::string name_;

};
/**@}*/
