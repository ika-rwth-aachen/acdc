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

#include <utility>

#include "HeadingAngleFuser.h"
#include "data/Params.h"
#include <definitions/utility/ika_utilities.h>



HeadingAngleFuser::HeadingAngleFuser(std::shared_ptr<Data> data, std::string name)
    : AbstractFusionModule(std::move(data), name) {}

void HeadingAngleFuser::runSingleSensor() {

  int count = -1;
  for (auto &fusedObject: data_->object_list_fused.objects) {
    ++count;
    int measurementIndex = data_->associated_measured[count];
    if (measurementIndex < 0) {
      continue; // no associated measurement
    }
    auto& measuredObject = data_->object_list_measured.objects[measurementIndex];

    if (IkaUtilities::getObjectHeadingVar(measuredObject) < 0.f) {
      // heading angle not measured: keep current global value
      // position[0] = myIkaObj.fMean[(int)definitions::ca_model::posX];


      continue;
    }

    if (IkaUtilities::getObjectHeadingVar(fusedObject) > 1e2f) {
      // global object has never been updated with a measured heading angle
      // -> initialize with measured one
      IkaUtilities::setObjectHeading(fusedObject, IkaUtilities::getObjectHeading(measuredObject));
      IkaUtilities::setObjectHeadingVar(fusedObject, IkaUtilities::getObjectHeadingVar(measuredObject));

      continue;
    }

    // scalar Kalman filter impelementation
    IkaUtilities::setObjectHeadingVar(fusedObject, IkaUtilities::getObjectHeadingVar(fusedObject) + 0.1 * M_PI);
    float K = IkaUtilities::getObjectHeadingVar(fusedObject) / (IkaUtilities::getObjectHeadingVar(fusedObject) + 
                IkaUtilities::getObjectHeadingVar(measuredObject));

    float globalSin = std::sin(IkaUtilities::getObjectHeading(fusedObject)) 
                      + K * (std::sin(IkaUtilities::getObjectHeading(measuredObject))
                      - std::sin(IkaUtilities::getObjectHeading(fusedObject)));

    float globalCos = std::cos(IkaUtilities::getObjectHeading(fusedObject)) 
                      + K * (std::cos(IkaUtilities::getObjectHeading(measuredObject))
                      - std::cos(IkaUtilities::getObjectHeading(fusedObject)));

    IkaUtilities::setObjectHeading(fusedObject, std::atan2(globalSin, globalCos));
    IkaUtilities::setObjectHeadingVar(fusedObject, (1.f - K) * IkaUtilities::getObjectHeadingVar(fusedObject));
  }
}
