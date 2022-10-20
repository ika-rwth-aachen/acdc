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

#include "ExistenceFuser.h"
#include "data/Params.h"

ExistenceFuser::ExistenceFuser(std::shared_ptr<Data> data, std::string name)
: AbstractFusionModule(data, name) {}

void ExistenceFuser::runSingleSensor() {
  // compute a weighted sum of existence probability values of measured and fused object

  definitions::IkaObjectList::_IdSource_type source_id = data_->object_list_measured.IdSource;
  float measuredWeight = Params::get().cfg.sensor_weights_object_existence[source_id];
  float fusedWeight = 1.0f;
  int count = -1;

  for (auto &fusedObject: data_->object_list_fused.objects) {
    ++count;
    int measurementIndex = data_->associated_measured[count];

    if (measurementIndex < 0){
      continue; // no associated measurement
    }

    // Add the loss rate again on top since the object has been measured again.
    if (data_->prediction_gap_in_seconds > 0.f) {
      fusedObject.fExistenceProbability +=
          Params::get().cfg.existence_probability_loss_rate *
          data_->prediction_gap_in_seconds;
      fusedObject.fExistenceProbability =
          fusutil::clamp(fusedObject.fExistenceProbability, 0.0f, 1.0f);
    }

    auto& measuredObject = data_->object_list_measured.objects[size_t(measurementIndex)];
    float sum_of_weights = measuredWeight * measuredObject.fExistenceProbability
                                + fusedWeight * fusedObject.fExistenceProbability;
    float total_weight = measuredWeight + fusedWeight;

    fusedObject.fExistenceProbability = sum_of_weights / total_weight;
  }
}
