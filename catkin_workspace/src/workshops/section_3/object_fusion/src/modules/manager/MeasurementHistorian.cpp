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

#include "MeasurementHistorian.h"

#include "data/Params.h"

#include <definitions/IkaObjectList.h>
#include <definitions/utility/ika_utilities.h>

MeasurementHistorian::MeasurementHistorian(std::shared_ptr<Data> data, std::string name)
    : AbstractFusionModule(data, name) {}

void MeasurementHistorian::runSingleSensor() {
  int global_idx = 0;
  for (auto& global_object : data_->object_list_fused.objects) {
    int measured_idx = data_->associated_measured[global_idx];
    if (measured_idx >= 0) {
      const auto& measured_object = data_->object_list_measured.objects[unsigned(measured_idx)];
      representMeasuredObject(global_object, measured_object);
    }

    clipMeasurementHistory(global_object);
    global_idx++;
  }
}


void MeasurementHistorian::representMeasuredObject(GlobalObject& global_object,
                                                   const definitions::IkaObject& measured_object) {
  bool sensorAlreadyRepresented = false;
  for (auto& globalHistoryStamp : global_object.measHist) {
    if (measured_object.IdExternal == globalHistoryStamp.IdSensor) {
      // update global history stamp in place with latest measurement stamp
      globalHistoryStamp = measured_object.measHist[0];
      sensorAlreadyRepresented = true;
      break;
    }
  }

  if (!sensorAlreadyRepresented) {
    assert(measured_object.measHist.size() > 0 && "No history for sensor specified!");
    global_object.measHist.push_back(measured_object.measHist[0]);
  }
}

void MeasurementHistorian::clipMeasurementHistory(GlobalObject& global_object) {
  auto& mostRecentStamp = global_object.header.stamp;
  auto globalHistoryStamp = global_object.measHist.begin();

  while (globalHistoryStamp != global_object.measHist.end()) {

    ros::Duration measurement_age = mostRecentStamp - globalHistoryStamp->measuredStamp;
    if (measurement_age.toSec() > Params::get().cfg.measurement_history_threshold_sec) {
      global_object.measHist.erase(globalHistoryStamp); // remove outdated measurement
    }
    else {
      globalHistoryStamp++;
    }
  }
}
