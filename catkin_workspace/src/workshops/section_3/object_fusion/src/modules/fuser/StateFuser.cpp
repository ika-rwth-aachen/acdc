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

#include "StateFuser.h"
#include "data/Params.h"
#include <definitions/utility/ika_utilities.h>


StateFuser::StateFuser(std::shared_ptr<Data> data, std::string name)
: AbstractFusionModule(data, name) {}

Eigen::MatrixXf StateFuser::constructMeasurementMatrix() {

  // find valid object in measured object list
  Eigen::VectorXf variance;
  bool foundValidObject = false;
  for (auto &measuredObject: data_->object_list_measured.objects) {
    if (measuredObject.bObjectValid) {
      variance = IkaUtilities::getEigenVarianceVec(&measuredObject);
      foundValidObject = true;
    }
  }

  if (!foundValidObject) {
    return Eigen::MatrixXf::Zero(0,variance.size());
  }

  int row_size = 0;
  for (int i = 0; i < variance.size(); ++i) {
    if (variance[i] >= 0) {
      ++row_size;
    }
  }
  if (row_size == 0) return Eigen::MatrixXf::Zero(0, variance.size());

  Eigen::MatrixXf C = Eigen::MatrixXf::Zero(long(row_size), variance.size());

  int row = 0;
  for (int col = 0; col < variance.size() && row < row_size; ++col) {
    if (variance[col] >= 0) {
      C(row, col) = 1;
      row++;
    }
  }
  return C;
}


void StateFuser::runSingleSensor() {

  Eigen::MatrixXf C = constructMeasurementMatrix();

  int count = -1;
  for (auto &globalObject : data_->object_list_fused.objects) {
    count++;

    auto x_hat_G = IkaUtilities::getEigenStateVec(&globalObject); // predicted global state

    int measurementIndex = data_->associated_measured[count];

    if (measurementIndex < 0) {
      continue; // no associated measurement
    }

    definitions::IkaObject& measuredObject = data_->object_list_measured.objects[measurementIndex];
    auto P_S_diag = IkaUtilities::getEigenVarianceVec(&measuredObject); // predicted measured state variance

    /** START TASK 3 CODE HERE **/


    /** END TASK 3 CODE HERE **/
  }
}
