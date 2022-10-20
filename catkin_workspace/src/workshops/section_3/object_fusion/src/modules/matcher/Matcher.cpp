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

#include "Matcher.h"

#include "data/Params.h"
#include <definitions/utility/ika_utilities.h>
#include <utility/FusionUtility.h>

#include <utility>
#include <iostream>

Matcher::Matcher(std::shared_ptr<Data> data, std::string name)
    : AbstractFusionModule(std::move(data), name) {
  mahalanobis_calculator = new MahalanobisCalculator();
  iou_calculator = new IouCalculator();
  auction = new Auction();
}

void Matcher::runSingleSensor() {
  initializations();
  if (!num_measured_objects) return;

  computeDistanceMatrix();

  if (num_global_objects) {
    data_->associated_global = assignmentAlgorithm(distance_matrix);
  }
  else {
    trivialAssignmentAlgo();
  }
  
  selfAssignOnAlgoError();
  markAssignmentsInData();
  if (Params::get().cfg.chosen_distance_measure == 2) { // only for iou
    setValidity();
  }
}

void Matcher::initializations() {
  num_measured_objects = data_->object_list_measured.objects.size();
  num_global_objects = data_->object_list_fused.objects.size();

  data_->associated_measured = -Eigen::VectorXi::Ones(signed(num_global_objects)); // invalid
  data_->associated_global = -Eigen::VectorXi::Ones(signed(num_measured_objects)); // invalid
}

void Matcher::setValidity() {
  // make all invalid
  for (int idx = 0; idx < data_->object_list_fused.objects.size(); idx++) {
    data_->object_list_fused.objects[idx].bObjectValid = false;
  }

  if (data_->object_list_fused.objects.size() > 0) {
    // make only those ones valid that fused correctly based on associated_global
    for (int new_idx = 0; new_idx < data_->associated_global.size(); new_idx++) {
      int valid_idx = data_->associated_global[new_idx];
      data_->object_list_fused.objects[valid_idx].bObjectValid = true;
    }

    // unasigned objects
    bool has_no_iou;
    for (int j = 0; j < distance_matrix.cols(); j++) {
      has_no_iou = true;
      for (int i = 0; i < distance_matrix.rows(); i++) {
        if (distance_matrix(i,j) > -1) {
          has_no_iou = false;
        }
      }
      if (has_no_iou) {
          data_->object_list_fused.objects[j].bObjectValid = true;
      }
    }
  }
}

void Matcher::computeDistanceMatrix() {
  // default distance of -1 for objects not within threshold
  distance_matrix = -Eigen::MatrixXf::Ones(long(num_measured_objects), long(num_global_objects));

  for (unsigned long idx_measured = 0; idx_measured < num_measured_objects; idx_measured++) {
    const definitions::IkaObject& measuredObj = data_->object_list_measured.objects.at(idx_measured);
    if (!measuredObj.bObjectValid) continue;

    for (unsigned long idx_global = 0; idx_global < num_global_objects; idx_global++) {
      const GlobalObject& globalObj = data_->object_list_fused.objects.at(idx_global);
      if (!globalObj.bObjectValid) continue;

      float& dist= distance_matrix(signed(idx_measured), signed(idx_global));
      dist = computePairwiseDistance(measuredObj, globalObj);
    }
  }
}

float Matcher::computePairwiseDistance(const definitions::IkaObject& measuredObj, const GlobalObject& globalObj) {
  int chosen_distance_measure = Params::get().cfg.chosen_distance_measure;
  // std::cout << "chosen_distance_measure: " << chosen_distance_measure << " \n";


  switch (chosen_distance_measure) {
  case 1: // Mahalanobis distance
    return mahalanobis_calculator->extendedDistance(&measuredObj, &globalObj);
  case 2: // Intersection over union
    return iou_calculator->extendedDistance(&measuredObj, &globalObj);
  default: // default is Mahalanobis
    return mahalanobis_calculator->extendedDistance(&measuredObj, &globalObj);
  }
}

Eigen::VectorXi Matcher::assignmentAlgorithm(const Eigen::MatrixXf &distance_matrix) {
  int chosen_assignment_algo = 1;
  switch (chosen_assignment_algo) {
  case 1: // Auction algorithm
    return auction->matchMeasuredToGlobal(distance_matrix);
  case 2: // Hungarian / Munkres algorithm
    // todo: return mukres->matchMeasuredToGlobal(distance_matrix);
  default: // default is Auction
    return auction->matchMeasuredToGlobal(distance_matrix);
  }
    // std::cout << "distance_matrix init: " << " \n" << distance_matrix << " \n";
  
}

void Matcher::trivialAssignmentAlgo() {
  for (int idx_measured = 0; idx_measured < data_->associated_global.size(); idx_measured++) {
    data_->associated_global(idx_measured) = idx_measured;
  }
}

void Matcher::selfAssignOnAlgoError() {
  for (long idx_measured = 0; idx_measured < data_->associated_global.size(); idx_measured++) {

    // Check that all measured objects have been assigned. If not print and error and
    // make a manual self assigment.
    if (data_->associated_global(idx_measured) < 0) {
      // An object gets assigned to itself or rather a new object if the assigned
      // index exceed the number of global objects.
       data_->associated_global(idx_measured) = int(num_global_objects); // eliminate all -1 assignments
       std::cerr << "[ERROR] In Matcher: one measured object was neither assigned "
                 << "to a global object nor to itself. Continue with self assignment. "
                 << "(Global objects " << num_global_objects << ", measured objects "
                 << num_measured_objects << ")" << std::endl;
    }
  }
}


void Matcher::markAssignmentsInData() {

  // set all global objects to not measured. to be overwritten for true ones directly after
  for (size_t idx_global = 0; idx_global < data_->object_list_fused.objects.size(); idx_global++) {
    data_->object_list_fused.objects[idx_global].bObjectMeasured = false;
  }

  for (long idx_measured = 0; idx_measured < data_->associated_global.size(); idx_measured++) {

    definitions::IkaObject& measured_object = data_->object_list_measured.objects.at(unsigned(idx_measured));
    if (!measured_object.bObjectValid) {
      continue;
    }

    const int& idx_global_assigned = data_->associated_global(idx_measured);
    if (fusutil::successfulAssignment(idx_global_assigned, num_global_objects)) {
      data_->object_list_fused.objects[size_t(idx_global_assigned)].bObjectMeasured = true;
      data_->object_list_fused.objects[size_t(idx_global_assigned)].associated = true;

      data_->associated_measured[int(idx_global_assigned)] = int(idx_measured);
    }
  }
}
