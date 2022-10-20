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

#include "Auction.h"

#include "data/Params.h"
#include <definitions/utility/ika_utilities.h>

#include <utility>
#include <iostream>

Auction::Auction() {}

Eigen::VectorXi Auction::matchMeasuredToGlobal(const Eigen::MatrixXf &distance_matrix_A)
{
  this->distance_matrix_A = distance_matrix_A;
  this->num_measured_objects = unsigned(distance_matrix_A.rows());
  this->num_global_objects = unsigned(distance_matrix_A.cols());

  computeCostMatrixC();
  initialize_p_r();
  iterativeAuctionAlgo();
  return assignment_vector_r;
}

void Auction::computeCostMatrixC() {

  float& mahalanobis_threshold = Params::get().cfg.mahalanobis_threshold;
  Eigen::MatrixXf cost_matrix_C_a = 2.0f * mahalanobis_threshold
                                    * Eigen::MatrixXf::Ones(distance_matrix_A.rows(), distance_matrix_A.cols())
                                    - distance_matrix_A;
  // row-major storage in eigen --> loop order
  for (long j = 0; j < distance_matrix_A.cols(); j++) {
    for (long i = 0; i < distance_matrix_A.rows(); i++) {
      if (distance_matrix_A(i,j) < 0) {
        cost_matrix_C_a(i, j) = 0.f; // no cost or worth of a pair if it did not fulfill the threshold
      }
    }
  }

  Eigen::MatrixXf cost_matrix_C_b = mahalanobis_threshold * Eigen::MatrixXf::Identity(cost_matrix_C_a.rows(), cost_matrix_C_a.rows());

  cost_matrix_C = Eigen::MatrixXf(num_measured_objects, num_global_objects + num_measured_objects);
  if (num_global_objects) {
    cost_matrix_C << cost_matrix_C_a, cost_matrix_C_b;
  }
  else {
    cost_matrix_C = cost_matrix_C_b;
  }
}

void Auction::initialize_p_r() {
  bid_price_vector_p = Eigen::VectorXf::Zero(long(num_global_objects + num_measured_objects));
  assignment_vector_r = -Eigen::VectorXi::Ones(long(num_measured_objects)); // initialize to invalid (-1)
}


void Auction::iterativeAuctionAlgo() {
  // step numbers in comments refer to Aeberhardt thesis page 62
  bool successfulTermination = false;
  for (size_t auction_iter = 0; auction_iter < MAX_ITERATIONS; auction_iter++) {

    int idx_measured_bidder = getFirstUnassignedMeasured(); // step 1

    if (idx_measured_bidder < 0) { // all objects have been assigned
      successfulTermination = true;
      break; // step 5
    }

    float benefit_diff = 0.f;
    int benefit_max_coeff = -1;
    getMaxAnd2ndMaxBenefit(idx_measured_bidder, benefit_diff, benefit_max_coeff); // step 2
    arrangeNewAssignment(idx_measured_bidder, benefit_max_coeff); // step 3
    increaseBidPrice(benefit_max_coeff, benefit_diff); // step 4
  }
  if(!successfulTermination){
     std::cerr << "[ERROR] Maximum number of iterations in association process is reached." << std::endl;
  }
}


int Auction::getFirstUnassignedMeasured() {

  for (int idx_measured = 0; idx_measured < assignment_vector_r.size(); ++idx_measured) {
    if (assignment_vector_r(idx_measured) < 0) {
      return idx_measured;
    }
  }
  return -1; // code reaches here if all measured objects have been assigned
}

void Auction::getMaxAnd2ndMaxBenefit(const int idx_measured_bidder,
                                     float& benefit_diff,
                                     int& benefit_max_coeff) {
  // benefit_max
  // ATTENTION: there is an error in Aeberhard thesis. benefit is computed from cost_matrix_C, not distance_matrix_A!
  Eigen::VectorXf benefit_vector = cost_matrix_C.row(idx_measured_bidder) - bid_price_vector_p;
  float benefit_max = benefit_vector.maxCoeff(&benefit_max_coeff);

  // benefit_secmax
  benefit_vector(benefit_max_coeff) = 0.f; // remove maximum value to be able to find 2nd max
  float benefit_secmax = benefit_vector.maxCoeff();

  benefit_diff = benefit_max - benefit_secmax;
}

void Auction::arrangeNewAssignment(const int idx_measured_bidder,
                                   const int idx_global_max_benefit) {

  // remove potential previous assignment of another measured object
  for (int iter_measured = 0; iter_measured < assignment_vector_r.size(); iter_measured++) {
    if (assignment_vector_r(iter_measured) == idx_global_max_benefit) {
      assignment_vector_r(iter_measured) = -1;
      break;
    }
  }
  assignment_vector_r(idx_measured_bidder) = idx_global_max_benefit; // make new assignment
}

void Auction::increaseBidPrice(const int idx_global_max_benefit,
                               const float& benefit_diff) {
  float eps = 1e-5f; // tolerance value for deviation of algorithm from optimal solution
  bid_price_vector_p(idx_global_max_benefit) += benefit_diff + eps; // increase offer price due to current bid
}
