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

#include <Eigen/Dense>
#include <definitions/IkaObject.h>
#include <definitions/utility/object_definitions.h>

/**
 * @brief Maximum number of iterations by matcher algorithm.
 * If no association is found by then, no assiciation is made.
 */
#define MAX_ITERATIONS 1000

/**
 * @brief Associate the measured object list to the global object list.
 * @details This algorithm is described in the dissertation "Object-Level Fusion for Surround
 * Environment Perception in Automated Driving Applications" (2017)
 */
class Auction {
public:
  Auction();

  /**
   * @brief matchMeasuredToGlobal
   * @param distance_matrix_A pairwise distances between measured objects (rows)
   * and global objects (columns)
   * @return assignment_vector_r takes measured idx and returns global idx
   */
  Eigen::VectorXi matchMeasuredToGlobal(const Eigen::MatrixXf& distance_matrix_A);

private:

  Eigen::MatrixXf distance_matrix_A; /**< @brief input to algorithm. see Aeberhardt thesis */
  Eigen::VectorXi assignment_vector_r; /**< @brief output of algorithm. see Aeberhardt thesis */
  Eigen::MatrixXf cost_matrix_C; /**< @brief see Aeberhardt thesis */
  Eigen::RowVectorXf bid_price_vector_p; /**< @brief see Aeberhardt thesis */

  unsigned long num_measured_objects; /**< @brief auxiliary variable */
  unsigned long num_global_objects; /**< @brief auxiliary variable */

  void initialize_p_r(); /**< @brief see Aeberhardt thesis */
  void computeCostMatrixC(); /**< @brief see Aeberhardt thesis */

  /**
   * @brief Steps 1-5 of auction algorithm from Aeberhardt thesis
   * @details Lets measured objects bid for global objects until there is a 1-to-1 assignment.
   * This auction runs until all measured objects are assigned or the maximum loop count is reached.
   * Self-assignment of measured objects if no global object is found for them.
   */
  void iterativeAuctionAlgo();

  // functions to be called from within iterativeAuctionAlgo()
  /**
   * @brief Step 1 of auction algorithm in Aeberhardt thesis
   * @return index of first object in measured list that has not been assigned to a global object yet.
   */
  int getFirstUnassignedMeasured();
  /**
   * @brief Step 2 of auction algorithm in Aeberhardt thesis
   * @details Compute the benefit that a measured object has if it buys a global object
   *
   * @param idx_measured_bidder index of measured object that is the bidder / buyer
   * @param benefit_diff (return) difference between maximum benefit and second largest benefit
   * @param benefit_max_coeff (return) index of global object that yields maximum benefit to measured object
   */
  void getMaxAnd2ndMaxBenefit(const int idx_measured_bidder,
                              float& benefit_diff,
                              int& benefit_max_coeff);
  /**
   * @brief Step 3 of auction algorithm in Aeberhardt thesis
   * @details remove a potential previous assignment and makes a new assignment based on the current auction iteration
   *
   * @param idx_measured_bidder index of measured object to be assigned
   * @param idx_global_max_benefit index of global object to be assigned
   */
  void arrangeNewAssignment(const int idx_measured_bidder,
                            const int idx_global_max_benefit);
  /**
   * @brief Step 4 of auction algorithm in Aeberhardt thesis
   *
   * @param idx_global_max_benefit index of global object whose price gets increased
   * @param benefit_diff difference between max and 2ndmax benefit, about which the bid price is increased.
   */
  void increaseBidPrice(const int idx_global_max_benefit,
                        const float& benefit_diff);

};
