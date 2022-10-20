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
#include "modules/AbstractFusionModule.h"
#include "modules/matcher/distance_measures/Mahalanobis.h"
#include "modules/matcher/distance_measures/IntersectionOverUnion.h"
#include "modules/matcher/algorithms/Auction.h"

/**
 * @authors Michael Hoss, Simon Schaefer
 * @brief Associate measured objects to global objects.
 * @details Possible distance measures between objects: Mahalanobis distance.
 * Possible algorithms: Auction
 */
class Matcher : public AbstractFusionModule {
 private:

  unsigned long num_measured_objects; /**< @brief is set to measured object list size at beginning of each cycle */
  unsigned long num_global_objects; /**< @brief is set to global object list size at beginning of each cycle */
  Eigen::MatrixXf distance_matrix; /**< @brief input to algorithms */

  MahalanobisCalculator *mahalanobis_calculator;
  IouCalculator *iou_calculator;
  Auction *auction;

  /**
   * @brief initialize association outputs in fusion data and define auxiliary variables here
   */
  void initializations();
  void computeDistanceMatrix(); /**< @brief pairwise distances between measured (rows) and global (columns) objects */
  float computePairwiseDistance(const definitions::IkaObject& measuredObj, const GlobalObject& globalObj);
  void setValidity();

  /**
   * @brief Wrapper that calls the assignment algorithm of choice.
   * @return a vector that returns the corresponding global object indices for measured objects (or -1 for unmatched)
   */
  Eigen::VectorXi assignmentAlgorithm(const Eigen::MatrixXf &distance_matrix);

  /**
   * @brief In case of no global objects, all measured objects get assigned to themselves.
   */
  void trivialAssignmentAlgo();

  void selfAssignOnAlgoError();
  void markAssignmentsInData();

 public:
  Matcher(std::shared_ptr<Data> data, std::string name);
  void runSingleSensor() override;
};
/**@}*/
