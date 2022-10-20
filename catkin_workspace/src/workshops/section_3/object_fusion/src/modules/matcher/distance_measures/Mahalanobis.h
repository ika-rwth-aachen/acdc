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

#pragma once

#include <Eigen/Dense>
#include <definitions/IkaObject.h>
#include <definitions/utility/ika_utilities.h>

class MahalanobisCalculator
{
public:
  MahalanobisCalculator();

  /**
   * @brief @see function uncorrelate_common_process_noise()
   */
  static bool bool_uncorrelate_common_process_noise;

  /**
   * @brief Interface to get basic mahalanobis distance including penalty terms
   */
  static float extendedDistance(const definitions::IkaObject* measured_object,
                        const GlobalObject* global_object);

private:

  static Eigen::MatrixXf constructMeasurementMatrixForAssoc(const Eigen::VectorXf& variances, const definitions::IkaObject *measured_object);

  /**
   * @brief basic mahalanobis distance between two definitions::IkaObject.
   */
  static float distance(const definitions::IkaObject& measured_object,
                        const GlobalObject& global_object,
                        const Eigen::MatrixXf& dim_red_mat);

  /**
   * @brief reduce the innovation covariance matrix due to common process noise in input tracks.
   * @details should only be done if the input object list is already filtered/tracked.
   * see Aeberhardt thesis p. 69-71.
   */
  static void uncorrelate_common_process_noise(
      Eigen::MatrixXf& S,
      const Eigen::MatrixXf& measured_cov,
      const Eigen::MatrixXf& global_cov);

  /**
   * @brief Calculates a mahalanobis penalty between two definitions::IkaObjects.
   * @details Penalizes classification clashes.
   */
  static float classificationPenalty(const definitions::IkaObject *global_object,
                                     const definitions::IkaObject* measured_object);
};
