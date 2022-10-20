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

#include "modules/matcher/distance_measures/Mahalanobis.h"
#include "data/Params.h"

#include <Eigen/Dense>

bool MahalanobisCalculator::bool_uncorrelate_common_process_noise = false;

MahalanobisCalculator::MahalanobisCalculator() {}

float MahalanobisCalculator::extendedDistance(
  const definitions::IkaObject *measured_object,
  const GlobalObject *global_object) {
  const Eigen::VectorXf &variances = IkaUtilities::getEigenVarianceVec(measured_object);

  // Construct the measurement matrix for this object. If an sensor does
  // not measure the velocity, there is no reason to try a velocity-based association.
  const Eigen::MatrixXf meas_matrix_assoc = constructMeasurementMatrixForAssoc(variances, measured_object);

  float mahDist_ = distance(*measured_object, *global_object, meas_matrix_assoc);
  float mahPenalty_ = classificationPenalty(global_object, measured_object);
  float mahTotalDist_ = mahDist_ + mahPenalty_;

  bool within_threshold = (mahTotalDist_ < Params::get().cfg.mahalanobis_threshold);
  return within_threshold ? mahTotalDist_ : -1.f; // otherwise -1 to prevent association (convention)
}

// The new getStateVecLength needs to define the object type -> use measured_object for that
Eigen::MatrixXf MahalanobisCalculator::constructMeasurementMatrixForAssoc(const Eigen::VectorXf &variances, const definitions::IkaObject *measured_object) {
  // Create an identiy matrix that only covers the position in x and y since this
  // is always used for association.
  // Ugly solution, but works
  Eigen::MatrixXf measurement_matrix_cand;
  if (IkaUtilities::getStateVecLengthConst(*measured_object) == 11) {
        // measurement_matrix_cand = Eigen::Matrix<float, 4, 11>::Zero();
        measurement_matrix_cand = Eigen::Matrix<float, 4, 10>::Zero();
  } else if (IkaUtilities::getStateVecLengthConst(*measured_object) == 10) {
    //  This might be incorrect, as not tested for the CTRV/CTRA model
    measurement_matrix_cand = Eigen::Matrix<float, 4, 10>::Zero();
  } else if (IkaUtilities::getStateVecLengthConst(*measured_object) == 0) {
    std::cout << "Sth might have gone wrong as StateVecLengthConst == 0";
    measurement_matrix_cand = Eigen::Matrix<float, 4, 0>::Zero();

  }

  // Iterate through other association candidates and check whether they should be used.
  int used_association_dimensions = 0;
  // Uncomment absVelX and absVelY to use velocity for association.
  for (int index : {
           (int)definitions::ca_model::posX, // The same position / idx for all models!
           (int)definitions::ca_model::posY
           //definitions::ika_object_states::absVelX
           //definitions::ika_object_states::absVelY
       }) {
    if (variances[index] >= 0.0f)
    {
      // positive variance --> value got measured by sensor --> can be used for association
      measurement_matrix_cand(used_association_dimensions, index) = 1.0f;
      ++used_association_dimensions;
    }
  }
  return measurement_matrix_cand.topRows(used_association_dimensions);
}

float MahalanobisCalculator::distance(
  const definitions::IkaObject &measured_object,
  const GlobalObject &global_object,
  const Eigen::MatrixXf &dim_red_mat)  // H in slides 
  {
  // Use the following Snippets for your solution
  // const Eigen::VectorXf &x_hat_S = IkaUtilities::getEigenStateVec(&measured_object);
  // const Eigen::VectorXf &x_hat_G = IkaUtilities::getEigenStateVec(&global_object);
  // const Eigen::VectorXf &P_S_diag = IkaUtilities::getEigenVarianceVec(&measured_object);
  // global_object.P(); // P_G
  // P_S_diag.asDiagonal(); // P_S
  
  /** START TASK 2.2 CODE HERE **/


  /** END TASK 2.2 CODE HERE **/
  return 1.f;
}

void MahalanobisCalculator::uncorrelate_common_process_noise(
    Eigen::MatrixXf &S,
    const Eigen::MatrixXf &measured_cov,
    const Eigen::MatrixXf &global_cov) {
  // compute approximation of cross-correlation for previously filtered input data
  // this is necessary because multiple tracking algorithms have the same process noise
  float tuning_param = 0.4f; // recommended by Aeberhardt thesis
  Eigen::MatrixXf cross_cov_approx = tuning_param * (measured_cov.array() * global_cov.array()).sqrt();
  S -= 2.0f * cross_cov_approx;
}

float MahalanobisCalculator::classificationPenalty(const definitions::IkaObject *measured_object,
                                                   const definitions::IkaObject *global_object) {
  assert(measured_object != nullptr);
  assert(global_object != nullptr);

  float penalty = Params::get().cfg.mahalanobis_penalty[std::make_pair(measured_object->IdType, global_object->IdType)];
  if (penalty < 0.f)
    return (Params::get().cfg.mahalanobis_threshold + 1.0f);
  else
    return (penalty * Params::get().cfg.mahalanobis_threshold);
}
