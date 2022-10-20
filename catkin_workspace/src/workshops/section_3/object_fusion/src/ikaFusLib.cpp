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

#include "ikaFusLib.h"

#include "data/Params.h"
#include "modules/predictor/StatePredictor.h"
#include "modules/predictor/ExistencePredictor.h"
#include "modules/predictor/EgoMotionCompensation.h"
#include "modules/matcher/Matcher.h"
#include "modules/fuser/StateFuser.h"
#include "modules/fuser/HeadingAngleFuser.h"
#include "modules/fuser/ExistenceFuser.h"
#include "modules/fuser/ClassificationFuser.h"
#include "modules/manager/ObjectRemover.h"
#include "modules/manager/ObjectCreator.h"
#include "modules/manager/MeasurementHistorian.h"


ikaFusLib::ikaFusLib() {

  data_ = std::make_shared<Data>();
  // predictor
  modules_.push_back(std::unique_ptr<ExistencePredictor>(new ExistencePredictor(data_, "ExistencePredictor"))); 
  modules_.push_back(std::unique_ptr<ObjectRemover>(new ObjectRemover(data_, "Manager"))); // actually manager
  modules_.push_back(std::unique_ptr<StatePredictor>(new StatePredictor(data_, "StatePredictor")));
  modules_.push_back(std::unique_ptr<EgoMotionCompensation>(new EgoMotionCompensation(data_, "EgoMotionCompensation")));

  // matcher
  modules_.push_back(std::unique_ptr<Matcher>(new Matcher(data_, "Matcher")));

  // fuser
  modules_.push_back(std::unique_ptr<StateFuser>(new StateFuser(data_, "StateFuser")));
  modules_.push_back(std::unique_ptr<HeadingAngleFuser>(new HeadingAngleFuser(data_, "HeadingAngleFuser")));
  modules_.push_back(std::unique_ptr<ExistenceFuser>(new ExistenceFuser(data_, "ExistenceFuser")));
  modules_.push_back(std::unique_ptr<ClassificationFuser>(new ClassificationFuser(data_, "ClassificationFuser")));

  // manager
  modules_.push_back(std::unique_ptr<ObjectCreator>(new ObjectCreator(data_, "GlobalObjectCreator")));
  modules_.push_back(std::unique_ptr<MeasurementHistorian>(new MeasurementHistorian(data_, "MeasurementHistorian")));
}

bool ikaFusLib::initiateKalmanFilterConstants(Eigen::MatrixXf &constant_system_matrix,
                                                  Eigen::MatrixXf &time_variant_system_matrix,
                                                  Eigen::MatrixXf &time_variant_process_noise_matrix) {
  data_->F_const_ = constant_system_matrix;
  data_->F_timevar_ = time_variant_system_matrix;
  data_->Q_timevar_ = time_variant_process_noise_matrix;
  return true;
}

void ikaFusLib::initiateFusionConstants(ConfigParams cfg_params) {
  Params::get().setParamsFromConfig(cfg_params);
}

definitions::IkaObjectList ikaFusLib::fuseIntoGlobal(
    const definitions::IkaObjectList& object_list_measured,
    const definitions::IkaEgoMotion& ego_motion) {

  data_->object_list_measured = object_list_measured;
  data_->ego_motion = ego_motion;

  setPredictionGap();
  handleTimeJumps();

  // Run each module stacked in the  modules_ vector -> each an object, takes data_
  for (auto &module : modules_) {
    module->runSingleSensor();
  }

  // pack into ROS message and return 
  return getFusedOutputList();
}

void ikaFusLib::setPredictionGap() {

  auto stamp_measurements_ms = fusutil::ROSstamp2unsignedlong_ms(data_->object_list_measured.header.stamp);
  auto stamp_previous_global_ms = fusutil::ROSstamp2unsignedlong_ms(data_->object_list_fused.header.stamp);

  long delta_time_in_ms = signed(stamp_measurements_ms) - signed(stamp_previous_global_ms);
  data_->prediction_gap_in_seconds = delta_time_in_ms/1e3f;

  // debugging out-of-sequence measurements!
//  std::cerr << "Fusion: t_meas=" << stamp_measurements_ms << "[ms]"
//            << ", t_prev_glob=" << stamp_previous_global_ms << "[ms]"
//            << ", dt=" << delta_time_in_ms
//            << ", measurement=" << IkaUtilities::sourceToName(object_list_measured.IdSource)
//            << negText
//            << std::endl;
}

void ikaFusLib::handleTimeJumps() {
  std::string negText = "";
  if (data_->prediction_gap_in_seconds < Params::get().cfg.time_jump_backward_sec) {
    std::cerr << "Time jump backward of " << data_->prediction_gap_in_seconds
              << "s < " << Params::get().cfg.time_jump_backward_sec << "s";
    resetGlobalObjectList();
  }
  else if (data_->prediction_gap_in_seconds > Params::get().cfg.time_jump_forward_sec) {
    std::cerr << "Time jump forward of " << data_->prediction_gap_in_seconds
              << "s > " << Params::get().cfg.time_jump_forward_sec << "s";
    resetGlobalObjectList();
  }
  else if (data_->prediction_gap_in_seconds < 0.f) {
    negText = "(negative dt!)";
    // keep the negative prediction gap in memory to eventually consider
    // out-of-sequence measurements in the modules
  }
}

void ikaFusLib::resetGlobalObjectList() {
  std::cerr << " -> Clearing global object list." << std::endl;
  IkaUtilities::clearObjectList(&data_->object_list_fused);
  data_->object_list_fused.header.stamp = data_->object_list_measured.header.stamp;
  data_->prediction_gap_in_seconds = 0.f;
}

definitions::IkaObjectList ikaFusLib::getFusedOutputList() {
  // Create a ROS-comaptible message.
  definitions::IkaObjectList object_list;
  object_list.header = data_->object_list_fused.header;
  object_list.IdSource = data_->object_list_fused.IdSource;
  for (const auto& object: data_->object_list_fused.objects) {
    if (outputCriterion(object)) {
      object_list.objects.push_back(object);
    }
  }
  return object_list;
}

bool ikaFusLib::outputCriterion(const GlobalObject& global_object) {
  if (!global_object.associated) return false;
  if (global_object.fExistenceProbability
      < Params::get().cfg.existence_probability_output_threshold) return false;

  return true;
}

