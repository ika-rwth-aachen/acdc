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

#include <cmath>
#include <tensorflow/cc/client/client_session.h>
#include <tensorflow/cc/ops/array_ops.h>
#include <tensorflow/cc/ops/image_ops.h>
#include <tensorflow/core/framework/tensor.h>
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>

#include "definitions.h"
#include "pillar_utils.h"

namespace lidar_detection
{

  /**
   * @brief Class defines the detector, which computes bounding boxes from the prediction tensors
   *
   */
  class Detector
  {

    /**
     * @brief Struct defines bundled input for nms
     *
     */
    struct Input
    {
      std::unique_ptr<tensorflow::ops::Placeholder> input_boxes;
      std::unique_ptr<tensorflow::ops::Placeholder> input_scores;
      std::unique_ptr<tensorflow::ops::Placeholder> max_boxes;
      std::unique_ptr<tensorflow::ops::Placeholder> iou_threshold;
      std::unique_ptr<tensorflow::ops::Placeholder> score_threshold;
    };

  public:
    /**
     * @brief Construct a new Detector object
     *
     */
    Detector();

    /**
     * @brief Setting config and params to the local detector copy
     *
     * @param config
     * @param params
     */
    void initialize(const SlikafConfig &config,
                    const Params &params);

    /**
     * @brief Create bounding boxes from prediction tensors. This function calls all other private class functions.
     *
     * @param outputs
     * @param indices
     * @param center_boxes
     */
    void createBoxes(const std::vector<tensorflow::Tensor> &outputs,
                     tensorflow::Tensor &indices,
                     std::vector<BoundingBoxCenter> &center_boxes);

  private:
    /**
     * @brief Computing bounding boxes based on the prediction results and filter according to parameters
     *
     * @param outputs
     * @param objects
     */
    void lidarOutputToBoxes(const std::vector<tensorflow::Tensor> &outputs,
                            std::vector<BoundingBoxCenter> &objects);

    /**
     * @brief Compute the vertix bounding box representation from center bounding boxes
     *
     * @param bounding_boxes_center
     * @param bounding_boxes_vertex
     */
    void centerToCornerBox2D(const std::vector<BoundingBoxCenter> &bounding_boxes_center,
                             std::vector<BoundingBoxMaxVertex> &bounding_boxes_vertex);

    /**
     * @brief Performing non-maximum suppression for the bounding boxes, resulting in an indices tensor
     *
     * @param bounding_boxes
     * @param indices
     */
    void nonMaxSuppression(const std::vector<BoundingBoxMaxVertex> bounding_boxes,
                           tensorflow::Tensor &indices);

    // scope for tensorflow operations
    std::unique_ptr<tensorflow::Scope> root_;
    std::unique_ptr<tensorflow::ClientSession> session_;
    std::unique_ptr<tensorflow::ops::NonMaxSuppressionV4> operation_;

    // input output tensors
    Input input_;
    std::vector<tensorflow::Tensor> outputs_;

    // struct for slikaf config
    Params params_;
    SlikafConfig config_;
  };

  /**
   * @brief Own implementation of std::clamp, since this is only available starting from C++17
   *
   */
  template <class T>
  constexpr const T &clamp(const T &v, const T &lo, const T &hi)
  {
    assert(!(hi < lo));
    return (v < lo) ? lo : (hi < v) ? hi
                                    : v;
  }

} // namespace lidar_detection
