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

#include <detector.h>

namespace lidar_detection
{

  Detector::Detector()
  {
    tensorflow::Scope scope = tensorflow::Scope::NewRootScope();
    root_.reset(&scope);

    // build the tensorflow graph:
    //    a tensorflow non max suppression expects the following parameters that
    //    have to be gathered before creating the operation.

    // bounding Boxes as (N, 4) tensor
    input_.input_boxes = std::make_unique<tensorflow::ops::Placeholder>(
        root_->WithOpName("input/boxes"), tensorflow::DT_FLOAT);
    // box scores as (N) tensor
    input_.input_scores = std::make_unique<tensorflow::ops::Placeholder>(
        root_->WithOpName("input/scores"), tensorflow::DT_FLOAT);
    // number of boxes as 0D int tensor
    input_.max_boxes = std::make_unique<tensorflow::ops::Placeholder>(
        root_->WithOpName("input/num_boxes"), tensorflow::DT_INT32);
    // iou threshold for considering as overlap as 0D float tensor
    input_.iou_threshold = std::make_unique<tensorflow::ops::Placeholder>(
        root_->WithOpName("input/iou_threshold"), tensorflow::DT_FLOAT);
    // score threshold as 0D float tensor
    input_.score_threshold = std::make_unique<tensorflow::ops::Placeholder>(
        root_->WithOpName("input/score_threshold"), tensorflow::DT_FLOAT);

    // build final nms graph
    operation_ = std::make_unique<tensorflow::ops::NonMaxSuppressionV4>(
        *root_, *input_.input_boxes, *input_.input_scores, *input_.max_boxes,
        *input_.iou_threshold, *input_.score_threshold);

    // create tf session
    tensorflow::SessionOptions options = tensorflow::SessionOptions();
    options.config.mutable_gpu_options()->set_allow_growth(true);
    options.config.mutable_gpu_options()->set_per_process_gpu_memory_fraction(0.5);
    session_ = std::make_unique<tensorflow::ClientSession>(scope, options);
  }

  void Detector::initialize(const SlikafConfig &config,
                            const Params &params)
  {
    config_ = config;
    params_ = params;
  }

  void Detector::createBoxes(const std::vector<tensorflow::Tensor> &outputs, tensorflow::Tensor &indices, std::vector<BoundingBoxCenter> &center_boxes)
  {
    // create temporary max_boxes vector
    std::vector<BoundingBoxMaxVertex> max_boxes;

    // create center bounding boxes
    lidarOutputToBoxes(outputs, center_boxes);

    // create max and min bounding boxes
    centerToCornerBox2D(center_boxes, max_boxes);

    // perform a non max suppression and get the results.
    nonMaxSuppression(max_boxes, indices);
  }

  void Detector::lidarOutputToBoxes(const std::vector<tensorflow::Tensor> &outputs,
                                    std::vector<BoundingBoxCenter> &objects)
  {
#ifdef MODE_PP
    // safe tensors as Eigen TensorMap.
    // huge runtime improvement due to optimization
    // TODO: why option = 16?
    Eigen::TensorMap<Eigen::Tensor<const float, 4, 1, long int>, 16,
                     Eigen::MakePointer>
        occupancy = outputs[0].tensor<float, 4>();
    Eigen::TensorMap<Eigen::Tensor<const float, 5, 1, long int>, 16,
                     Eigen::MakePointer>
        location = outputs[1].tensor<float, 5>();
    Eigen::TensorMap<Eigen::Tensor<const float, 5, 1, long int>, 16,
                     Eigen::MakePointer>
        size = outputs[2].tensor<float, 5>();
    Eigen::TensorMap<Eigen::Tensor<const float, 4, 1, long int>, 16,
                     Eigen::MakePointer>
        angle = outputs[3].tensor<float, 4>();
    Eigen::TensorMap<Eigen::Tensor<const float, 4, 1, long int>, 16,
                     Eigen::MakePointer>
        heading = outputs[4].tensor<float, 4>();
    Eigen::TensorMap<Eigen::Tensor<const float, 5, 1, long int>, 16,
                     Eigen::MakePointer>
        classification = outputs[5].tensor<float, 5>();

    // iterate over grid
    for (int x_index = 0; x_index < occupancy.dimension(1); ++x_index)
    {
      for (int y_index = 0; y_index < occupancy.dimension(2); ++y_index)
      {
        // calculate the mean probability over all anchors
        float mean_probability = 0.0;
        for (int j = 0; j < occupancy.dimension(3); ++j)
        {
          mean_probability += occupancy(0, x_index, y_index, j);
        }
        mean_probability /= occupancy.dimension(3);

        // iterate over all anchors
        for (int anchor = 0; anchor < occupancy.dimension(3); ++anchor)
        {

          // check for minimum_tresh, just for runtime reasons
          if (occupancy(0, x_index, y_index, anchor) < config_.minimum_thresh)
          {
            continue;
          }

          // initilize bounding_box
          BoundingBoxCenter bounding_box;

          // position calculation
          // The origin is basically placed in the middle of a cell although this is
          // technically not possibly. The grid requires an even number of cells
          // an the origin is therefore placed at the cross of the four center
          // cells. However, the network was trained to learn the offset from the
          // bottom right corner of the cell.
          float x_center = x_index * config_.delta_x * config_.downscaling + config_.x_min;
          float y_center = y_index * config_.delta_y * config_.downscaling + config_.y_min;

          // set position of bounding_box
          bounding_box.center(0) =
              x_center +
              location(0, x_index, y_index, anchor, 0) *
                  config_.anchor_diagonals[anchor];
          bounding_box.center(1) =
              y_center +
              location(0, x_index, y_index, anchor, 1) *
                  config_.anchor_diagonals[anchor];
          bounding_box.z =
              config_.anchor_boxes[anchor].z_center +
              location(0, x_index, y_index, anchor, 2) *
                  config_.anchor_boxes[anchor].height;

          // set size of bounding_box
          bounding_box.length =
              std::exp(size(0, x_index, y_index, anchor, 0)) *
              config_.anchor_boxes[anchor].length;
          bounding_box.width =
              std::exp(size(0, x_index, y_index, anchor, 1)) *
              config_.anchor_boxes[anchor].width;
          bounding_box.height =
              std::exp(size(0, x_index, y_index, anchor, 2)) *
              config_.anchor_boxes[anchor].height;

          // set yaw of bounding_box
          bounding_box.yaw =
              std::asin(clamp(angle(0, x_index, y_index, anchor), -1.0f, 1.0f)) +
              config_.anchor_boxes[anchor].orientation;

          // filter boxes containing nan values
          if (std::isnan(bounding_box.length) || std::isnan(bounding_box.width) || std::isnan(bounding_box.height) || std::isnan(bounding_box.yaw))
          {
            continue;
          }

          // build class_score vector
          std::vector<float> class_scores;
          for (int i = 0; i < classification.dimension(4); i++)
          {
            class_scores.push_back(classification(0, x_index, y_index, anchor, i));
          }

          // check if class_names has same dimension
          if (class_scores.size() != config_.class_names.size())
          {
            ROS_ERROR("Detector Error: 'class_names' is required to have the same size than the cls layer of the neural network");
          }

          // set class_idx and score
          // START TASK 2 CODE   

          int class_idx = -1;

          // END TASK 2 CODE   

          float score = occupancy(0, x_index, y_index, anchor);

          // filter by specific detection thresholds.
          if (class_idx != -1 && score < config_.score_thresh[class_idx])
          {
            continue;
          }

          // filter objects by heuristics
          if (bounding_box.width > params_.max_width ||
              bounding_box.length > params_.max_length ||
              bounding_box.height > params_.max_height)
            continue;

          // filter all anchors with lower probability than mean
          if (occupancy(0, x_index, y_index, anchor) < mean_probability)
          {
            continue;
          }

          // set class_idx and score
          bounding_box.class_idx = class_idx;
          bounding_box.class_name = config_.class_names[class_idx];
          bounding_box.score = score;

          objects.push_back(bounding_box);
        }
      }
    }
#endif
  }

  void Detector::centerToCornerBox2D(
      const std::vector<BoundingBoxCenter> &bounding_boxes_center,
      std::vector<BoundingBoxMaxVertex> &bounding_boxes_vertex)
  {
    // iterate over boxes
    for (const auto &center : bounding_boxes_center)
    {

      BoundingBoxMaxVertex vertex;

      // rotate bounding box with rotation matrix
      Eigen::Matrix2d rotation;
      float sin_yaw = std::sin(center.yaw);
      float cos_yaw = std::cos(center.yaw);
      rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;

      // offsets to vertex
      Eigen::Vector2d offset_front_left =
          rotation * Eigen::Vector2d(center.length * 0.5, center.width * 0.5);
      Eigen::Vector2d offset_front_right =
          rotation * Eigen::Vector2d(center.length * 0.5, -center.width * 0.5);

      // get all vertices from offsets and center
      Eigen::Vector2d front_right = center.center + offset_front_right;
      Eigen::Vector2d front_left = center.center + offset_front_left;
      Eigen::Vector2d rear_left = center.center - offset_front_right;
      Eigen::Vector2d rear_right = center.center - offset_front_left;

      // set the max and min vertices.
      vertex.min(0) = std::min({front_right(0), front_left(0), rear_left(0), rear_right(0)});
      vertex.min(1) = std::min({front_right(1), front_left(1), rear_left(1), rear_right(1)});
      vertex.max(0) = std::max({front_right(0), front_left(0), rear_left(0), rear_right(0)});
      vertex.max(1) = std::max({front_right(1), front_left(1), rear_left(1), rear_right(1)});
      vertex.score = center.score;

      // add bounding box to vector.
      bounding_boxes_vertex.push_back(vertex);
    }
  }

  // nms function is included in detetor object class because of the tensorflow session
  void Detector::nonMaxSuppression(const std::vector<BoundingBoxMaxVertex> bounding_boxes,
                                   tensorflow::Tensor &indices)
  {
    int num_boxes = bounding_boxes.size();

    // create inputs for nms
    auto input_boxes = tensorflow::Tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({num_boxes, 4}));
    auto input_scores = tensorflow::Tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({num_boxes}));
    auto max_boxes = tensorflow::Tensor(tensorflow::DT_INT32, tensorflow::TensorShape({}));
    auto iou_threshold = tensorflow::Tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({}));
    auto score_threshold = tensorflow::Tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape({}));

    // use TensorMaps to fill in the content to improve speed
    Eigen::TensorMap<Eigen::Tensor<float, 2, 1, long int>, 16, Eigen::MakePointer>
        input_boxes_map = input_boxes.tensor<float, 2>();
    Eigen::TensorMap<Eigen::Tensor<float, 1, 1, long int>, 16, Eigen::MakePointer>
        input_scores_map = input_scores.tensor<float, 1>();

    // fill boxes and scores into tensor
    for (int i = 0; i < num_boxes; ++i)
    {
      // x_min, y_min, x_max, y_max
      input_boxes_map(i, 0) = bounding_boxes.at(i).min(0);
      input_boxes_map(i, 1) = bounding_boxes.at(i).min(1);
      input_boxes_map(i, 2) = bounding_boxes.at(i).max(0);
      input_boxes_map(i, 3) = bounding_boxes.at(i).max(1);

      assert(bounding_boxes.at(i).score && "All boxes need to contain a score!");
      input_scores_map(i) = bounding_boxes.at(i).score;
    }

// fill parameters into tensor
#ifdef MODE_PP
    max_boxes.tensor<int, 0>()() = config_.nms_max_objects;
    iou_threshold.tensor<float, 0>()() = config_.nms_iou_thresh;
    score_threshold.tensor<float, 0>()() = 0.0;
#endif

    // run operation
    tensorflow::Status status;
    status = session_->Run(
        {{*input_.input_boxes, input_boxes},
         {*input_.input_scores, input_scores},
         {*input_.max_boxes, max_boxes},
         {*input_.iou_threshold, iou_threshold},
         {*input_.score_threshold, score_threshold}},
        {operation_->selected_indices, operation_->valid_outputs}, &outputs_);

    // check session
    if (!status.ok())
      ROS_ERROR("Detector Error: session_->Run(): %s", status.ToString().c_str());

    indices = outputs_[0];
  }

} // namespace lidar_detection
