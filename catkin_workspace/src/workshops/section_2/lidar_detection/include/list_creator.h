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

#include <definitions/IkaObjectList.h>
#include <definitions/utility/object_definitions.h>
#include <definitions/utility/ika_utilities.h>

#include <tensorflow/cc/client/client_session.h>
#include <tensorflow/cc/ops/array_ops.h>
#include <tensorflow/cc/ops/image_ops.h>
#include <tensorflow/core/framework/tensor.h>
#include <tensorflow/core/platform/env.h>
#include <tensorflow/core/public/session.h>

#include <boost/algorithm/string.hpp>

#include <definitions.h>

namespace lidar_detection
{

  /**
   * @brief Abstract base class for all message dependent ListCreator's
   *
   * @tparam object_list_t
   */
  template <typename object_list_t>
  class ListCreator
  {
  public:
    /**
     * @brief Set params and config to the list creator
     *
     * @param params
     * @param tf_buffer
     */
    void initialize(const Params &params,
                    const tf2_ros::Buffer &tf_buffer)
    {
      params_ = params;
      tf_buffer_.reset(&tf_buffer);
    }

    /**
     * @brief Virtual base calls to create an ObjectList object. This function will be overwritten.
     *
     * @param header
     * @param indices
     * @param objects
     * @param object_list
     */
    virtual void createObjectList(const std_msgs::Header header,
                                  const tensorflow::Tensor &indices,
                                  const std::vector<BoundingBoxCenter> &objects,
                                  object_list_t &object_list) = 0;

  protected:
    Params params_;

    boost::shared_ptr<const tf2_ros::Buffer> tf_buffer_;
  };

  /**
   * @brief ListCreator in ika message format
   *
   */
  class ikaListCreator : public ListCreator<definitions::IkaObjectList>
  {
  public:
    /**
     * @brief Create an ObjectList object from remaining bounding boxes and nms indices
     *
     * @param indices
     * @param objects
     * @param object_list
     */
    void createObjectList(const std_msgs::Header,
                          const tensorflow::Tensor &indices,
                          const std::vector<BoundingBoxCenter> &objects,
                          definitions::IkaObjectList &object_list);
  };

} // namespace lidar_detection