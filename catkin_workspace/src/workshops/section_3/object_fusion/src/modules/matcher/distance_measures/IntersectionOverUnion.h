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

#include <definitions/IkaObject.h>
#include <definitions/utility/ika_utilities.h>

class IouCalculator
{
public:
  IouCalculator();

  class Bbox {
  public:
    float top_left_x;
    float top_left_y;
    float bottom_right_x;
    float bottom_right_y;
    float length;
    float width; 
};

  /**
   * @brief Interface to get basic iou "distance" for objects association 
   */
  static float extendedDistance(const definitions::IkaObject* measured_object,
                        const GlobalObject* global_object);

private:
  /**
   * @brief Compute the iou from bounding boxes
   */
  static float computeIoU(IouCalculator::Bbox &a, IouCalculator::Bbox &b);

  /**
   * @brief Transform given information to a bbox object for computation (top left and bottom right corners of the bbox)
   */
  static IouCalculator::Bbox getBbox(const float *pos_x, const float *pos_y, const float *length, const float *width, const float *rot_angle);

};
