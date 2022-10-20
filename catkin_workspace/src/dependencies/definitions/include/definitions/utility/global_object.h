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

#include <vector>

#include <Eigen/Dense>

#include <definitions/IkaObjectList.h>

// This data type inherits from the ika object definition and therefore contains
// all attributes from the ROS message. As this
class GlobalObject : public definitions::IkaObject {
 public:
  GlobalObject() {}

  GlobalObject(definitions::IkaObject object,
               Eigen::Map<Eigen::VectorXf, 0, Eigen::InnerStride<> > variances)
      : definitions::IkaObject(object) {
    // First, initialize the P matrix with the variances given by the
    // object itself.
    P_ = variances.asDiagonal();
  }

  // Mutable getter for P.
  Eigen::MatrixXf& P() { return P_; }

  // Const getter for P
  const Eigen::MatrixXf& P() const { return P_; }

  // Has this object been measured at least twice or by two different sensors.
  bool associated = false;

 private:
  // Covariance Matrix for this object.
  Eigen::MatrixXf P_;
};

// This data type inherits from the ika object definition and therefore contains
// all attributes from the ROS message. As this
class GlobalObjectList : public definitions::IkaObjectList {
 public:
  // make this public as the object vector in the ros message is public also.
  std::vector<GlobalObject> objects;
};
