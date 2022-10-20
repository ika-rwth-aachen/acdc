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

#include <rviz/properties/float_property.h>

/*
 * Move to rviz namespace to match property naming convention.
 */
namespace rviz {

/**
 * @author Simon Schaefer
 * @date 01.01.2019
 * @brief Simple Float property with an advanced display method.
 * @details This property will draw an arrow next to the value. The value is represented in fractions of Pi.
 */
class AngleProperty : public rviz::FloatProperty {
 public :
  /**
   * @brief Just execute super constructor. Nothing done here.
   * @see FloatProperty::FloatProperty
   */
  explicit AngleProperty(const QString &name = QString(),
                float default_value = 0,
                const QString &description = QString(),
                Property *parent = nullptr,
                const char *changed_slot = nullptr,
                QObject *receiver = nullptr);

  /**
   * @brief Will draw a new representation of an angle.
   * @details Will draw an arrow with zero at top and counterclockwise positive.
   * @return always true, this is the way the system know to repaint.
   */
  bool paint(QPainter *painter, const QStyleOptionViewItem &option) const override;
};

}  // namespace rviz
