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

#include "AngleProperty.h"

#include <QPainter>
#include <QPainterPath>
#include <QStyleOptionViewItem>
#include <QWidget>

#include <cmath>

rviz::AngleProperty::AngleProperty(const QString &name,
                                   float default_value,
                                   const QString &description,
                                   rviz::Property *parent,
                                   const char *changed_slot,
                                   QObject *receiver) : FloatProperty(name,
                                                                      default_value,
                                                                      description,
                                                                      parent,
                                                                      changed_slot,
                                                                      receiver) {

}

bool rviz::AngleProperty::paint(QPainter *painter, const QStyleOptionViewItem &option) const {

  painter->save();
  QColor color = Qt::blue;
  QRect rect = option.rect;
  auto angle_in_rad = static_cast<float>(value_.toFloat());
  float line_length = rect.height();
  auto angle_in_pi_parts = static_cast<float>(angle_in_rad / M_PI);
  QString text = QString::number(angle_in_pi_parts,'f',2) + " Pi"; // https://doc.qt.io/qt-5/qstring.html#argument-formats
  rect.adjust(rect.height() + 4, 1, 0, 0);
  painter->drawText(rect, text);
  rect.adjust(-rect.height() - 4, -1, 0, 0);

  QPointF starting_point;
  starting_point.setX(
      rect.x() + rect.height() / 2.0f - line_length / 4.0f * std::cos(-angle_in_rad - M_PI / 2.0f));
  starting_point.setY(
      rect.y() + rect.height() / 2.0f - line_length / 4.0f * std::sin(-angle_in_rad - M_PI / 2.0f));

  QPointF starting_point_r;
  starting_point_r.setX(
      rect.x() + rect.height() / 2.0f - line_length / 2.0f * std::cos(-angle_in_rad - M_PI / 2.0f + 0.5f));
  starting_point_r.setY(
      rect.y() + rect.height() / 2.0f - line_length / 2.0f * std::sin(-angle_in_rad - M_PI / 2.0f + 0.5f));

  QPointF starting_point_l;
  starting_point_l.setX(
      rect.x() + rect.height() / 2.0f - line_length / 2.0f * std::cos(-angle_in_rad - M_PI / 2.0f - 0.5f));
  starting_point_l.setY(
      rect.y() + rect.height() / 2.0f - line_length / 2.0f * std::sin(-angle_in_rad - M_PI / 2.0f - 0.5f));

  QPointF end_point;
  end_point.setX(rect.x() + rect.height() / 2.0f + line_length / 2.0f * std::cos(-angle_in_rad - M_PI / 2));
  end_point.setY(rect.y() + rect.height() / 2.0f + line_length / 2.0f * std::sin(-angle_in_rad - M_PI / 2));

  QPainterPath path;
  path.moveTo(end_point);
  path.lineTo(starting_point_r);
  path.lineTo(starting_point);
  path.lineTo(starting_point_l);
  path.lineTo(end_point);

  painter->fillPath(path, QBrush(color));
  painter->setRenderHint(QPainter::Antialiasing);
  painter->setRenderHint(QPainter::HighQualityAntialiasing);
  painter->restore();
  return true;
}
