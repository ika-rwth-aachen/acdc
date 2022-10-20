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

#include <list_creator.h>

namespace lidar_detection
{
  void ikaListCreator::createObjectList(const std_msgs::Header header,
                                        const tensorflow::Tensor &indices,
                                        const std::vector<BoundingBoxCenter> &objects,
                                        definitions::IkaObjectList &object_list)
  {
    object_list.header = header;

    if (indices.dims() == 0)
      return;

    // try to receive the transformation from inference frame to output frame
    geometry_msgs::TransformStamped transformation;
    bool trans = false;

    if (!params_.output_frame.empty() &&
        params_.output_frame != params_.inference_frame)
    {
      try
      {
        transformation = tf_buffer_->lookupTransform(params_.output_frame, params_.inference_frame, ros::Time(0));
        trans = true;
      }
      catch (tf2::TransformException &e)
      {
        ROS_ERROR("Lidar Creator Error: transformation from inference_frame to output_frame not found %s", e.what());
        return;
      }

      object_list.header.frame_id = params_.output_frame;
    }

    definitions::input_sensors input_sensor = definitions::input_sensors::VLP;

    // iterate over all bounding boxes
    for (int idx = 0; idx < indices.dim_size(0); ++idx)
    {
      // get box index
      int box_index = indices.tensor<int, 1>()(idx);
      const BoundingBoxCenter &bounding_box = objects[box_index];

      // create an object
      definitions::IkaObject object;

      object.bObjectValid = true;
      object.bObjectMeasured = true;
      object.bObjectNew = true;

      object.header.frame_id = header.frame_id;
      object.header.stamp = header.stamp;

      object.IdExternal = input_sensor;

      object.fExistenceProbability = bounding_box.score;

      // ! HACK ! to draw probability in rviz.
      object.IdInternal = static_cast<int>(object.fExistenceProbability * 100);

      // set measurement values
      if (!object.measHist.size())
      {
        object.measHist.push_back(definitions::IkaSensorStamp());
      }
      object.measHist[0].IdSensor = definitions::input_sensors::VLP;
      object.measHist[0].measuredStamp = object_list.header.stamp;

      // set object motion model
      object.IdMotionModel = definitions::motion_model::CV;
      object.fMean.resize((int)definitions::ca_model::COUNT);
      object.fCovariance.resize(pow((int)definitions::ca_model::COUNT, 2));

      // set object position
      object.fMean[(int)definitions::ca_model::posX] = bounding_box.center(0);
      object.fMean[(int)definitions::ca_model::posY] = bounding_box.center(1);
      object.fMean[(int)definitions::ca_model::posZ] = bounding_box.z;

      // START TASK 1 CODE   
      
      // set object dimensions
      object.fMean[(int)definitions::ca_model::length] = 2;
      object.fMean[(int)definitions::ca_model::width] = 2;
      object.fMean[(int)definitions::ca_model::height] = 2;

      // set yaw angle
      object.fMean[(int)definitions::ca_model::heading] = 0;

      // END TASK 1 CODE   

      // get current class_name
      std::string cls_name = bounding_box.class_name;
      boost::algorithm::to_lower(cls_name);

      // assign ika type
      if (cls_name.compare("car") == 0)
        object.IdType = definitions::ika_object_types::CAR;
      else if (cls_name.compare("pedestrian") == 0)
        object.IdType = definitions::ika_object_types::PEDESTRIAN;
      else if (cls_name.compare("bicycle") == 0)
        object.IdType = definitions::ika_object_types::BICYCLE;
      else if (cls_name.compare("truck") == 0)
        object.IdType = definitions::ika_object_types::TRUCK;
      else
        object.IdType = definitions::ika_object_types::UNCLASSIFIED;

      // specify variance
      object.fCovariance = IkaUtilities::CovarianceFromVariance(params_.variance);

      // transform object to base link
      if (trans)
      {
        IkaUtilities::transformIkaObject(&object, &transformation);
      }

      object_list.objects.push_back(object);
    }
  }
} // namespace lidar_detection
