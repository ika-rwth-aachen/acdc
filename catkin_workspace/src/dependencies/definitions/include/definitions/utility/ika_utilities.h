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

/**
 * @defgroup IKA_DEFINITION Ika global Definitions
 * @brief Module for global definitions needed by the sensor fusion.
 * @details This module contains all framework-independent definition
 * @addtogroup IKA_DEFINITION
 */
/**@{*/

#pragma once

#include <cmath>
#include <chrono>
#include <ctime>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/AccelWithCovariance.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>
#include <tf/transform_broadcaster.h> // for createQuaternionMsgFromYaw()

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <definitions/IkaObject.h>
#include <definitions/IkaObjectList.h>
#include <definitions/default_values.h>

#include <definitions/utility/global_object.h>
#include <definitions/utility/object_definitions.h>
#include <Eigen/Dense>

#include <tf/transform_listener.h>

#define M_PI_TIMES_2 6.283185307179586476925      /* 2 * pi */
#define SQRT_M_PI_TIMES_2 2.506628274631000502416 /* sqrt(2 * pi) */
/**
 * @author Simon Schaefer
 * @date 10.12.2018
 * @brief Utility functions that are globally available to keep consistency in the overall code.
 * @note Not the hole code was refactored to use this code.
 * @todo Refactor the whole code to work with this instead of local definitions.
 */
class IkaUtilities
{
public:
  /**
   * @brief Will clip a double between lower and upper limit.
   * @param double to clip
   * @param lower limit
   * @param upper limit
   * @return clipped double
   */
  static double clip(const double &n, const double &lower, const double &upper)
  {
    return std::max(lower, std::min(n, upper));
  }

  /**
   * @brief Will transform a angle from degree to radiants.
   * @param degree Angle in degree.
   * @return Angle in radiants.
   */
  static float toRadiant(float degree)
  {
    return static_cast<float>(degree / 180.0f * M_PI);
  }

  /**
   * @brief Will transform a angle from radiants to degree.
   * @param radiants Angle in radiants.
   * @return Angle in degree.
   */
  static float toDegree(float radiants)
  {
    return static_cast<float>(radiants * 180.0f / M_PI);
  }

  /**
   * @brief will limit the input to 0 <= input <= 2*M_PI, adding or subtracting 2*pi if necessary
   * @param input, most likely radiant (double)
   * @return nothing
   */
  static void limitTo2PI(double &input)
  {
    while (input > M_PI_TIMES_2)
    {
      input -= M_PI_TIMES_2;
    }
    while (input < 0.)
    {
      input += M_PI_TIMES_2;
    }
  }

  /**
   * @brief will limit the input to 0 <= input <= 2*M_PI, adding or subtracting 2*pi if necessary
   * @param input, most likely radiant (float)
   * @return nothing
   */
  static void limitTo2PI(float &input)
  {
    while (input > M_PI_TIMES_2)
    {
      input -= M_PI_TIMES_2;
    }
    while (input < 0.)
    {
      input += M_PI_TIMES_2;
    }
  }

  /**
   * @brief will limit the input to 0 <= input <= 360, adding or subtracting 360 if necessary
   * @param input, most likely degree (double)
   * @return nothing
   */
  static void limitTo360Degree(double &input)
  {
    while (input > 360.)
    {
      input -= 360.;
    }
    while (input < 0.)
    {
      input += 360.;
    }
  }
  /**
   * @brief will limit the input to 0 <= input <= 360, adding or subtracting 360 if necessary
   * @param input, most likely degree (float)
   * @return nothing
   */
  static void limitTo360Degree(float &input)
  {
    while (input > 360.)
    {
      input -= 360.;
    }
    while (input < 0.)
    {
      input += 360.;
    }
  }

  /**
   * @brief Will map a type of object to is corresponding name.
   * @details Will return "Unknown" if no decision could be made.
   * @warning Changes have to be made if a new type were added.
   * @param type Type of object.
   * @return Name of object.
   */
  static std::string typeToName(uint8_t type)
  {
    switch (type)
    {
    case definitions::ika_object_types::CAR:
      return "Car";
    case definitions::ika_object_types::TRUCK:
      return "Truck";
    case definitions::ika_object_types::MOTORBIKE:
      return "Motorbike";
    case definitions::ika_object_types::PEDESTRIAN:
      return "Ped.";
    case definitions::ika_object_types::BICYCLE:
      return "Bicycle";

    default:
      return "Unknown";
    }
  }

  /**
   * @brief Will map a name of object to is corresponding type.
   * @details Will return "Unknown" if no decision could be made.
   * @warning Changes have to be made if a new type were added.
   * @param type name of object.
   * @return type of object.
   * @author Moritz Cremer
   */
  static uint8_t nametoType(std::string name)
  {
    if (name == "Car")
      return definitions::ika_object_types::CAR;
    else if (name == "Truck")
      return definitions::ika_object_types::TRUCK;
    else if (name == "Motorbike")
      return definitions::ika_object_types::MOTORBIKE;
    else if (name == "Ped.")
      return definitions::ika_object_types::PEDESTRIAN;
    else if (name == "Bicycle")
      return definitions::ika_object_types::BICYCLE;
    else if (name == "Unknown")
      return definitions::ika_object_types::UNCLASSIFIED;
    else
      return definitions::ika_object_types::UNCLASSIFIED;
  }

  /**
   * @brief Will map from a source or output to a unique name.
   * @warning Changes have to be made if a new sensor were added.
   * @param source_id Source id of the object list.
   * @return Unique name of an object list.
   */
  static std::string sourceToName(uint8_t source_id)
  {
    switch (source_id)
    {
    case definitions::input_sensors::SOURCE_NO_SOURCE:
      return "NO_SOURCE";
    case definitions::input_sensors::ESR_FRONT:
      return "ESR_FRONT";
    case definitions::input_sensors::ESR_REAR:
      return "ESR_REAR";
    case definitions::input_sensors::RADAR:
      return "RADAR";
    case definitions::input_sensors::EPM:
      return "EPM";
    case definitions::input_sensors::LUX:
      return "LUX";
    case definitions::input_sensors::SRR_FRONT:
      return "SRR_FRONT";
    case definitions::input_sensors::RT:
      return "RT";
    case definitions::input_sensors::SRR_REAR:
      return "SRR_REAR";
    case definitions::input_sensors::SRR_LEFT:
      return "SRR_LEFT";
    case definitions::input_sensors::SRR_RIGHT:
      return "SRR_RIGHT";
    case definitions::input_sensors::SCALA:
      return "SCALA";
    case definitions::input_sensors::VLP:
      return "VLP";
    case definitions::input_sensors::RTRANGE:
      return "RT_RANGE";
    case definitions::input_sensors::LIDAR_RS:
      return "LIDAR_RS";
    case definitions::input_sensors::CAMERA:
      return "CAMERA";
    case definitions::input_sensors::STATION_A:
      return "STATION_A";
    case definitions::input_sensors::STATION_B:
      return "STATION_B";
    case definitions::input_sensors::CAM:
      return "CAM";

    case definitions::env_model_outputs::FUSED_OBJECTS:
      return "FUSED";
    case definitions::env_model_outputs::RT_ASSOCIATIONS:
      return "RT_ASSOC";

    default:
      return "NOT_FOUND";
    }
  }

  /**
   * @warning Changes have to be made if a new sensor or output is added.
   */
  static std::string sourceToLetter(uint8_t source_id)
  {
    switch (source_id)
    {
    case definitions::input_sensors::SOURCE_NO_SOURCE:
      return "_";
    case definitions::input_sensors::ESR_FRONT:
      return "R";
    case definitions::input_sensors::ESR_REAR:
      return "R";
    case definitions::input_sensors::RADAR:
      return "R";
    case definitions::input_sensors::EPM:
      return "E";
    case definitions::input_sensors::LUX:
      return "L";
    case definitions::input_sensors::SRR_FRONT:
      return "R";
    case definitions::input_sensors::RT:
      return "3";
    case definitions::input_sensors::SRR_REAR:
      return "R";
    case definitions::input_sensors::SRR_LEFT:
      return "R";
    case definitions::input_sensors::SRR_RIGHT:
      return "R";
    case definitions::input_sensors::SCALA:
      return "S";
    case definitions::input_sensors::VLP:
      return "L";
    case definitions::input_sensors::RTRANGE:
      return "T";
    case definitions::input_sensors::LIDAR_RS:
      return "L";
    case definitions::input_sensors::CAMERA:
      return "C";
    case definitions::input_sensors::CAM:
      return "C";
    case definitions::env_model_outputs::FUSED_OBJECTS:
      return "F";
    case definitions::env_model_outputs::RT_ASSOCIATIONS:
      return "A";

    default:
      return "?";
    }
  }

  /**
   * @brief Will generate a unique name for an object based on information about the object.
   * @param index Index of the object in the list.
   * @param object Actually object.
   * @return Unique name of object.
   */
  static std::string generateUniqueID(size_t index, definitions::IkaObject *object)
  {
    assert(object != nullptr);
    std::string unique_object_id = std::to_string(index);
    unique_object_id += "_" + std::to_string(object->IdExternal);
    unique_object_id += "_" + std::to_string(object->IdInternal);
    return unique_object_id;
  }

  /**
   * @brief Will clear all object from an object list.
   * @details Also clears objects that are not in range of the definitions::IkaObjectList::objects.size().
   * @param object_list Object list to be cleared.
   */
  static void clearObjectList(definitions::IkaObjectList *object_list)
  {
    assert(object_list != nullptr);
    object_list->objects.clear();
  }

  /**
   * @brief Will clear all object from an object list.
   * @details Also clears objects that are not in range of the
   * definitions::IkaObjectList::objects.size().
   * @param object_list Object list to be cleared.
   */
  static void clearObjectList(GlobalObjectList *object_list)
  {
    assert(object_list != nullptr);
    object_list->objects.clear();
  }

  /**
   * @brief Will calculate the first gap in a object list.
   * @param object_list Object list to search in.
   * @return The first invalid index in a list, object_list.objects.size() if no
   * gap were found.
   */
  static size_t calculateFirstInvalidLocationInList(
      const definitions::IkaObjectList *object_list)
  {
    assert(object_list != nullptr);
    for (size_t i = 0; i < object_list->objects.size(); i++)
    {
      if (!object_list->objects[i].bObjectValid)
      {
        return i;
      }
    }
    return object_list->objects.size();
  }

  /**
   * @brief Will calculate the first gap in a object list.
   * @param object_list Object list to search in.
   * @return The first invalid index in a list, object_list.objects.size() if no gap were found.
   */
  static size_t calculateFirstInvalidLocationInList(
      const GlobalObjectList *object_list)
  {
    assert(object_list != nullptr);
    for (size_t i = 0; i < object_list->objects.size(); i++)
    {
      if (!object_list->objects[i].bObjectValid)
      {
        return i;
      }
    }
    return object_list->objects.size();
  }

  /**
   * @brief Calculates the index of the last valid object
   * @param object_list Object list to search in.
   * @return The last valid index in a list, -1 if no object were found.
   */
  static size_t maxIdxValid(const definitions::IkaObjectList *object_list)
  {
    assert(object_list != nullptr);
    size_t max_valid_idx = -1;
    for (size_t index = 0; index < object_list->objects.size(); ++index)
    {
      if (object_list->objects[index].bObjectValid)
        max_valid_idx = index;
    }
    return max_valid_idx;
  }

  /**
   * @brief Calculates the index of the last valid object
   * @param object_list Object list to search in.
   * @return The last valid index in a list, -1 if no object was found.
   */
  static size_t maxIdxValid(const GlobalObjectList *object_list)
  {
    assert(object_list != nullptr);
    size_t max_valid_idx = -1;
    for (size_t index = 0; index < object_list->objects.size(); ++index)
    {
      if (object_list->objects[index].bObjectValid)
        max_valid_idx = index;
    }
    return max_valid_idx;
  }

  /**
   * @brief Calculates the number of valid objects.
   * @param object_list Object list to search in.
   * @return The number of valid objects in a list, 0 if no object were found.
   */
  static size_t numberValidObjects(const definitions::IkaObjectList *object_list)
  {
    assert(object_list != nullptr);
    return count_if(object_list->objects.begin(), object_list->objects.end(), [](definitions::IkaObject object) { return object.bObjectValid; });
  }

  /**
   * @brief Calculates the number of valid objects.
   * @param object_list Object list to search in.
   * @return The number of valid objects in a list, 0 if no object was found.
   */
  static size_t numberValidObjects(const GlobalObjectList *object_list)
  {
    assert(object_list != nullptr);
    return count_if(
        object_list->objects.begin(), object_list->objects.end(),
        [](definitions::IkaObject object) { return object.bObjectValid; });
  }

  /**
   * @brief Trims the input list by deleting all invalid objects at the end of a
   * list.
   * @param object_list Object list to search in.
   * @return A list without invalid objects at the end.
   */
  static void trimIkaObjectList(definitions::IkaObjectList *object_list)
  {
    assert(object_list != nullptr);
    auto iterator = object_list->objects.rbegin();
    while (iterator != object_list->objects.rend() && !(*iterator).bObjectValid)
    {
      object_list->objects.pop_back();
      ++iterator;
    }
  }

  /**
   * @brief Trims the input list by deleting all invalid objects at the end of a
   * list.
   * @param object_list Object list to search in.
   * @return A list without invalid objects at the end.
   */
  static void trimIkaObjectList(GlobalObjectList *object_list)
  {
    assert(object_list != nullptr);
    auto iterator = object_list->objects.begin();
    while (iterator != object_list->objects.end())
    {
      if (!(*iterator).bObjectValid)
      {
        iterator = object_list->objects.erase(iterator);
      }
      else
      {
        ++iterator;
      }
    }
  }

  /**
   * @brief Converts XmlRpcVector to Eigen::VectorXf
   * @param input XmlRpc as double
   * @return Eigen::VectorXf
   */
  static Eigen::VectorXf XmlRpcVectorToEigenVectorXf(XmlRpc::XmlRpcValue &input)
  {
    ROS_ASSERT(input.getType() == XmlRpc::XmlRpcValue::TypeArray);
    Eigen::VectorXf vector(input.size());

    for (int32_t i = 0; i < input.size(); ++i)
    {
      if (input[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        vector(i) = float(static_cast<int>(input[i]));
      }
      else if (input[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {
        vector(i) = float(static_cast<double>(input[i]));
      }
      else if (input[i].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      {
        vector(i) = float(static_cast<bool>(input[i]));
      }
      else
      {
        ROS_FATAL("Invalid Type of input!");
        return Eigen::VectorXf(0);
      }
    }

    return vector;
  }

  /**
   * @brief Converts XmlRpcMatrix to Eigen::MatrixXf
   * @param input XmlRpc as double
   * @return Eigen::VectorXf
   */
  static Eigen::MatrixXf XmlRpcMatrixToEigenMatrixXf(XmlRpc::XmlRpcValue &input)
  {
    ROS_ASSERT(input.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(input[0].getType() == XmlRpc::XmlRpcValue::TypeArray);
    Eigen::MatrixXf matrix(input.size(), input[0].size());

    for (int32_t i = 0; i < input.size(); ++i)
    {
      for (int32_t j = 0; j < input[0].size(); ++j)
      {
        if (input[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
          matrix(i, j) = float(static_cast<int>(input[i][j]));
        }
        else if (input[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
          matrix(i, j) = float(static_cast<double>(input[i][j]));
        }
        else if (input[i][j].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
        {
          matrix(i, j) = float(static_cast<bool>(input[i][j]));
        }
        else
        {
          ROS_FATAL("Invalid Type of input!");
          return Eigen::MatrixXf(0, 0);
        }
      }
    }

    return matrix;
  }

  /**
   * @brief return ikaObject position as an vector(x,y,z)
   * @param ikaObject
   * @return vector of object's position(x,y,z)
   */
  static std::vector<float> getObjectPosition(const definitions::IkaObject &myIkaObj)
  {
    std::vector<float> position(3);
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      position[0] = myIkaObj.fMean[(int)definitions::ca_model::posX];
      position[1] = myIkaObj.fMean[(int)definitions::ca_model::posY];
      position[2] = myIkaObj.fMean[(int)definitions::ca_model::posZ];
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      position[0] = myIkaObj.fMean[(int)definitions::ctra_model::posX];
      position[1] = myIkaObj.fMean[(int)definitions::ctra_model::posY];
      position[2] = myIkaObj.fMean[(int)definitions::ctra_model::posZ];
    }
    return position;
  }

    static void setObjectPositionXY(definitions::IkaObject &myIkaObj, const float posX, const float posY)
  {
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      myIkaObj.fMean[(int)definitions::ca_model::posX] = posX;
      myIkaObj.fMean[(int)definitions::ca_model::posY] = posY;
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      myIkaObj.fMean[(int)definitions::ctra_model::posX] = posX;
      myIkaObj.fMean[(int)definitions::ctra_model::posY] = posY;
    }
  }


  /**
   * @brief return ikaObject position variance as an vector(x,y,z)
   * @param ikaObject
   * @return vector of object's position variance(x,y,z)
   */
  static std::vector<float> getObjectPositionVar(const definitions::IkaObject &myIkaObj)
  {
    std::vector<float> positionVar(3);
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      positionVar[0] = myIkaObj.fCovariance[(int)definitions::ca_model_variance::posX];
      positionVar[1] = myIkaObj.fCovariance[(int)definitions::ca_model_variance::posY];
      positionVar[2] = myIkaObj.fCovariance[(int)definitions::ca_model_variance::posZ];
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      positionVar[0] = myIkaObj.fCovariance[(int)definitions::ctra_model_variance::posX];
      positionVar[1] = myIkaObj.fCovariance[(int)definitions::ctra_model_variance::posY];
      positionVar[2] = myIkaObj.fCovariance[(int)definitions::ctra_model_variance::posZ];
    }
    return positionVar;
  }

  /**
   * @brief return ikaObject velocity as an vector(x,y)
   * @param ikaObject
   * @return vector of object's velocity(x,y)
   */
  static std::vector<float> getObjectVelocity(const definitions::IkaObject &myIkaObj)
  { 
    std::vector<float> velocity(2);
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      velocity[0] = myIkaObj.fMean[(int)definitions::ca_model::absVelX];
      velocity[1] = myIkaObj.fMean[(int)definitions::ca_model::absVelY];
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      velocity[0] = myIkaObj.fMean[(int)definitions::ctra_model::absVel] *
                std::cos(myIkaObj.fMean[(int)definitions::ctra_model::heading]);
      velocity[1] = myIkaObj.fMean[(int)definitions::ctra_model::absVel] *
                std::sin(myIkaObj.fMean[(int)definitions::ctra_model::heading]);
    }
    return velocity;
  }

  static void setObjectVelocity(definitions::IkaObject &myIkaObj, const float absVelX, const float absVelY)
  {
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      myIkaObj.fMean[(int)definitions::ca_model::absVelX] = absVelX;
      myIkaObj.fMean[(int)definitions::ca_model::absVelY] = absVelY;
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      myIkaObj.fMean[(int)definitions::ctra_model::absVel] = absVelX / 
              std::cos(myIkaObj.fMean[(int)definitions::ctra_model::heading]);
    }
  }


    /**
   * @brief return ikaObject velocity variance as an vector(x,y)
   * @param ikaObject
   * @return vector of object's velocity variance(x,y)
   */
  static std::vector<float> getObjectVelocityVar(const definitions::IkaObject &myIkaObj)
  { 
    std::vector<float> velocityVar(2);
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      velocityVar[0] = myIkaObj.fCovariance[(int)definitions::ca_model_variance::absVelX];
      velocityVar[1] = myIkaObj.fCovariance[(int)definitions::ca_model_variance::absVelY];
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      velocityVar[0] = myIkaObj.fCovariance[(int)definitions::ctra_model_variance::absVel];
      velocityVar[1] = myIkaObj.fCovariance[(int)definitions::ctra_model_variance::absVel];
    }
    return velocityVar;
  }

  /**
   * @brief return ikaObject acceleration as an vector(x,y)
   * @param ikaObject
   * @return vector of object's acceleration(x,y)
   */
  static std::vector<float> getObjectAcceleration(const definitions::IkaObject &myIkaObj)
  { 
    std::vector<float> acceleration(2);
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      acceleration[0] = myIkaObj.fMean[(int)definitions::ca_model::absAccX];
      acceleration[1] = myIkaObj.fMean[(int)definitions::ca_model::absAccY];
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      acceleration[0] = myIkaObj.fMean[(int)definitions::ctra_model::absAcc] *
                std::cos(myIkaObj.fMean[(int)definitions::ctra_model::heading]);
      acceleration[1] = myIkaObj.fMean[(int)definitions::ctra_model::absAcc] *
                std::sin(myIkaObj.fMean[(int)definitions::ctra_model::heading]);
    }
    return acceleration;
  }

  static void setObjectAcceleration(definitions::IkaObject &myIkaObj, const float absAccX, const float absAccY)
  {
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      myIkaObj.fMean[(int)definitions::ca_model::absAccX] = absAccX;
      myIkaObj.fMean[(int)definitions::ca_model::absAccY] = absAccY;
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      myIkaObj.fMean[(int)definitions::ctra_model::absAcc] = absAccX / 
              std::cos(myIkaObj.fMean[(int)definitions::ctra_model::heading]);
    }
  }



  /**
   * @brief return ikaObject acceleration variance as an vector(x,y)
   * @param ikaObject
   * @return vector of object's acceleration variance(x,y)
   */
  static std::vector<float> getObjectAccelerationVar(const definitions::IkaObject &myIkaObj)
  { 
    std::vector<float> accelerationVar(2);
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      accelerationVar[0] = myIkaObj.fCovariance[(int)definitions::ca_model_variance::absAccX];
      accelerationVar[1] = myIkaObj.fCovariance[(int)definitions::ca_model_variance::absAccY];
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      accelerationVar[0] = myIkaObj.fCovariance[(int)definitions::ctra_model_variance::absAcc]; 
      accelerationVar[1] = myIkaObj.fCovariance[(int)definitions::ctra_model_variance::absAcc];
    }
    return accelerationVar;
  }

  /**
   * @brief return ikaObject size as an vector(length, width, height)
   * @param ikaObject
   * @return vector of object's size(length, width, height)
   */
  static std::vector<float> getObjectSize(const definitions::IkaObject &myIkaObj)
  { 
    std::vector<float> size(3);
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
      (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      size[0] = myIkaObj.fMean[(int)definitions::ca_model::length];
      size[1] = myIkaObj.fMean[(int)definitions::ca_model::width];
      size[2] = myIkaObj.fMean[(int)definitions::ca_model::height];
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      size[0] = myIkaObj.fMean[(int)definitions::ctra_model::length];
      size[1] = myIkaObj.fMean[(int)definitions::ctra_model::width];
      size[2] = myIkaObj.fMean[(int)definitions::ctra_model::height];
    }
    return size;
  }

  /**
   * @brief return ikaObject size variance as an vector(length, width, height)
   * @param ikaObject
   * @return vector of object's size variance(length, width, height)
   */
  static std::vector<float> getObjectSizeVar(const definitions::IkaObject &myIkaObj)
  { 
    std::vector<float> sizeVar(3);
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
      (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      sizeVar[0] = myIkaObj.fCovariance[(int)definitions::ca_model_variance::length];
      sizeVar[1] = myIkaObj.fCovariance[(int)definitions::ca_model_variance::width];
      sizeVar[2] = myIkaObj.fCovariance[(int)definitions::ca_model_variance::height];
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      sizeVar[0] = myIkaObj.fCovariance[(int)definitions::ctra_model_variance::length];
      sizeVar[1] = myIkaObj.fCovariance[(int)definitions::ctra_model_variance::width];
      sizeVar[2] = myIkaObj.fCovariance[(int)definitions::ctra_model_variance::height];
    }
    return sizeVar;
  }

  /**
   * @brief return ikaObject heading
   * @param ikaObject
   * @return object's heading
   */
  static float getObjectHeading(const definitions::IkaObject &myIkaObj)
  {

    float fHeading;
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      fHeading = myIkaObj.fMean[(int)definitions::ca_model::heading];
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      fHeading = myIkaObj.fMean[(int)definitions::ctra_model::heading];
    } else {
      throw std::invalid_argument("wrong object type");
    }
    return fHeading;
  }

  /**
   * @brief return ikaObject heading variance
   * @param ikaObject
   * @return object's heading variance
   */
  static float getObjectHeadingVar(const definitions::IkaObject &myIkaObj)
  {

    float fHeadingVar;
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      fHeadingVar = myIkaObj.fCovariance[(int)definitions::ca_model_variance::heading];
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      fHeadingVar = myIkaObj.fCovariance[(int)definitions::ctra_model_variance::heading];
    }
    return fHeadingVar;
  }

  static void setObjectHeading(definitions::IkaObject &myIkaObj, float fHeading)
  {

    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      myIkaObj.fMean[(int)definitions::ca_model::heading] = fHeading;
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
                myIkaObj.fMean[(int)definitions::ctra_model::heading] = fHeading;
    }
  }

  static void setObjectHeadingVar(definitions::IkaObject &myIkaObj, float fHeadingVar)
  {
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      myIkaObj.fCovariance[(int)definitions::ca_model_variance::heading] = fHeadingVar;
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      myIkaObj.fCovariance[(int)definitions::ctra_model_variance::heading] = fHeadingVar;
    }
  }



  /**
   * @brief return ikaObject yawrate
   * @param ikaObject
   * @return object's yawrate
   */
  static float getObjectYawrate(const definitions::IkaObject &myIkaObj)
  { 

    float fYawrate;
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      fYawrate = 0.0f;
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      fYawrate = myIkaObj.fMean[(int)definitions::ctra_model::yawrate];
    }
    return fYawrate;
  }

  /**
   * @brief return ikaObject yawrate variance
   * @param ikaObject
   * @return object's yawrate variance
   */
  static float getObjectYawrateVar(const definitions::IkaObject &myIkaObj)
  { 

    float fYawrateVar;
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      fYawrateVar = -1.0f;
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      fYawrateVar = myIkaObj.fCovariance[(int)definitions::ctra_model_variance::yawrate];
    }
    return fYawrateVar;
  }

  /**
   * @brief return ikaObject variance vector
   * @param ikaObject
   * @return object's variance vector
   */
  static std::vector<float> getObjectVarianceVector(definitions::IkaObject &myIkaObj)
  { 
    std::vector<float> variance_vector;
    int variance_vector_length = sqrt(myIkaObj.fCovariance.size());
    variance_vector.resize(variance_vector_length);
    for(int i=0; i < variance_vector_length; i++)
    {
      variance_vector[i] = myIkaObj.fCovariance[i *variance_vector_length + i];
    }
    return variance_vector;
  }

  /**
   * @brief return covariance matrix (as a vector) with variance_vector as main diagonal (all other values = 0)
   * @param variance_vector
   * @return Covariancematrix as vector
   */
  static std::vector<float> CovarianceFromVariance(std::vector<float> &variance_vector)
  {
    std::vector<float> fCovariance;
    fCovariance.resize(pow(variance_vector.size(),2));
    for(int i=0; i< variance_vector.size(); i++) {
      fCovariance[i * variance_vector.size() + i] = variance_vector[i];
    }
    return fCovariance;
  }

  /**
   * @brief return covariance matrix (as an vector) retrieved from GlobalObject.P
   * @param GlobalObject
   * @return Covariancematrix as vector
   */
  static std::vector<float> getCovarianceVecFromP(const GlobalObject &object)
  {
    int stateVecLength = getStateVecLength(object);
    std::vector<float> fCovariance(pow(stateVecLength,2));
    if((object.IdMotionModel == definitions::motion_model::CV) || 
       (object.IdMotionModel == definitions::motion_model::CA)) {
      fCovariance[(int)definitions::ca_model_variance::heading] = object.fCovariance[(int)definitions::ca_model_variance::heading];
    }
    for(int i=0; i<object.P().rows(); i++) //row
    {
      for(int j=0; j<object.P().cols(); j++)  //column
      {
        fCovariance[i * stateVecLength + j] = object.P()(i,j);
      }
    }
    
    return fCovariance;
  }

  /**
   * @brief return covariance matrix (as eigen matrix) retrieved from ikaObject.fCovariance
   * @param ikaObject
   * @return Covariancematrix as eigen matrix
   */
  static Eigen::MatrixXf getEigenCovarianceFromObject(const definitions::IkaObject& myIkaObj)
  {
    int stateVecLength = getStateVecLength(myIkaObj);
    Eigen::MatrixXf Covariance(stateVecLength, stateVecLength);
    for(int i=0; i<stateVecLength; i++) {
      for(int j=0; j<stateVecLength; j++){
        Covariance(i,j) = myIkaObj.fCovariance[i*stateVecLength+j];
      }
    }
    return Covariance;
  }

  /**
   * @brief returns pointer to an ikaObject state vector begin that can be used as an array
   * @param pointer to ikaObject
   * @return pointer to first position of state vector of that ikaObject
   */
  static float *getStateVec(definitions::IkaObject *myIkaObj)
  {
    float *myStateArray = &myIkaObj->fMean[0];
    return myStateArray;
  }

  /**
   * @brief returns pointer to an ikaObject state vector begin that can be used as
   * an array
   * @param pointer to ikaObject
   * @return pointer to first position of state vector of that ikaObject
   */
  static const float *getStateVec(const definitions::IkaObject *myIkaObj)
  {
    const float *myStateArray = &myIkaObj->fMean[0];
    return myStateArray;
  }

  /**
   * @brief return ikaObject state vector as an eigen vector, using original memory
   * @param pointer to ikaObject
   * @return ikaObject state vector as an eigen vector
   */
  static Eigen::Map<Eigen::VectorXf> getEigenStateVec(definitions::IkaObject *myIkaObj)
  {
    float *mapStart = getStateVec(myIkaObj);
    auto stateVecLength = getStateVecLength(myIkaObj);
    if((myIkaObj->IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj->IdMotionModel == definitions::motion_model::CA)) {
      stateVecLength -= 1;
    }
    Eigen::Map<Eigen::VectorXf> myMap(mapStart, stateVecLength);
    return myMap;
  }

  /**
   * @brief return ikaObject state vector as an eigen vector, using original
   * memory
   * @param pointer to ikaObject
   * @return ikaObject state vector as an eigen vector
   */
  static Eigen::Map<const Eigen::VectorXf> getEigenStateVec(
      const definitions::IkaObject *myIkaObj)
  {
    const float *mapStart = getStateVec(myIkaObj);
    auto stateVecLength = getStateVecLength(myIkaObj);
    if((myIkaObj->IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj->IdMotionModel == definitions::motion_model::CA)) {
      stateVecLength -= 1;
    }
    Eigen::Map<const Eigen::VectorXf> myMap(mapStart, stateVecLength);
    return myMap;
  }

  /**
   * @brief returns pointer to an ikaObject variance vector begin that can be used as an array
   * @param pointer to ikaObject
   * @return pointer to first position of variance vector of that ikaObject
   */
  static float *getVarianceVec(definitions::IkaObject *myIkaObj)
  {
    float *myVarianceArray = &myIkaObj->fCovariance[0];
    return myVarianceArray;
  }

  /**
   * @brief returns pointer to an ikaObject variance vector begin that can be
   * used as an array
   * @param pointer to ikaObject
   * @return pointer to first position of variance vector of that ikaObject
   */
  static const float *getVarianceVec(const definitions::IkaObject *myIkaObj)
  {
    const float *myVarianceArray = &myIkaObj->fCovariance[0];
    return myVarianceArray;
  }

  /**
   * @brief return ikaObject variance vector as an eigen vector, using original memory
   * @param pointer to ikaObject
   * @return ikaObject variance vector as an eigen vector
   */
  static Eigen::Map<Eigen::VectorXf, 0, Eigen::InnerStride<> > getEigenVarianceVec(definitions::IkaObject *myIkaObj)
  {
    float *mapStart = getVarianceVec(myIkaObj);
    auto stateVecLength = getStateVecLength(myIkaObj);
    int strideSize = stateVecLength+1;
    if((myIkaObj->IdMotionModel == definitions::motion_model::CV) || 
       (myIkaObj->IdMotionModel == definitions::motion_model::CA)) {
      stateVecLength -= 1;
    }
    Eigen::Map<Eigen::VectorXf, 0, Eigen::InnerStride<> > myMap(mapStart, stateVecLength, Eigen::InnerStride<>(strideSize));
    return myMap;
  }

  /**
   * @brief return ikaObject variance vector as an eigen vector, using original
   * memory
   * @param pointer to ikaObject
   * @return ikaObject variance vector as an eigen vector
   */
  static Eigen::Map<const Eigen::VectorXf, 0, Eigen::InnerStride<> >  getEigenVarianceVec(
      const definitions::IkaObject *myIkaObj)
  {
    const float *mapStart = getVarianceVec(myIkaObj);
    auto stateVecLength = getStateVecLength(myIkaObj);
    int strideSize = stateVecLength+1;
    if((myIkaObj->IdMotionModel == definitions::motion_model::CV) || 
       (myIkaObj->IdMotionModel == definitions::motion_model::CA)) {
      stateVecLength -= 1;
    } 
    Eigen::Map<const Eigen::VectorXf, 0, Eigen::InnerStride<> > myMap(mapStart, stateVecLength, Eigen::InnerStride<>(strideSize));
    return myMap;
  }

  /**
   * @brief returns size of state vector
   * @return size of state vector of ikaObject
   */
  static int getStateVecLength(const definitions::IkaObject& myIkaObj)
  {
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      return (int)definitions::ca_model::COUNT;
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      return (int)definitions::ctra_model::COUNT;
    } else {
      return 0;
    }
  }

  /**
   * @brief returns size of state vector
   * @return size of state vector of ikaObject
   */
  static int getStateVecLength(const definitions::IkaObject *myIkaObj)
  {
    if((myIkaObj->IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj->IdMotionModel == definitions::motion_model::CA)) {
      return (int)definitions::ca_model::COUNT;
    } else if((myIkaObj->IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj->IdMotionModel == definitions::motion_model::CTRA)) {
      return (int)definitions::ctra_model::COUNT;
    } else {
      return 0;
    }
  }

  /**
   * @brief returns size of state vector
   * @return size of state vector of ikaObject
   */
  static int getStateVecLength( definitions::IkaObject& myIkaObj)
  {
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      return (int)definitions::ca_model::COUNT;
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      return (int)definitions::ctra_model::COUNT;
    } else {
      return 0;
    }
  }

  static int getStateVecLengthConst(const definitions::IkaObject& myIkaObj)
  {
    if((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj.IdMotionModel == definitions::motion_model::CA)) {
      return (int)definitions::ca_model::COUNT;
    } else if((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) {
      return (int)definitions::ctra_model::COUNT;
    } else {
      return 0;
    }
  }

  /**
   * @brief returns size of state vector
   * @return size of state vector of ikaObject
   */
  static int getStateVecLength(definitions::IkaObject *myIkaObj)
  {
    if((myIkaObj->IdMotionModel == definitions::motion_model::CV) || 
        (myIkaObj->IdMotionModel == definitions::motion_model::CA)) {
      return (int)definitions::ca_model::COUNT;
    } else if((myIkaObj->IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj->IdMotionModel == definitions::motion_model::CTRA)) {
      return (int)definitions::ctra_model::COUNT;
    } else {
      return 0;
    }
  }

  /**
   * @brief Changes object's motion model to destination model
   * @return ikaObject in new motion model
   */
  static void convertMotionModel(definitions::IkaObject &myIkaObj, int motion_model)
  {
    std::vector<float> fMean_init = myIkaObj.fMean;
    std::vector<float> variance_init = getObjectVarianceVector(myIkaObj);
    std::vector<float> variance;
    variance.resize((int)definitions::ctra_model::COUNT);

    if(
      ((myIkaObj.IdMotionModel == definitions::motion_model::CV) || 
       (myIkaObj.IdMotionModel == definitions::motion_model::CA)) &&
      ((motion_model == definitions::motion_model::CTRV) || 
       (motion_model == definitions::motion_model::CTRA)))
    {
      myIkaObj.fMean.resize((int)definitions::ctra_model::COUNT);
      variance.resize((int)definitions::ctra_model::COUNT);
      myIkaObj.fMean[(int)definitions::ctra_model::posX] = fMean_init[(int)definitions::ca_model::posX];
      myIkaObj.fMean[(int)definitions::ctra_model::posY] = fMean_init[(int)definitions::ca_model::posY];
      myIkaObj.fMean[(int)definitions::ctra_model::posZ] = fMean_init[(int)definitions::ca_model::posZ];
      myIkaObj.fMean[(int)definitions::ctra_model::absVel] = sqrt(pow(fMean_init[(int)definitions::ca_model::absVelX],2) +
                                                                  pow(fMean_init[(int)definitions::ca_model::absVelY],2));
      myIkaObj.fMean[(int)definitions::ctra_model::absAcc] = sqrt(pow(fMean_init[(int)definitions::ca_model::absAccX],2) +
                                                                  pow(fMean_init[(int)definitions::ca_model::absAccY],2));
      myIkaObj.fMean[(int)definitions::ctra_model::heading] = fMean_init[(int)definitions::ca_model::heading];
      myIkaObj.fMean[(int)definitions::ctra_model::yawrate] = 0.0f;
      myIkaObj.fMean[(int)definitions::ctra_model::length] = fMean_init[(int)definitions::ca_model::length];
      myIkaObj.fMean[(int)definitions::ctra_model::width] = fMean_init[(int)definitions::ca_model::width];
      myIkaObj.fMean[(int)definitions::ctra_model::height] = fMean_init[(int)definitions::ca_model::height];


      variance[(int)definitions::ctra_model::posX] = variance_init[(int)definitions::ca_model::posX];
      variance[(int)definitions::ctra_model::posY] = variance_init[(int)definitions::ca_model::posY];
      variance[(int)definitions::ctra_model::posZ] = variance_init[(int)definitions::ca_model::posZ];
      if( (variance_init[(int)definitions::ca_model::absVelX] <= 0) || (variance_init[(int)definitions::ca_model::absVelY] <= 0))
      { 
        variance[(int)definitions::ctra_model::absVel] = -1.0f;
      } else {
        variance[(int)definitions::ctra_model::absVel] = sqrt(pow(variance_init[(int)definitions::ca_model::absVelX],2) +
                                                              pow(variance_init[(int)definitions::ca_model::absVelY],2));
      }
      if( (variance_init[(int)definitions::ca_model::absAccX] <= 0) || (variance_init[(int)definitions::ca_model::absAccY] <= 0))
      { 
        variance[(int)definitions::ctra_model::absAcc] = -1.0f;
      } else {
        variance[(int)definitions::ctra_model::absAcc] = sqrt(pow(variance_init[(int)definitions::ca_model::absAccX],2) +
                                                              pow(variance_init[(int)definitions::ca_model::absAccY],2));
      }
      variance[(int)definitions::ctra_model::heading] = variance_init[(int)definitions::ca_model::heading];
      variance[(int)definitions::ctra_model::yawrate] = -1.0f;
      variance[(int)definitions::ctra_model::length] = variance_init[(int)definitions::ca_model::length];
      variance[(int)definitions::ctra_model::width] = variance_init[(int)definitions::ca_model::width];
      variance[(int)definitions::ctra_model::height] = variance_init[(int)definitions::ca_model::height];

      myIkaObj.IdMotionModel = definitions::motion_model::CTRA;

      if(motion_model == definitions::motion_model::CTRV) {
        myIkaObj.fMean[(int)definitions::ctra_model::absAcc] = 0.0f;
        variance[(int)definitions::ctra_model::absAcc] = -1.0f;
        myIkaObj.IdMotionModel = definitions::motion_model::CTRV;
      }

      myIkaObj.fCovariance = CovarianceFromVariance(variance);

    } else if (
        ((myIkaObj.IdMotionModel == definitions::motion_model::CTRV) || 
         (myIkaObj.IdMotionModel == definitions::motion_model::CTRA)) &&
        ((motion_model == definitions::motion_model::CV) || 
         (motion_model == definitions::motion_model::CA)))
    {
      myIkaObj.fMean.resize((int)definitions::ca_model::COUNT);
      variance.resize((int)definitions::ca_model::COUNT);
      myIkaObj.fMean[(int)definitions::ca_model::posX] = fMean_init[(int)definitions::ctra_model::posX];
      myIkaObj.fMean[(int)definitions::ca_model::posY] = fMean_init[(int)definitions::ctra_model::posY];
      myIkaObj.fMean[(int)definitions::ca_model::posZ] = fMean_init[(int)definitions::ctra_model::posZ];
      myIkaObj.fMean[(int)definitions::ca_model::absVelX] = fMean_init[(int)definitions::ctra_model::absVel] * 
                                                            std::cos(fMean_init[(int)definitions::ctra_model::heading]);
      myIkaObj.fMean[(int)definitions::ca_model::absVelY] = fMean_init[(int)definitions::ctra_model::absVel] * 
                                                            std::sin(fMean_init[(int)definitions::ctra_model::heading]);
      myIkaObj.fMean[(int)definitions::ca_model::absAccX] = fMean_init[(int)definitions::ctra_model::absAcc] * 
                                                            std::cos(fMean_init[(int)definitions::ctra_model::heading]);
      myIkaObj.fMean[(int)definitions::ca_model::absAccY] = fMean_init[(int)definitions::ctra_model::absAcc] * 
                                                            std::sin(fMean_init[(int)definitions::ctra_model::heading]);
      myIkaObj.fMean[(int)definitions::ca_model::length] = fMean_init[(int)definitions::ctra_model::length];
      myIkaObj.fMean[(int)definitions::ca_model::width] = fMean_init[(int)definitions::ctra_model::width];
      myIkaObj.fMean[(int)definitions::ca_model::height] = fMean_init[(int)definitions::ctra_model::height];
      myIkaObj.fMean[(int)definitions::ca_model::heading] = fMean_init[(int)definitions::ctra_model::heading];

      variance[(int)definitions::ca_model::posX] = variance_init[(int)definitions::ctra_model::posX];
      variance[(int)definitions::ca_model::posY] = variance_init[(int)definitions::ctra_model::posY];
      variance[(int)definitions::ca_model::posZ] = variance_init[(int)definitions::ctra_model::posZ];
      if(variance_init[(int)definitions::ctra_model::absVel] <= 0)
      { 
        variance[(int)definitions::ca_model::absVelX] = -1.0f;
        variance[(int)definitions::ca_model::absVelY] = -1.0f;
      } else {
        variance[(int)definitions::ca_model::absVelX] = variance_init[(int)definitions::ctra_model::absVel];
        variance[(int)definitions::ca_model::absVelX] = variance_init[(int)definitions::ctra_model::absVel];
      }
      if(variance_init[(int)definitions::ctra_model::absAcc] <= 0)
      { 
        variance[(int)definitions::ca_model::absAccX] = -1.0f;
        variance[(int)definitions::ca_model::absAccY] = -1.0f;
      } else {
        variance[(int)definitions::ca_model::absAccX] = variance_init[(int)definitions::ctra_model::absAcc];
        variance[(int)definitions::ca_model::absAccY] = variance_init[(int)definitions::ctra_model::absAcc];
      }
      variance[(int)definitions::ca_model::length] = variance_init[(int)definitions::ctra_model::length];
      variance[(int)definitions::ca_model::width] = variance_init[(int)definitions::ctra_model::width];
      variance[(int)definitions::ca_model::height] = variance_init[(int)definitions::ctra_model::height];
      variance[(int)definitions::ca_model::heading] = variance_init[(int)definitions::ctra_model::heading];

      myIkaObj.IdMotionModel = definitions::motion_model::CA;

      if(motion_model == definitions::motion_model::CV) {
        myIkaObj.fMean[(int)definitions::ca_model::absAccX] = 0.0f;
        myIkaObj.fMean[(int)definitions::ca_model::absAccY] = 0.0f;
        variance[(int)definitions::ca_model::absAccX] = -1.0f;
        variance[(int)definitions::ca_model::absAccY] = -1.0f;
        myIkaObj.IdMotionModel = definitions::motion_model::CV;
      }

      myIkaObj.fCovariance = CovarianceFromVariance(variance);

    } else if ((myIkaObj.IdMotionModel == definitions::motion_model::CTRA) &&
               (motion_model == definitions::motion_model::CTRV))
    {
      myIkaObj.fMean[(int)definitions::ctra_model::absAcc] = 0.0f;
      myIkaObj.fCovariance[(int)definitions::ctra_model_variance::absAcc] = -1.0f;
      myIkaObj.IdMotionModel = definitions::motion_model::CTRV;

    } else if ((myIkaObj.IdMotionModel == definitions::motion_model::CA) &&
               (motion_model == definitions::motion_model::CV))
    {
      myIkaObj.fMean[(int)definitions::ca_model::absAccX] = 0.0f;
      myIkaObj.fMean[(int)definitions::ca_model::absAccY] = 0.0f;
      myIkaObj.fCovariance[(int)definitions::ca_model_variance::absAccX] = -1.0f;
      myIkaObj.fCovariance[(int)definitions::ca_model_variance::absAccY] = -1.0f;
      myIkaObj.IdMotionModel = definitions::motion_model::CV;
    }
  }

  /**
   * @brief will limit the input to -M_PI <= input <= M_PI, adding or subtracting 2*pi if necessary
   * @param input, most likely radiant (double)
   * @return nothing
   */
  static void limitToPosNegPI(double &input)
  {
    while (input <= -M_PI)
    {
      input += M_PI_TIMES_2;
    }
    while (input > M_PI)
    {
      input -= M_PI_TIMES_2;
    }
  }

  /**
   * @brief will limit the input to -M_PI <= input <= M_PI, adding or subtracting 2*pi if necessary
   * @param input, most likely radiant (float)
   * @return nothing
   */
  static void limitToPosNegPI(float &input)
  {
    while (input <= -M_PI)
    {
      input += M_PI_TIMES_2;
    }
    while (input > M_PI)
    {
      input -= M_PI_TIMES_2;
    }
  }

  /**
   * @brief transforms an ikaObject using a geometry_msgs transform
   * This function does NOT alter the header of myIkaObj!
   * @param myIkaObj pointer to ikaObject (in- and output)
   * @param myTransform transform that has been determined previously
   */
  static void transformIkaObject(definitions::IkaObject *myIkaObj,
                                 geometry_msgs::TransformStamped *msgTransform)
  {
    if((myIkaObj->IdMotionModel == definitions::motion_model::CV) || 
       (myIkaObj->IdMotionModel == definitions::motion_model::CA)) {
      transformIkaObjectCa(myIkaObj, msgTransform);
    } else if((myIkaObj->IdMotionModel == definitions::motion_model::CTRV) || 
              (myIkaObj->IdMotionModel == definitions::motion_model::CTRA)) {
      transformIkaObjectCtra(myIkaObj, msgTransform);
    }
  }

  /**
   * @brief transforms an ikaObject using a geometry_msgs transform
   * This function does NOT alter the header of myIkaObj!
   * @param myIkaObj pointer to ikaObject (in- and output)
   * @param myTransform transform that has been determined previously
   */
  static void transformIkaObjectCa(definitions::IkaObject *myIkaObj,
                                 geometry_msgs::TransformStamped *msgTransform)
  {
    // operators
    tf2::Transform tfTransform;
    tf2::convert(msgTransform->transform, tfTransform);
    const tf2::Quaternion &rot = tfTransform.getRotation();

    // geometry_msgs-formatted ikaObject data
    geometry_msgs::PoseWithCovarianceStamped poseCov;
    poseCov.pose.pose.position.x = double(myIkaObj->fMean[(int)definitions::ca_model::posX]);
    poseCov.pose.pose.position.y = double(myIkaObj->fMean[(int)definitions::ca_model::posY]);
    poseCov.pose.pose.position.z = double(myIkaObj->fMean[(int)definitions::ca_model::posZ]);
    poseCov.pose.pose.orientation = tf::createQuaternionMsgFromYaw(myIkaObj->fMean[(int)definitions::ca_model::heading]);
    poseCov.pose.covariance[0] = double(myIkaObj->fCovariance[(int)definitions::ca_model_variance::posX]);
    poseCov.pose.covariance[7] = double(myIkaObj->fCovariance[(int)definitions::ca_model_variance::posY]);
    poseCov.pose.covariance[14] = double(myIkaObj->fCovariance[(int)definitions::ca_model_variance::posZ]);

    // tf2-formatted ikaObject data
    tf2::Vector3 vecVel(double(myIkaObj->fMean[(int)definitions::ca_model::absVelX]), 
                        double(myIkaObj->fMean[(int)definitions::ca_model::absVelY]), 0.0);
    tf2::Vector3 vecAcc(double(myIkaObj->fMean[(int)definitions::ca_model::absAccX]),
                        double(myIkaObj->fMean[(int)definitions::ca_model::absAccY]), 0.0);
    tf2::Vector3 vecVarVel(double(myIkaObj->fCovariance[(int)definitions::ca_model_variance::absVelX]),
                           double(myIkaObj->fCovariance[(int)definitions::ca_model_variance::absVelY]), 0.0);
    tf2::Vector3 vecVarAcc(double(myIkaObj->fCovariance[(int)definitions::ca_model_variance::absAccX]), 
                           double(myIkaObj->fCovariance[(int)definitions::ca_model_variance::absAccY]), 0.0);

    // actual geometrical transformations
    //for tf2 types
    vecVel = tf2::quatRotate(rot, vecVel);
    vecAcc = tf2::quatRotate(rot, vecAcc);
    vecVarVel = tf2::quatRotate(rot, vecVarVel);
    vecVarAcc = tf2::quatRotate(rot, vecVarAcc);
    // missing for own tf2-based transform implementation: heading.
    // therefore, pose with covariance stamped is used

    // for geometry_msgs types
    geometry_msgs::PoseWithCovarianceStamped poseCovResult(poseCov);
    tf2::doTransform(poseCov, poseCovResult, *msgTransform);

    // write back results into ikaObject
    myIkaObj->fMean[(int)definitions::ca_model::posX] = poseCovResult.pose.pose.position.x;
    myIkaObj->fMean[(int)definitions::ca_model::posY] = poseCovResult.pose.pose.position.y;
    myIkaObj->fMean[(int)definitions::ca_model::posZ] = poseCovResult.pose.pose.position.z;

    myIkaObj->fMean[(int)definitions::ca_model::absVelX] = vecVel.getX();
    myIkaObj->fMean[(int)definitions::ca_model::absVelY] = vecVel.getY();
    myIkaObj->fMean[(int)definitions::ca_model::absAccX] = vecAcc.getX();
    myIkaObj->fMean[(int)definitions::ca_model::absAccY] = vecAcc.getY();

    tf2::Quaternion headingTfQuat;
    tf2::convert(poseCovResult.pose.pose.orientation, headingTfQuat);
    myIkaObj->fMean[(int)definitions::ca_model::heading] = float(tf2::impl::getYaw(headingTfQuat));

    myIkaObj->fCovariance[(int)definitions::ca_model_variance::absVelX] = vecVarVel.getX();
    myIkaObj->fCovariance[(int)definitions::ca_model_variance::absVelY] = vecVarVel.getY();
    myIkaObj->fCovariance[(int)definitions::ca_model_variance::absAccX] = vecVarAcc.getX();
    myIkaObj->fCovariance[(int)definitions::ca_model_variance::absAccY] = vecVarAcc.getY();
  }

  /**
   * @brief transforms an ikaObject using a geometry_msgs transform
   * This function does NOT alter the header of myIkaObj!
   * @param myIkaObj pointer to ikaObject (in- and output)
   * @param myTransform transform that has been determined previously
   */
  static void transformIkaObjectCtra(definitions::IkaObject *myIkaObj,
                                 geometry_msgs::TransformStamped *msgTransform)
  {
    // operators
    tf2::Transform tfTransform;
    tf2::convert(msgTransform->transform, tfTransform);
    const tf2::Quaternion &rot = tfTransform.getRotation();

    // geometry_msgs-formatted ikaObject data
    geometry_msgs::PoseWithCovarianceStamped poseCov;
    poseCov.pose.pose.position.x = double(myIkaObj->fMean[(int)definitions::ctra_model::posX]);
    poseCov.pose.pose.position.y = double(myIkaObj->fMean[(int)definitions::ctra_model::posY]);
    poseCov.pose.pose.position.z = double(myIkaObj->fMean[(int)definitions::ctra_model::posZ]);
    poseCov.pose.pose.orientation = tf::createQuaternionMsgFromYaw(myIkaObj->fMean[(int)definitions::ctra_model::heading]);
    poseCov.pose.covariance[0] = double(myIkaObj->fCovariance[(int)definitions::ctra_model_variance::posX]);
    poseCov.pose.covariance[7] = double(myIkaObj->fCovariance[(int)definitions::ctra_model_variance::posY]);
    poseCov.pose.covariance[14] = double(myIkaObj->fCovariance[(int)definitions::ctra_model_variance::posZ]);

    // for geometry_msgs types
    geometry_msgs::PoseWithCovarianceStamped poseCovResult(poseCov);
    tf2::doTransform(poseCov, poseCovResult, *msgTransform);

    // write back results into ikaObject
    myIkaObj->fMean[(int)definitions::ctra_model::posX] = poseCovResult.pose.pose.position.x;
    myIkaObj->fMean[(int)definitions::ctra_model::posY] = poseCovResult.pose.pose.position.y;
    myIkaObj->fMean[(int)definitions::ctra_model::posZ] = poseCovResult.pose.pose.position.z;

    tf2::Quaternion headingTfQuat;
    tf2::convert(poseCovResult.pose.pose.orientation, headingTfQuat);
    myIkaObj->fMean[(int)definitions::ctra_model::heading] = float(tf2::impl::getYaw(headingTfQuat));

  }

/**
 * Evaluates the probabilty density function(normal distribution) with mean and std_dev with value x
 * @param mean: mean of normal distribution
 * @param std_dev: standard deviation of normal distribution
 * @param x: value, where to evaluate pdf
 * @return probability of value x
 */
  static double evaluate_pdf(const double &mean, const double &std_dev, const double &x)
  {
    // http://www.cplusplus.com/reference/random/normal_distribution/
    // ROS_ASSERT(!std::isnan(mean));
    // ROS_ASSERT(!std::isnan(std_dev));
    // ROS_ASSERT(std_dev != 0);
    // ROS_ASSERT(!std::isnan(x));
    double result = (1. / (std_dev * SQRT_M_PI_TIMES_2)) * exp(-((x - mean) * (x - mean)) / (2. * std_dev * std_dev));
    ROS_ASSERT(!std::isnan(result));
    return result;
  }


 /**
 * uses a std::chrono::_V2::system_clock::time_point and a message to display time, also resets the timer
 * @param msg: std::string, message to print, e.g. where the function was called
 * @param start: std::chrono::_V2::system_clock::time_point, the start time to calculate dt with
 * @return nothing, but message is printed to std::cout and start is reset
 */
  static void printTimeDiff(const std::string &msg, std::chrono::_V2::system_clock::time_point &start)
  {
    std::cout << std::setprecision(15)
              << msg
              << (std::chrono::high_resolution_clock::now() - start).count()
              << "ns"
              << std::endl;
    start = std::chrono::high_resolution_clock::now();
  }

  static definitions::IkaObjectList transformObjListToFrame(const tf::TransformListener& tf_listener, const definitions::IkaObjectList &obj_list_msg_in, const std::string &target_frame)
  {
    definitions::IkaObjectList obj_list_msg_out;
    try
    {
      obj_list_msg_out = obj_list_msg_in; //copy input object list
      tf::StampedTransform target_input_transform;

      tf_listener.lookupTransform(target_frame, obj_list_msg_in.header.frame_id, ros::Time(0), target_input_transform);
      geometry_msgs::TransformStamped msgTransform;
      tf::transformStampedTFToMsg(target_input_transform, msgTransform);

      for (uint i = 0; i < obj_list_msg_out.objects.size(); i++)
      {
        IkaUtilities::transformIkaObject(&obj_list_msg_out.objects.at(i), &msgTransform);
      }
      obj_list_msg_out.header.stamp = ros::Time::now();
      obj_list_msg_out.header.frame_id = target_frame;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("transformObjListToFrame: %s", ex.what());
      ros::Duration(0.1).sleep();
      obj_list_msg_out = transformObjListToFrame(tf_listener, obj_list_msg_in, target_frame);
    }

    return obj_list_msg_out;
  }


}; // end class IkaUtilities.cpp

/*
 [MHO]
 The following code could provide conversion functions from ikaObject to ROS geometry_msgs
 data types such that transforms work by means of the tf2 framework.
 However, we cannnot fulfill the requirement of overloading the toMsg functions since
 ikaObject stores position and velocity information in the very same data structure.
 This means we would have an ambiguity in the definition of the toMsg function.
 see also:
 http://wiki.ros.org/tf2/Tutorials/Quaternions
 http://wiki.ros.org/tf2/Tutorials/Create%20Data%20Conversion%20Package%20%28C%2B%2B%29

 [BAL]: Date: 14.04.2021
 Commented out, since noone uses it. 
 If needed, adaptions for new object format needs to be done.
*/

//namespace tf2 {

//  geometry_msgs::Vector3 toMsg(const tf2::Stamped<definitions::IkaObject>& in)
//  {
//    geometry_msgs::Vector3 msg;
//    msg.x = in.fAbsVelX;
//    msg.y = in.fAbsVelY;
//    msg.z = 0.0;
//    return msg;
//  }

//  geometry_msgs::Point toMsg(const tf2::Stamped<definitions::IkaObject>& in)
//  {
//    geometry_msgs::Point msg;
//    msg.x = in.fPosX;
//    msg.y = in.fPosY;
//    msg.z = in.fPosZ;
//    return msg;
//  }

//  void fromMsg(const geometry_msgs::Point& msg, tf2::Stamped<definitions::IkaObject>& out)
//  {
//    out.fPosX = msg.x;
//    out.fPosY = msg.y;
//    out.fPosZ = msg.z;
//  }

//  void fromMsg(const geometry_msgs::Vector3& msg, tf2::Stamped<definitions::IkaObject>& out)
//  {
//    out.fAbsVelX = msg.x;
//    out.fAbsVelY = msg.y;
//    // msg.z is not used
//  }
//namespace tf2 {

//  template <>
//  inline
//  void doTransform(const tf2::Stamped<definitions::IkaObject>& t_in,
//                   tf2::Stamped<definitions::IkaObject>& t_out,
//                   const geometry_msgs::TransformStamped& transform)
//  {

//    // the transformation has to be done here on our own.
//    // either through entirely own computations, or through manual conversion to ros data types and then using a ros transform.
//    t_out = t_in;
//    // t_out = tf2::Stamped<definitions::IkaObject>(t_in, transform.header.stamp, transform.header.frame_id);
//  }
//}

//
// [PPE]:
// The rosconsole macros implementing THROTTLE functionality (limit output per time) do not consider jumps back in time.
// However, these can occur when e.g. using Carmaker and restarting the simulation, resulting in these messages not being printed at all.
// We reimplement these macros with our IKA_ROS_ prefix which handle jumps back in time.
//
// Used sources:
// http://docs.ros.org/en/kinetic/api/rosconsole/html/macros__generated_8h_source.html
// http://docs.ros.org/en/kinetic/api/rosconsole/html/console_8h_source.html
//
#define IKA_ROS_LOG_THROTTLE(rate, level, name, ...) \
  do \
    { \
      ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
      static double last_hit = 0.0; \
      ::ros::Time now = ::ros::Time::now(); \
      if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && (ROS_UNLIKELY(last_hit + rate <= now.toSec()) || || ROS_UNLIKELY(last_hit > now.toSec()))) \
      { \
        last_hit = now.toSec(); \
        ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
      } \
    } while(false)

#define IKA_ROS_LOG_STREAM_THROTTLE(rate, level, name, args) \
  do \
    { \
      ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
      static double __ros_log_stream_throttle__last_hit__ = 0.0; \
      ::ros::Time __ros_log_stream_throttle__now__ = ::ros::Time::now(); \
      if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && (ROS_UNLIKELY(__ros_log_stream_throttle__last_hit__ + rate <= __ros_log_stream_throttle__now__.toSec()) || ROS_UNLIKELY(__ros_log_stream_throttle__last_hit__ > __ros_log_stream_throttle__now__.toSec()))) \
      { \
        __ros_log_stream_throttle__last_hit__ = __ros_log_stream_throttle__now__.toSec(); \
        ROSCONSOLE_PRINT_STREAM_AT_LOCATION(args); \
      } \
    } while(false)


#define IKA_ROS_LOG_DELAYED_THROTTLE(rate, level, name, ...) \
  do \
    { \
      ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
      ::ros::Time __ros_log_delayed_throttle__now__ = ::ros::Time::now(); \
      static double __ros_log_delayed_throttle__last_hit__ = __ros_log_delayed_throttle__now__.toSec(); \
      if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && (ROS_UNLIKELY(__ros_log_delayed_throttle__last_hit__ + rate <= __ros_log_delayed_throttle__now__.toSec()) || ROS_UNLIKELY(__ros_log_delayed_throttle__last_hit__ > __ros_log_delayed_throttle__now__.toSec()))) \
      { \
        __ros_log_delayed_throttle__last_hit__ = __ros_log_delayed_throttle__now__.toSec(); \
        ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
      } \
    } while(false)

#define IKA_ROS_LOG_STREAM_DELAYED_THROTTLE(rate, level, name, args) \
  do \
    { \
      ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
      ::ros::Time __ros_log_stream_delayed_throttle__now__ = ::ros::Time::now(); \
      static double __ros_log_stream_delayed_throttle__last_hit__ = __ros_log_stream_delayed_throttle__now__.toSec(); \
      if (ROS_UNLIKELY(__rosconsole_define_location__enabled) && (ROS_UNLIKELY(__ros_log_stream_delayed_throttle__last_hit__ + rate <= __ros_log_stream_delayed_throttle__now__.toSec()) || ROS_UNLIKELY(__ros_log_stream_delayed_throttle__last_hit__ > __ros_log_stream_delayed_throttle__now__.toSec()))) \
      { \
        __ros_log_stream_delayed_throttle__last_hit__ = __ros_log_stream_delayed_throttle__now__.toSec(); \
        ROSCONSOLE_PRINT_STREAM_AT_LOCATION(args); \
      } \
    } while(false)

#if (ROSCONSOLE_MIN_SEVERITY > ROSCONSOLE_SEVERITY_DEBUG)
  #define IKA_ROS_DEBUG_THROTTLE(rate, ...)
  #define IKA_ROS_DEBUG_STREAM_THROTTLE(rate, args)
  #define IKA_ROS_DEBUG_THROTTLE_NAMED(rate, name, ...)
  #define IKA_ROS_DEBUG_STREAM_THROTTLE_NAMED(rate, name, args)
  #define IKA_ROS_DEBUG_DELAYED_THROTTLE(rate, ...)
  #define IKA_ROS_DEBUG_STREAM_DELAYED_THROTTLE(rate, args)
  #define IKA_ROS_DEBUG_DELAYED_THROTTLE_NAMED(rate, name, ...)
  #define IKA_ROS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(rate, name, args)
#else
  #define IKA_ROS_DEBUG_THROTTLE(rate, ...) IKA_ROS_LOG_THROTTLE(rate, ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
  #define IKA_ROS_DEBUG_STREAM_THROTTLE(rate, args) IKA_ROS_LOG_STREAM_THROTTLE(rate, ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, args)
  #define IKA_ROS_DEBUG_THROTTLE_NAMED(rate, name, ...) IKA_ROS_LOG_THROTTLE(rate, ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
  #define IKA_ROS_DEBUG_STREAM_THROTTLE_NAMED(rate, name, args) IKA_ROS_LOG_STREAM_THROTTLE(rate, ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)
  #define IKA_ROS_DEBUG_DELAYED_THROTTLE(rate, ...) IKA_ROS_LOG_DELAYED_THROTTLE(rate, ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
  #define IKA_ROS_DEBUG_STREAM_DELAYED_THROTTLE(rate, args) IKA_ROS_LOG_STREAM_DELAYED_THROTTLE(rate, ::ros::console::levels::Debug, ROSCONSOLE_DEFAULT_NAME, args)
  #define IKA_ROS_DEBUG_DELAYED_THROTTLE_NAMED(rate, name, ...) IKA_ROS_LOG_DELAYED_THROTTLE(rate, ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
  #define IKA_ROS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(rate, name, args) IKA_ROS_LOG_STREAM_DELAYED_THROTTLE(rate, ::ros::console::levels::Debug, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)
#endif

#if (ROSCONSOLE_MIN_SEVERITY > ROSCONSOLE_SEVERITY_INFO)
  #define IKA_ROS_INFO_THROTTLE(rate, ...)
  #define IKA_ROS_INFO_STREAM_THROTTLE(rate, args)
  #define IKA_ROS_INFO_THROTTLE_NAMED(rate, name, ...)
  #define IKA_ROS_INFO_STREAM_THROTTLE_NAMED(rate, name, args)
  #define IKA_ROS_INFO_DELAYED_THROTTLE(rate, ...)
  #define IKA_ROS_INFO_STREAM_DELAYED_THROTTLE(rate, args)
  #define IKA_ROS_INFO_DELAYED_THROTTLE_NAMED(rate, name, ...)
  #define IKA_ROS_INFO_STREAM_DELAYED_THROTTLE_NAMED(rate, name, args)
#else
  #define IKA_ROS_INFO_THROTTLE(rate, ...) IKA_ROS_LOG_THROTTLE(rate, ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
  #define IKA_ROS_INFO_STREAM_THROTTLE(rate, args) IKA_ROS_LOG_STREAM_THROTTLE(rate, ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, args)
  #define IKA_ROS_INFO_THROTTLE_NAMED(rate, name, ...) IKA_ROS_LOG_THROTTLE(rate, ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
  #define IKA_ROS_INFO_STREAM_THROTTLE_NAMED(rate, name, args) IKA_ROS_LOG_STREAM_THROTTLE(rate, ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)
  #define IKA_ROS_INFO_DELAYED_THROTTLE(rate, ...) IKA_ROS_LOG_DELAYED_THROTTLE(rate, ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
  #define IKA_ROS_INFO_STREAM_DELAYED_THROTTLE(rate, args) IKA_ROS_LOG_STREAM_DELAYED_THROTTLE(rate, ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, args)
  #define IKA_ROS_INFO_DELAYED_THROTTLE_NAMED(rate, name, ...) IKA_ROS_LOG_DELAYED_THROTTLE(rate, ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
  #define IKA_ROS_INFO_STREAM_DELAYED_THROTTLE_NAMED(rate, name, args) IKA_ROS_LOG_STREAM_DELAYED_THROTTLE(rate, ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)
#endif

#if (ROSCONSOLE_MIN_SEVERITY > ROSCONSOLE_SEVERITY_WARN)
  #define IKA_ROS_WARN_THROTTLE(rate, ...)
  #define IKA_ROS_WARN_STREAM_THROTTLE(rate, args)
  #define IKA_ROS_WARN_THROTTLE_NAMED(rate, name, ...)
  #define IKA_ROS_WARN_STREAM_THROTTLE_NAMED(rate, name, args)
  #define IKA_ROS_WARN_DELAYED_THROTTLE(rate, ...)
  #define IKA_ROS_WARN_STREAM_DELAYED_THROTTLE(rate, args)
  #define IKA_ROS_WARN_DELAYED_THROTTLE_NAMED(rate, name, ...)
  #define IKA_ROS_WARN_STREAM_DELAYED_THROTTLE_NAMED(rate, name, args)
#else
  #define IKA_ROS_WARN_THROTTLE(rate, ...) IKA_ROS_LOG_THROTTLE(rate, ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
  #define IKA_ROS_WARN_STREAM_THROTTLE(rate, args) IKA_ROS_LOG_STREAM_THROTTLE(rate, ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, args)
  #define IKA_ROS_WARN_THROTTLE_NAMED(rate, name, ...) IKA_ROS_LOG_THROTTLE(rate, ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
  #define IKA_ROS_WARN_STREAM_THROTTLE_NAMED(rate, name, args) IKA_ROS_LOG_STREAM_THROTTLE(rate, ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)
  #define IKA_ROS_WARN_DELAYED_THROTTLE(rate, ...) IKA_ROS_LOG_DELAYED_THROTTLE(rate, ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
  #define IKA_ROS_WARN_STREAM_DELAYED_THROTTLE(rate, args) IKA_ROS_LOG_STREAM_DELAYED_THROTTLE(rate, ::ros::console::levels::Warn, ROSCONSOLE_DEFAULT_NAME, args)
  #define IKA_ROS_WARN_DELAYED_THROTTLE_NAMED(rate, name, ...) IKA_ROS_LOG_DELAYED_THROTTLE(rate, ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
  #define IKA_ROS_WARN_STREAM_DELAYED_THROTTLE_NAMED(rate, name, args) IKA_ROS_LOG_STREAM_DELAYED_THROTTLE(rate, ::ros::console::levels::Warn, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)
#endif

#if (ROSCONSOLE_MIN_SEVERITY > ROSCONSOLE_SEVERITY_ERROR)
  #define IKA_ROS_ERROR_THROTTLE(rate, ...)
  #define IKA_ROS_ERROR_STREAM_THROTTLE(rate, args)
  #define IKA_ROS_ERROR_THROTTLE_NAMED(rate, name, ...)
  #define IKA_ROS_ERROR_STREAM_THROTTLE_NAMED(rate, name, args)
  #define IKA_ROS_ERROR_DELAYED_THROTTLE(rate, ...)
  #define IKA_ROS_ERROR_STREAM_DELAYED_THROTTLE(rate, args)
  #define IKA_ROS_ERROR_DELAYED_THROTTLE_NAMED(rate, name, ...)
  #define IKA_ROS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(rate, name, args)
#else
  #define IKA_ROS_ERROR_THROTTLE(rate, ...) IKA_ROS_LOG_THROTTLE(rate, ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
  #define IKA_ROS_ERROR_STREAM_THROTTLE(rate, args) IKA_ROS_LOG_STREAM_THROTTLE(rate, ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, args)
  #define IKA_ROS_ERROR_THROTTLE_NAMED(rate, name, ...) IKA_ROS_LOG_THROTTLE(rate, ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
  #define IKA_ROS_ERROR_STREAM_THROTTLE_NAMED(rate, name, args) IKA_ROS_LOG_STREAM_THROTTLE(rate, ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)
  #define IKA_ROS_ERROR_DELAYED_THROTTLE(rate, ...) IKA_ROS_LOG_DELAYED_THROTTLE(rate, ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
  #define IKA_ROS_ERROR_STREAM_DELAYED_THROTTLE(rate, args) IKA_ROS_LOG_STREAM_DELAYED_THROTTLE(rate, ::ros::console::levels::Error, ROSCONSOLE_DEFAULT_NAME, args)
  #define IKA_ROS_ERROR_DELAYED_THROTTLE_NAMED(rate, name, ...) IKA_ROS_LOG_DELAYED_THROTTLE(rate, ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
  #define IKA_ROS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(rate, name, args) IKA_ROS_LOG_STREAM_DELAYED_THROTTLE(rate, ::ros::console::levels::Error, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)
#endif

#if (ROSCONSOLE_MIN_SEVERITY > ROSCONSOLE_SEVERITY_FATAL)
  #define IKA_ROS_FATAL_THROTTLE(rate, ...)
  #define IKA_ROS_FATAL_STREAM_THROTTLE(rate, args)
  #define IKA_ROS_FATAL_THROTTLE_NAMED(rate, name, ...)
  #define IKA_ROS_FATAL_STREAM_THROTTLE_NAMED(rate, name, args)
  #define IKA_ROS_FATAL_DELAYED_THROTTLE(rate, ...)
  #define IKA_ROS_FATAL_STREAM_DELAYED_THROTTLE(rate, args)
  #define IKA_ROS_FATAL_DELAYED_THROTTLE_NAMED(rate, name, ...)
  #define IKA_ROS_FATAL_STREAM_DELAYED_THROTTLE_NAMED(rate, name, args)
#else
  #define IKA_ROS_FATAL_THROTTLE(rate, ...) IKA_ROS_LOG_THROTTLE(rate, ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
  #define IKA_ROS_FATAL_STREAM_THROTTLE(rate, args) IKA_ROS_LOG_STREAM_THROTTLE(rate, ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, args)
  #define IKA_ROS_FATAL_THROTTLE_NAMED(rate, name, ...) IKA_ROS_LOG_THROTTLE(rate, ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
  #define IKA_ROS_FATAL_STREAM_THROTTLE_NAMED(rate, name, args) IKA_ROS_LOG_STREAM_THROTTLE(rate, ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)
  #define IKA_ROS_FATAL_DELAYED_THROTTLE(rate, ...) IKA_ROS_LOG_DELAYED_THROTTLE(rate, ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
  #define IKA_ROS_FATAL_STREAM_DELAYED_THROTTLE(rate, args) IKA_ROS_LOG_STREAM_DELAYED_THROTTLE(rate, ::ros::console::levels::Fatal, ROSCONSOLE_DEFAULT_NAME, args)
  #define IKA_ROS_FATAL_DELAYED_THROTTLE_NAMED(rate, name, ...) IKA_ROS_LOG_DELAYED_THROTTLE(rate, ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
  #define IKA_ROS_FATAL_STREAM_DELAYED_THROTTLE_NAMED(rate, name, args) IKA_ROS_LOG_STREAM_DELAYED_THROTTLE(rate, ::ros::console::levels::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)
#endif

/**@}*/
