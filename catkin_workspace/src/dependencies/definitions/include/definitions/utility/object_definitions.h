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

namespace definitions {

/**
 * @brief meanings of state space dimensions of ikaObject.state
 * all units in SI basis units
 */
enum motion_model {
  UNKNOWN = 0,
  CV = 1,
  CA = 2,
  CTRV = 3,
  CTRA = 4,
  MOTION_MODEL_COUNT = 5
};

enum class ca_model {
  posX = 0,
  posY = 1,
  posZ = 2,
  absVelX = 3,
  absVelY = 4,
  absAccX = 5,
  absAccY = 6,
  length = 7,
  width = 8,
  height = 9,
  heading = 10,
  COUNT = 11
};

enum class ca_model_variance {
  posX = 0,
  posY = 12,
  posZ = 24,
  absVelX = 36,
  absVelY = 48,
  absAccX = 60,
  absAccY = 72,
  length = 84,
  width = 96,
  height = 108,
  heading = 120
};

enum class ctra_model {
  posX = 0,
  posY = 1,
  posZ = 2,
  absVel = 3,
  absAcc = 4,
  heading = 5,
  yawrate = 6,
  length = 7,
  width = 8,
  height = 9,
  COUNT = 10
};

enum class ctra_model_variance {
  posX = 0,
  posY = 11,
  posZ = 22,
  absVel = 33,
  absAcc = 44,
  heading = 55,
  yawrate = 66,
  length = 77,
  width = 88,
  height = 99
};

/**
 * @brief meanings of additional relative state variables in ikaObject.relativeState
 * all units in SI basis units
 */
enum ika_object_rel_states {
  relVelX = 0,
  relVelY = 1,
  relAccX = 2,
  relAccY = 3
};

/**
 * @brief meanings of types in ikaObject.IdType
 */
enum ika_object_types {
  UNCLASSIFIED = 0,
  PEDESTRIAN = 1,
  BICYCLE = 2,
  MOTORBIKE = 3,
  CAR = 4,
  TRUCK = 5,
  VAN = 6,
  BUS = 7,
  ANIMAL = 8,
  ROAD_OBSTACLE = 9,
  TRAILER = 10,
  TYPES_COUNT = 11
};

/**
 * @brief behavior that may be useful for prediction
 */
enum ika_object_behaviours {
  BEHAVIOUR_NO_INFO=0,
  BEHAVIOUR_LANE_BOUND=1,
  BEHAVIOUR_LANE_UNBOUND=2,

  NUMBER_BEHAVIOURS = 3
};

/**
 * @brief source sensors in ikaObject.IdExternal and ikaObjectList.IdSource (use >=0 & <=99)
 */
enum input_sensors {
  SOURCE_NO_SOURCE = 0,
  ESR_FRONT = 1,
  ESR_REAR = 2,
  EPM = 3,
  LUX = 4,
  SRR_FRONT = 5,
  RT = 6,
  SRR_REAR = 7,
  SRR_LEFT = 8,
  SRR_RIGHT = 9,
  SCALA = 11,
  VLP = 12,
  RTRANGE = 13,
  RADAR = 14,
  LIDAR_RS = 15,
  CAMERA = 16,
  CAM = 91,
  STATION_A = 98,
  STATION_B = 99,
  FIRST_SOURCE = SOURCE_NO_SOURCE,
  LAST_SOURCE = STATION_B
};

/**
 * @brief output sensors (i.e. of sensor fusion) in ikaObject.IdExternal and ikaObjectList.IdSource (use >=0 & <=99)
 */
//output (use >=100 & <=199)
enum env_model_outputs {
  FUSED_OBJECTS=100,
  RT_ASSOCIATIONS=105,
  NO_SOURCE=199
};


/**
 * @brief reference point locations (i.e. used for ibeo scala)
 */
enum class ika_ref_point {
    CENTER_OF_GRAVITY = 0,
    FRONT_LEFT = 1,
    FRONT_RIGHT = 2,
    REAR_RIGHT = 3,
    REAR_LEFT = 4,
    FRONT_CENTER = 5,
    RIGHT_CENTER = 6,
    REAR_CENTER = 7,
    LEFT_CENTER = 8,
    OBJECT_CENTER = 9,
    UNKNOWN = 255
};


}

/**@}*/
