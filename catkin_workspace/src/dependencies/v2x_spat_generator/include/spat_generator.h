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

#include <string>
#include <ros/ros.h>

#include <definitions/v2x_SPAT.h>
#include <definitions/v2x_SPAT_IntersectionState.h>
#include <definitions/v2x_SPAT_MovementState.h>
#include <definitions/v2x_SPAT_MovementEvent.h>
#include <definitions/ASN_bitstring.h>

class spat_generator{
protected:

  ros::Publisher pub_spat_;

  ros::WallDuration time_to_green_;
  ros::WallDuration time_to_red_;

  int epoch_01_2022_ = 1640995200;

  // Params
  double frequency;
  std::string topic_out;
  double ttg;
  double ttr;

  // Custom SPATEM
  std::string spat_name;
  int stationID;
  std::string intersection_name;
  int id_region;
  int id_id;
  int nSignalGroups;
  
  // Important variables
  ros::WallTime start_time_;
  ros::WallTime time_to_change_;
  int current_states[2];

public:
  spat_generator();

  ~spat_generator();

  int init(int argc, char **argv);

  definitions::v2x_SPAT fillSpat();

};