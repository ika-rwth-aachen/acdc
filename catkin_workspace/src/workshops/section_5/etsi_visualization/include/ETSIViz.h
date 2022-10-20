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

#include <string>
#include <map>

#include <ros/ros.h>
#include <definitions/utility/ika_utilities.h>
#include <definitions/IkaObject.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

//V2x
#include <definitions/v2x_SPAT.h>
#include <definitions/v2x_MAP.h>
#include <definitions/v2x_CAM.h>

namespace etsi_visualization {

// Struct for signal group as part of intersection
typedef struct SignalGroup {
  uint8_t sg_id;
  uint8_t current_state;
  uint16_t next_change;
} SignalGroup_t;

class SPATViz
{
  public:
    SPATViz(){}
    int32_t intersection_id;
    uint32_t moy; // Minute of the year
    uint32_t dsec; // Millisec of moy
    std::vector<SignalGroup_t> sgs;
};

class ETSIViz {

  public:
    ETSIViz();

  private:
    void markerCallback(const ros::TimerEvent& event);
    void SPATCallback(const definitions::v2x_SPAT& msg);
    void MAPCallback(const definitions::v2x_MAP& msg);
    void CAMCallback(const definitions::v2x_CAM& msg);
    void clearOldMarkers(const size_t &ids_used, int& prev_ids_used, visualization_msgs::MarkerArray& markers, const std::string& ns) const;

    visualization_msgs::Marker SG2Sphere(const geometry_msgs::Point pt, const std::string& frame_id, const uint8_t& state, const size_t& id);
    visualization_msgs::Marker SPATPred(const geometry_msgs::Point& pt, const std::string& frame_id,  const uint16_t& next_change, const size_t& id) const;

    visualization_msgs::Marker MAP2RefPoint(const definitions::v2x_MAP_Intersection& isctn, const size_t& id) const;
    visualization_msgs::Marker MAP2RefText(const definitions::v2x_MAP_Intersection& isctn, const size_t& id) const;
    visualization_msgs::Marker MAPLane2LS(const definitions::v2x_MAP_Lane& lane, const std::string& frame_id, const size_t& id) const;
    visualization_msgs::Marker MAPLane2Text(const definitions::v2x_MAP_Lane& lane, const std::string& frame_id, const size_t& id) const;

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;

    ros::Publisher pub_marker_array_;
    ros::Subscriber sub_SPAT_;
    ros::Subscriber sub_MAP_;
    ros::Subscriber sub_CAM_;
    ros::Publisher pub_CAM_objList_;

    ros::Timer timer_;

    std::vector<SPATViz> spats_;
    std::vector<definitions::v2x_MAP_Intersection> converted_isctns_;

    std::string map_frame_id_;
    double map_vis_update_freq_ = 0.1;
    ros::Time last_time_stamp_map_ = ros::Time(0);
    ros::Time last_time_stamp_ = ros::Time(0);

    int ids_mapem_ref_ = 0;
    int ids_mapem_lanes_ = 0;
    int ids_spat_status_ = 0;
    int ids_spat_predictions_ = 0;
    int ego_stationID_ = 0;
    definitions::IkaObjectList cam_objList_;

    int epoch_01_2022_ = 1640995200;

};


}  // end of namespace etsi_visualization
