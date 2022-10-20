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

#include <ETSIViz.h>

namespace etsi_visualization {


ETSIViz::ETSIViz() : node_handle_(), private_node_handle_("~") {
  ROS_INFO("ETSIViz starting...");
  
  // load parameters 
  std::string spat_topic;
  std::string map_topic;
  std::string cam_topic;

  if(!private_node_handle_.param<std::string>("SPAT_Topic_Name", spat_topic, "/SPATs")) ROS_WARN("SPAT_Topic_Name not set, defaulting to %s.", spat_topic.c_str());
  if(!private_node_handle_.param<std::string>("MAP_Topic_Name", map_topic, "/MAPs")) ROS_WARN("MAP_Topic_Name not set, defaulting to %s.", spat_topic.c_str());
  if(!private_node_handle_.param<std::string>("CAM_Topic_Name", cam_topic, "/CAMs")) ROS_WARN("CAM_Topic_Name not set, defaulting to %s.", cam_topic.c_str());

  if(!private_node_handle_.param<std::string>("map_frame_id", map_frame_id_, "map")) ROS_WARN("map_frame_id not set, defaulting to %s.", map_frame_id_.c_str());

  // setup publisher and subscriber
  pub_marker_array_ = private_node_handle_.advertise<visualization_msgs::MarkerArray>("/v2x_receive/etsi_viz", 1);
  sub_SPAT_ = private_node_handle_.subscribe(spat_topic, 0, &ETSIViz::SPATCallback, this);
  sub_MAP_ = private_node_handle_.subscribe(map_topic, 0, &ETSIViz::MAPCallback, this);
  sub_CAM_ = private_node_handle_.subscribe(cam_topic, 0, &ETSIViz::CAMCallback, this);
  pub_CAM_objList_ = private_node_handle_.advertise<definitions::IkaObjectList>("/etsi_viz/CAM_objectList", 1);

  // create timer for publishing messages
  timer_ = node_handle_.createTimer(ros::Duration(0.1), &ETSIViz::markerCallback, this);

  ros::spin();
}

void ETSIViz::CAMCallback(const definitions::v2x_CAM& msg){
  if (cam_objList_.objects.empty()){
    cam_objList_.header.frame_id = "map";
    cam_objList_.IdSource = 91; //CAM
  }

  if (!private_node_handle_.getParam("/cam_generator/stationID", ego_stationID_)) ROS_WARN("No ego stationID set!");

  if (ego_stationID_ > 0 && ego_stationID_ == msg.header_stationID) return; // do not visualize ego cam

  bool stationID_exists = false;
  cam_objList_.header.stamp = ros::Time::now();
  for (auto &element : cam_objList_.objects){
    if (element.IdInternal == msg.header_stationID){
      stationID_exists = true;
      IkaUtilities::setObjectPositionXY(element, msg.basic_container.referencePosition_longitude, msg.basic_container.referencePosition_latitude);
      float v_x = msg.high_freq_container.speed_speedValue * std::cos(msg.high_freq_container.heading_headingValue);
      float v_y = msg.high_freq_container.speed_speedValue * std::sin(msg.high_freq_container.heading_headingValue);
      IkaUtilities::setObjectVelocity(element, v_x, v_y);
      IkaUtilities::setObjectHeading(element, msg.high_freq_container.heading_headingValue);
      break;
    }
  }

  if (stationID_exists == false){
    definitions::IkaObject obj;
    obj.IdMotionModel = definitions::motion_model::CTRA;
    obj.IdType = definitions::ika_object_types::CAR;
    obj.bObjectValid = true;
    obj.fExistenceProbability = 1.0;
    
      // ### START CODE HERE
      // fill with information from message
      // use helping comments from Wiki
      obj.IdInternal = 0; // stationID               // Task
      float lon = 0;      // longitude (x)           // Task
      float lat = 0;      // latitude (y)            // Task
      float v_x = 0;      // velocity in x direction // Task
      float v_y = 0;      // velocity in y direction // Task
      // ### END CODE HERE

    obj.fMean.resize((int)definitions::ctra_model::COUNT);
    obj.fCovariance.resize(pow((int)definitions::ctra_model::COUNT,2));
    IkaUtilities::setObjectPositionXY(obj, lon, lat); // x,y
    IkaUtilities::setObjectVelocity(obj, v_x, v_y);
    IkaUtilities::setObjectHeading(obj, msg.high_freq_container.heading_headingValue);         // heading
    obj.fMean[(int)definitions::ctra_model::length] = msg.high_freq_container.vehicleLength_vehicleLengthValue;   // length
    obj.fMean[(int)definitions::ctra_model::width] = msg.high_freq_container.vehicleWidth;     // width
    obj.fMean[(int)definitions::ctra_model::height] = 1.5;  // dummy
    cam_objList_.objects.push_back(obj);
  }

  pub_CAM_objList_.publish(cam_objList_);
}

void ETSIViz::markerCallback(const ros::TimerEvent& event) {

  visualization_msgs::MarkerArray markers;
  
  // Erase when ROS performs a jump back in time
  if((ros::Time::now() - last_time_stamp_).toSec() < -0.1)
  {
    ROS_WARN("etsi_visualization: Detected jump back in time. Clearing visualization.");
    converted_isctns_.clear();
    spats_.clear();
    last_time_stamp_map_ = ros::Time(0);
  }

  // MAPEM
  const bool update_map_vis = map_vis_update_freq_ > 0.0 ? ((ros::Time::now() - last_time_stamp_map_) >= ros::Duration(1.0/map_vis_update_freq_)) : false;
  size_t ids_mapem_ref = 0;
  size_t ids_mapem_lanes = 0;
  size_t ids_spat_status = 0;
  size_t ids_spat_predictions = 0;
  for(size_t i=0; i<converted_isctns_.size(); i++) {
    if(update_map_vis)
    {
      markers.markers.push_back(MAP2RefPoint(converted_isctns_[i], ids_mapem_ref++));
      markers.markers.push_back(MAP2RefText(converted_isctns_[i], ids_mapem_ref++));
      last_time_stamp_map_ = ros::Time::now();
    }

    const int32_t is_id = converted_isctns_[i].id;
    bool spat_found = false;
    SPATViz spatviz;

    // Get relevant SPAT for Intersection
    for(int sp = 0; sp < spats_.size(); sp++) {

      if(spats_[sp].intersection_id == is_id) {
        spat_found = true;
        spatviz = spats_[sp];
        break;
      }

    }

    // Loop individual lanes
    for(int l = 0; l < converted_isctns_[i].adjacent_lanes.size(); l++) {
      
      definitions::v2x_MAP_Lane lane = converted_isctns_[i].adjacent_lanes[l];
      
      if(update_map_vis)
      {
        markers.markers.push_back(MAPLane2LS(lane, converted_isctns_[i].header.frame_id, ids_mapem_lanes++));
        markers.markers.push_back(MAPLane2Text(lane, converted_isctns_[i].header.frame_id, ids_mapem_lanes++));
      }

      // Check if spat available for this intersection
      if(spat_found) {

        // Get first Point of lane for visualizing Signal state
        geometry_msgs::Point pt = lane.lane_coordinates.back();

        // Loop all connections to visualize signal groups
        for(int c = 0; c < lane.connections.size(); c++) {

          if(lane.connections[c].signalGroupId_present) {

            uint8_t sg_id = (uint8_t) lane.connections[c].signalGroupId;

            // Get state, next_change and confidence for signal group
            uint8_t state = 255; // Max
            uint16_t next_change = 65.535; // Max

            for(int es = 0; es < spatviz.sgs.size(); es++) {

              if(sg_id == spatviz.sgs[es].sg_id) {
                state = spatviz.sgs[es].current_state;
                next_change = spatviz.sgs[es].next_change;
                break;
              }

            }

            markers.markers.push_back(SG2Sphere(pt, converted_isctns_[i].header.frame_id, state, ids_spat_status++));
            if(state == 6 || state == 3 || state == 7) {
              markers.markers.push_back(SPATPred(pt, converted_isctns_[i].header.frame_id, next_change, ids_spat_predictions++));
            }
          }
        }
      }
    }
  }

  // Clear old markers
  if(update_map_vis)
  {
    clearOldMarkers(ids_mapem_ref,      ids_mapem_ref_,        markers, "MAPEM_Ref");
    clearOldMarkers(ids_mapem_lanes,    ids_mapem_lanes_,      markers, "MAPEM_Lanes");
  }
  clearOldMarkers(ids_spat_status,      ids_spat_status_,      markers, "SPAT_Status");
  clearOldMarkers(ids_spat_predictions, ids_spat_predictions_, markers, "SPAT_Predictions");

  pub_marker_array_.publish(markers);

  last_time_stamp_ = ros::Time::now();
}

void ETSIViz::clearOldMarkers(const size_t& ids_used, int& prev_ids_used, visualization_msgs::MarkerArray& markers, const std::string& ns) const
{
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::DELETE;
  marker.ns = ns;
  const int last_id_used = ids_used > 0 ? ids_used-1 : -1;
  for(int i=last_id_used+1; i<=prev_ids_used; i++)
  {
    marker.id = i;
    markers.markers.push_back(marker);
  }
  prev_ids_used = last_id_used;
}

}  // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ETSIViz");

  etsi_visualization::ETSIViz node;

  return 0;
}

