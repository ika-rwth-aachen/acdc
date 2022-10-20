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

void ETSIViz::MAPCallback(const definitions::v2x_MAP& msg) {
    
    // ### START CODE HERE ###
    // Identify number of intersections in message
    int n_intersections = 0; // Task
    // ### END CODE HERE ###

    // Loop over all intersections in message
    for(int i = 0; i < n_intersections; i++) {
        bool b_exists = false;

        definitions::v2x_MAP_Intersection intsctn = msg.intersections[i];

        // Get ID of intersection and check, if already available in list
        const int intsctn_id = intsctn.id;

        for(int j = 0; j < converted_isctns_.size(); j++) {

            // Update intersection if it is already in list
            if(intsctn_id == converted_isctns_[j].id) {

                converted_isctns_[j] = intsctn;
                ROS_DEBUG("Intersection %d updated, %ld intersections in list.", intsctn_id, converted_isctns_.size());
                b_exists = true;
            }
        }

        // Add converted intersection if not available so far
        if(!b_exists) {
            converted_isctns_.push_back(intsctn);
            ROS_DEBUG("Intersection %d updated, %ld intersections in list.", intsctn_id, converted_isctns_.size());
        }
    }
}

visualization_msgs::Marker ETSIViz::MAP2RefPoint(const definitions::v2x_MAP_Intersection& isctn, const size_t& id) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = isctn.header.frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns="MAPEM_Ref";
    marker.id = id;
    marker.action = 0;
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = (219.0/255.0);
    marker.color.g = (215.0/255.0);
    marker.color.b = (119.0/255.0);
    marker.color.a = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    return marker;
}

visualization_msgs::Marker ETSIViz::MAP2RefText(const definitions::v2x_MAP_Intersection& isctn, const size_t& id) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_id_;
    marker.header.stamp = ros::Time::now();

    marker.ns="MAPEM_Ref";
    marker.id = id;
    marker.action = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.pose.position.x = isctn.refPoint_x-15;
    marker.pose.position.y = isctn.refPoint_y+10;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.text = "Message Type: MAPEM\n";
    marker.text += "Intersection ID: " + std::to_string(isctn.id);
    marker.color.r = (219.0/255.0);
    marker.color.g = (215.0/255.0);
    marker.color.b = (119.0/255.0);
    marker.color.a = 1.0;
    marker.scale.z = 2.0;

    return marker;
}

visualization_msgs::Marker ETSIViz::MAPLane2LS(const definitions::v2x_MAP_Lane& lane, const std::string& frame_id, const size_t& id) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns="MAPEM_Lanes";
    marker.id = id;
    marker.action = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.points = lane.lane_coordinates;

    // Change color depending on driving direction
    if (lane.directionalUse == 0) { // Ingress
        marker.color.r = (131.0/255.0);
        marker.color.g = (226.0/255.0);
        marker.color.b = (242.0/255.0);
    } else if (lane.directionalUse == 1) { // Egress
        marker.color.r = (131.0/255.0);
        marker.color.g = (242.0/255.0);
        marker.color.b = (195.0/255.0);
    } else { // Unknown
        marker.color.r = (130.0/255.0);
        marker.color.g = (219.0/255.0);
        marker.color.b = (207.0/255.0);
    }

    // Change opacity depending on laneType
    if (lane.laneType == 0) { // Vehicle as "relevant" lanes
        marker.color.a = 1.0;
    } else {
        marker.color.a = 0.5;
    }
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    return marker;
}

visualization_msgs::Marker ETSIViz::MAPLane2Text(const definitions::v2x_MAP_Lane& lane, const std::string& frame_id, const size_t& id) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns="MAPEM_Lanes";
    marker.id = id;
    marker.action = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.pose.position.x = lane.lane_coordinates.back().x;
    marker.pose.position.y = lane.lane_coordinates.back().y-1;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.text = "Lane " + std::to_string(lane.laneId);
    marker.color.r = (130.0/255.0);
    marker.color.g = (219.0/255.0);
    marker.color.b = (207.0/255.0);
    marker.color.a = 1.0;
    marker.scale.z = 1.5;

    return marker;
}

}  // end of namespace
