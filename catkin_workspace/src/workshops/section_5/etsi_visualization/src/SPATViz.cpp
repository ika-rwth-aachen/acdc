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


void ETSIViz::SPATCallback(const definitions::v2x_SPAT& msg) {

    // Loop all intersections in message
    for(int i = 0; i < msg.spatData_intersections.size(); i++) {

        bool is_exists = false;

        // Create SPATViz Object
        SPATViz spatviz;

        definitions::v2x_SPAT_IntersectionState spat_intsctn = msg.spatData_intersections[i];

        // Get ts of incoming msg
        uint32_t in_moy = 0xffffffff; // Init with max value
        uint32_t in_dsec = 0xffffffff;

        if(spat_intsctn.moy_present) {
            in_moy = spat_intsctn.moy;
        }

        if(spat_intsctn.timeStamp_present) {
            in_dsec = spat_intsctn.timeStamp;
        }

        // Loop all movement states to get signal groups
        for(int m = 0; m < spat_intsctn.states.size(); m++) {

            // Create Signal Group
            SignalGroup_t sg;

            // ### START CODE HERE ###
            sg.current_state = spat_intsctn.states[m].state_time_speed[0].eventState; // Get first element of state_time_speed
            sg.sg_id = 0; // Task
            sg.next_change = 0; // Task
            // ### END CODE HERE ###

            spatviz.sgs.push_back(sg);

        }

        // Get ID of intersection and check, if already available in list
        spatviz.intersection_id = (int32_t) spat_intsctn.id_id;
        spatviz.moy = in_moy;
        spatviz.dsec = in_dsec;

        for(int j = 0; j < spats_.size(); j++) {

            // Update intersection if it is already in list
            if(spat_intsctn.id_id == spats_[j].intersection_id) {

                // Check if timestamp is higher (newer)
                if(in_moy > spats_[j].moy || in_dsec >= spats_[j].dsec) {

                    spats_[j] = spatviz;
                    ROS_DEBUG("SPAT Intersection %d updated, %ld intersections in list.", spat_intsctn.id_id, spats_.size());
                    is_exists = true;

                } else {

                    ROS_DEBUG("SPAT Intersection %d not updated, message is older than existing message.", spat_intsctn.id_id);
                    is_exists = true;
                }



            }

        }

        if(!is_exists) {
            // Add converted intersection if not available so far
            spats_.push_back(spatviz);
            ROS_DEBUG("SPAT Intersection %d updated, %ld intersections in list.", spat_intsctn.id_id, spats_.size());
        }

    }

}

visualization_msgs::Marker ETSIViz::SG2Sphere(const geometry_msgs::Point pt, const std::string& frame_id, const uint8_t& state, const size_t& id) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();    

    marker.ns="SPAT_Status";
    marker.id = id;
    marker.action = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = pt.x;
    marker.pose.position.y = pt.y;
    marker.pose.position.z = pt.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.color.a = 1.0;
    marker.scale.x = 1.2;
    marker.scale.y = 1.2;
    marker.scale.z = 1.2;
    
    // ### START CODE HERE ###
    // Set marker color depending on signal group state
    // The signal group state is given by the function variable "state"
    // hint: you can use an if (or switch) condition
    marker.color.r = 0.0; // Task
    marker.color.g = 0.0; // Task
    marker.color.b = 0.0; // Task
    // ### END CODE HERE ###


    return marker;

}

visualization_msgs::Marker ETSIViz::SPATPred(const geometry_msgs::Point& pt, const std::string& frame_id,  const uint16_t& next_change, const size_t& id) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();

    marker.ns="SPAT_Predictions";
    marker.id = id;
    marker.action = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    //--------------------------------------------------------------------------
    std::string change_str = "";

    //Current ROS-Time
    double ros_soy = ros::WallTime::now().toSec() - epoch_01_2022_; // second of year
    int ros_moy = ros_soy/60; // minute of year
    int ros_moh = ros_moy%60; // minute of hour
    int ros_sec = (int) (ros_soy)%60; // second of minute


    //next Change Time
    int nxt_moh = next_change/600;
    int nxt_sec = (next_change/10)%60;
    int change = (nxt_moh - ros_moh)*60 + (nxt_sec - ros_sec);
    if (ros_moh > 57 && nxt_moh < 3){
        change = change + 3600;
    }
    change_str = std::to_string(change);


    marker.pose.position.x = pt.x;
    marker.pose.position.y = pt.y+2;
    marker.pose.position.z = pt.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.text = "sc: " + change_str;
    marker.color.r = (204.0/255.0);
    marker.color.g = (0.0/255.0);
    marker.color.b = (102.0/255.0);
    if (change < 0){
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
    }
    marker.color.a = 1.0;
    marker.scale.z = 1.5;

    return marker;
}


}  // end of namespace