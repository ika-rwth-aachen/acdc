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

/*
 * v2x_planner_interface.cpp
 *
 * ACDC (2022)
 *
 * Authors:
 * Guido Küppers, guido.kueppers@ika.rwth-aachen.de
 * Jean-Pierre Busch, jean-pierre.busch@ika.rwth-aachen.de
 *
 * Description:
 * This file contains the v2x interface to the trajectory planner for section 5.
 *
 */

#include "trajectory_planner.hpp"
#include <Eigen/Geometry>

// Callback function for received SPATEMs.
void PLANNER::callbackSPAT(const definitions::v2x_SPAT& msg)
{
    // Process SPATEM

    // ### START CODE HERE ###
    // Identify number of intersections in message
    int n_intersections = 0; // Task
    // ### END CODE HERE ###

    // Loop all intersections in message
    for(int i = 0; i < n_intersections; i++) {
        definitions::v2x_SPAT_IntersectionState spat_intsctn = msg.spatData_intersections[i];
        // Loop all movement states to get signal groups
        for(int m = 0; m < spat_intsctn.states.size(); m++) {
            //Loop all traffic lights stored in the map data
            for(int k = 0; k<trafficlights.size(); k++)
            {
                if(trafficlights[k].stationID == msg.header_stationID && trafficlights[k].sig_id == spat_intsctn.states[m].signalGroup)
                {
                    trafficlights[k].last_spat = ros::Time::now();

                    // ### START CODE HERE ###
                    // Check if signal state is red or not
                    trafficlights[k].red = true; // Task
                    // ### END CODE HERE ###
                }
            }
        }
    }
}

// Callback function for received MAPEMs.
void PLANNER::callbackMAP(const definitions::v2x_MAP& msg)
{
    // Process MAPEM

    // ### START CODE HERE ###
    // Identify number of intersections in message
    int n_intersections = 0; // Task
    // ### END CODE HERE ###

    // Loop all intersections in message
    for(int i = 0; i < n_intersections; i++) {
        definitions::v2x_MAP_Intersection intsctn = msg.intersections[i];

        // Loop all lanes to get signal groups and traffic light positions
        for(int m = 0; m < intsctn.adjacent_lanes.size(); m++) {

            definitions::v2x_MAP_Lane lane = intsctn.adjacent_lanes[m];
            
            // ### START CODE HERE ###
            // only ingress lanes can consider traffic signals -> skip all egress lanes
            bool is_egress_lane = true; // Task      
            if (is_egress_lane){
                continue;
            }
            // ### END CODE HERE ###

            bool tl_exists = false;
            for (int n = 0; n < trafficlights.size(); n++){
                if(trafficlights[n].stationID == intsctn.id && trafficlights[n].tl_id == lane.laneId){
                    trafficlights[n].ingress_lane = lane.lane_coordinates;
                    trafficlights[n].sig_id = lane.connections[0].signalGroupId;
                    tl_exists = true;
                }
            }
            if (!tl_exists){
                PLANNER::TrafficLight tl;
                tl.red = true;
                tl.stationID = intsctn.id;
                tl.tl_id = lane.laneId;
                tl.sig_id = lane.connections[0].signalGroupId;
                tl.ingress_lane = lane.lane_coordinates;
                trafficlights.push_back(tl);
            }
        }
    }
}

// Function that derives the relevant traffic light in the local vehicle environment.
PLANNER::TrafficLight PLANNER::getRelevantTrafficLight(std::vector<TrafficLight> tls)
{
    if(tls.size()==1) //Only one TrafficLight in List
    {
        return tls[0];
    }
    else if(tls.size()<=0)
    {
        ROS_ERROR_STREAM("No TrafficLight in vector! Properly there was no MAPEM received!");
    }
    else //Search for relevant Target
    {
        double wDist = 1.0;
        double wY = 5.0;
        double wBehind = 5.0;
        std::vector<double> costs;
        costs.resize(tls.size());
        double lowest_cost = INFINITY;
        int id_lowest_cost = 0;
        //Calculate costs for each traffic light which indicates the "probability" of being the relevant target
        for(int i = 0; i<tls.size(); i++)
        {
            costs[i] = 0;
            //1. Euclidean Distance
            double dist = std::sqrt(pow(tls[i].ingress_lane.back().x,2.0)+pow(tls[i].ingress_lane.back().y,2.0));
            costs[i] += wDist * dist;

            //2. y-Deviation
            costs[i] += wY * (std::exp(std::fabs(tls[i].ingress_lane.back().y) * 2.0)-1);

            //3. X-Dist (is object behind us?)
            if(tls[i].ingress_lane.back().x<0.0)
            {
                costs[i] += wBehind * std::fabs(tls[i].ingress_lane.back().x);
            }

            if(costs[i]<lowest_cost)
            {
                lowest_cost = costs[i];
                id_lowest_cost = i;
            }
        }
        return tls[id_lowest_cost];
    }
}

// Function that transforms the traffic-lights given in a global map-frame into local vehicle coordinates
std::vector<PLANNER::TrafficLight> PLANNER::getLocalTrafficLights()
{
    std::vector<TrafficLight> loc_tl;
    loc_tl = trafficlights;
    geometry_msgs::PoseStamped pose_in, pose_out;
    geometry_msgs::Point point_out;
    pose_in.header.frame_id = "map";
    pose_in.pose.orientation.w = 1.0;
    pose_out.pose.orientation.w = 1.0;

    // Loop over vector
    for(int i = 0; i<trafficlights.size(); i++)
    {
        // Check for SPAT Timeout
        if((ros::Time::now()-trafficlights[i].last_spat).toSec()>=10.0)
        {
            ROS_WARN_STREAM("SPAT Timeout for Traffic Light ID: " << trafficlights[i].tl_id);
            trafficlights[i].red = true;
            loc_tl[i].red = true;
        }
        
        // Transform TL Positions into vehicle coordinates
        loc_tl[i].ingress_lane.clear();
        for(int j = 0; j<trafficlights[i].ingress_lane.size(); j++)
        {
            pose_in.pose.position = trafficlights[i].ingress_lane[j];
            transform_buffer_->transform(pose_in, pose_out, "vehicle_body");
            point_out = pose_out.pose.position;
            loc_tl[i].ingress_lane.push_back(point_out);
        }
    }
    return loc_tl;
}