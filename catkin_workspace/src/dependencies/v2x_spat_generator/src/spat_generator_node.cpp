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

#include <spat_generator.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "spat_generator_node");

  spat_generator node;
  node.init(argc, argv);
}

// Constructor
spat_generator::spat_generator()
{

}
// Destructor
spat_generator::~spat_generator()
{

}

// Fill Spat Function
// ==================================================================
definitions::v2x_SPAT spat_generator::fillSpat()
{
	// Init Spat Message and submessages
	definitions::v2x_SPAT spatOut;
	definitions::v2x_SPAT_IntersectionState intersection;
	definitions::v2x_SPAT_MovementState mvmtStates[nSignalGroups];
	definitions::ASN_bitstring asnBitstring;

	// Time calculations -----------------------------------------
	int soy = ros::WallTime::now().toSec() - epoch_01_2022_;		// second of year
	int moy = soy/60;											// minute of year
	int moh = moy%60;											// minute of hour
	int sec = (soy)%60;											// second of minute

	if ((ros::WallTime::now() - time_to_change_).toSec() > 0){
		start_time_ = ros::WallTime::now();
		if (current_states[0] == 3){
			//current_state_ego_ = 6;
			//current_state_obj_ = 3;
			std::swap(current_states[0], current_states[1]);
			time_to_change_ = start_time_ + time_to_red_;
		}
		else if (current_states[0] == 6){
			//current_state_ego_ = 3;
			//current_state_obj_ = 6;
			std::swap(current_states[0], current_states[1]);
			time_to_change_ = start_time_ + time_to_green_;
		}
	}

	int change_soy = time_to_change_.toSec() - epoch_01_2022_;	// second of year
	int change_moy = change_soy/60;								// minute of year
	int change_moh = change_moy%60;								// minute of hour
	int change_sec = (change_soy)%60;							// second of minute

	// Fill message -------------------------------------------------

	// ASN Bitstring
	asnBitstring.buf.push_back(1);
	asnBitstring.bits_unused = 0;

	// Event
	for (int i = 0; i < nSignalGroups; i++) {
		mvmtStates[i].signalGroup = i+1;
		mvmtStates[i].movementName = "";
		mvmtStates[i].movementName_present = false;
		
		definitions::v2x_SPAT_MovementEvent mvmtEvent;
		mvmtEvent.eventState = current_states[i];
		mvmtEvent.timing_startTime = 0;
		mvmtEvent.timing_startTime_present = false;
		mvmtEvent.timing_minEndTime = (change_moh*60+change_sec)*10-1;
		mvmtEvent.timing_likelyTime = (change_moh*60+change_sec)*10;
		mvmtEvent.timing_likelyTime_present = true;
		mvmtEvent.timing_maxEndTime = (change_moh*60+change_sec)*10+1;
		mvmtEvent.timing_maxEndTime_present = true;
		mvmtEvent.timing_present = true;

		mvmtStates[i].state_time_speed.push_back(mvmtEvent);

		intersection.states.push_back(mvmtStates[i]);
	}

	// Intersection
	if (intersection_name != ""){
		intersection.name = intersection_name;
		intersection.name_present = true;
	}
	else{
		intersection.name_present = false;
	}
	if (id_region != -1){
		intersection.id_region = id_region;
		intersection.id_region_present = true;
	}
	else{
		intersection.id_region_present = false;
	}
	intersection.id_id = id_id;
	intersection.revision = 1;
	intersection.moy = moy;
	intersection.moy_present = true;
	intersection.timeStamp = sec;
	intersection.timeStamp_present = true;
	intersection.status = asnBitstring;
	intersection.enabledLanes_present = false;
	intersection.maneuverAssistList_present = false;
	intersection.priority_present = false;
	intersection.preempt_present = false;
	intersection.regional_present = false;

	// Main Spat
	spatOut.header_protocolVersion = 2;
	spatOut.header_messageID = 4; // SPAT
	spatOut.header_stationID = stationID;
	spatOut.spatData_msgID = 0;
    spatOut.spatData_msgSubID = 0;
    spatOut.spatData_msgSubID_present = false;
    if (spat_name != ""){
		spatOut.spatData_name = spat_name;
    	spatOut.spatData_name_present = true;
	}
	else{
		spatOut.spatData_name_present = false;
	}
    spatOut.spatData_regional_present = false;

	spatOut.spatData_intersections.push_back(intersection);

	return spatOut;

}

// Main function
// ==================================================================
int spat_generator::init(int argc, char **argv)
{

    // Enable Parameter Support in Nodehandle
    ros::NodeHandle n("~");

	// Get General Parameters
	n.param<double>("config/frequency", frequency, 1.0);
	n.param<std::string>("config/topic_out", topic_out, "/SPATEMs");
	n.param<double>("config/time_to_green", ttg, 45.0); 
	n.param<double>("config/time_to_red", ttr, 15.0); 

	ROS_INFO("Publish on topic: %s with %f Hz", topic_out.c_str(), frequency);
	ROS_INFO("Time to green: %f s", ttg);
	ROS_INFO("Time to red: %f s", ttr);

	// Get custom SPATEM
	n.param<std::string>("spat_data/name", spat_name, "");
	n.param<int>("spat_data/stationID", stationID, -1);
	n.param<std::string>("spat_data/intersection/name", intersection_name, "");
	n.param<int>("spat_data/intersection/id_region", id_region, -1);
	n.param<int>("spat_data/intersection/id_id", id_id, -1);
	n.param<int>("spat_data/intersection/nSignalGroups", nSignalGroups, 2);

	// Init important variables and constants
	time_to_green_ = ros::WallDuration(ttg);
  	time_to_red_ = ros::WallDuration(ttr);
	start_time_ = ros::WallTime::now();
	current_states[0] = 3;			// start with red for ego
	current_states[1] = 6;			// start with green for obj
	time_to_change_ = start_time_ + time_to_green_;

	// Init loopRate
	ros::Rate loopRate(frequency);

	// Init publisher
	pub_spat_ = n.advertise<definitions::v2x_SPAT>(topic_out, 0);

    // Start main loop
    while(ros::ok())
    {

    	// Fill spat message
    	definitions::v2x_SPAT spatMessage = fillSpat();

		// Publish spat message
		pub_spat_.publish(spatMessage);

    	// Sleep
    	loopRate.sleep();

    }

    // End process
    return 0;

}