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
 * @file TrajectoryCtrl.cpp
 * @author Guido Küppers
 * @brief  ROS-Node for trajectory control.
 * @details The trajector_ctrl_node receives an ika-Trajectory and generates the specific vehicle controls!
 */

#include "TrajectoryCtrl.h"

//Main of Trajectory Control Node
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trajectory_control_node");
    ros::NodeHandle node_handle("~");

    TrajectoryControl controller(node_handle); //Create Trajectory Control Object

    ros::spin();

    return 0;
}

//Constructor of Trajectory Control Object
TrajectoryControl::TrajectoryControl(const ros::NodeHandle &node_handle)
{
    node_handle_ = node_handle;

    this->vehicle_state_sub_ = new ros::Subscriber();
    std::string default_subscribe_topic_vehicle = "/vehicle/wheel_sensor";
    std::string subscribe_topic_vehicle;
    node_handle_.param<std::string>("controller/wheel_topic", subscribe_topic_vehicle, default_subscribe_topic_vehicle);
    *this->vehicle_state_sub_ = node_handle_.subscribe(subscribe_topic_vehicle, 1, &TrajectoryControl::VehicleStateCallback, this);

    this->trajectory_sub_ = new ros::Subscriber();
    std::string default_subscribe_topic_traj = "/mpc/trajectory_interface";
    std::string subscribe_topic_traj;
    node_handle_.param<std::string>("/mpc/traj_topic", subscribe_topic_traj, default_subscribe_topic_traj);
    *this->trajectory_sub_ = node_handle_.subscribe(subscribe_topic_traj, 1, &TrajectoryControl::TrajectoryCallback, this);

    this->vehicle_ctrl_pub_ = new ros::Publisher();
    *this->vehicle_ctrl_pub_ = node_handle_.advertise<definitions::FlatlandVehicleState>("/vehicle/actuator_commands", 1);

    //Initialization of the cyclic vehicle-control timer; the callback VehicleCtrlCycle will be called every 0.01s e.g. with 100Hz.
    this->vhcl_ctrl_timer_ = new ros::Timer;
    *this->vhcl_ctrl_timer_ = node_handle_.createTimer(ros::Duration(0.01), &TrajectoryControl::VehicleCtrlCycle, this);

    //Initialize dv-PID
    this->dv_pid_ = new PID(0.0, 0.0, 0.0);

    //Initialize dy-PID
    this->dy_pid_ = new PID(0.0, 0.0, 0.0);

    //Initialize dpsi-PID
    this->dpsi_pid_ = new PID(0.0, 0.0, 0.0);

    // Dynamic reconfigure
    dynamic_reconfigure::Server<trajectory_ctrl::trj_ctrlConfig>::CallbackType f;
    f = boost::bind(&TrajectoryControl::callbackDynReconf, this, _1, _2);
    srv_.setCallback(f);
    this->callbackDynReconf(cfg_, 0);

    // Initialize output variable 
    vhcl_ctrl_output_.velocity = 0.0;
    vhcl_ctrl_output_.yaw_rate = 0.0;
    vhcl_ctrl_output_.acceleration = 0.0;
    vhcl_ctrl_output_.steering_angle = 0.0;

    this->ResetOdometry();
}

// Destructor
TrajectoryControl::~TrajectoryControl()
{
    delete vehicle_state_sub_;
    delete trajectory_sub_;
    delete vehicle_ctrl_pub_;
    delete vhcl_ctrl_timer_;
    delete dv_pid_;
    delete dy_pid_;
    delete dpsi_pid_;
}

// Called when parameters are updated.
void TrajectoryControl::callbackDynReconf(trajectory_ctrl::trj_ctrlConfig &config, uint32_t level)
{
    cfg_ = config;

    ROS_INFO("Parameters received.");
    this->dv_pid_->SetParameters(cfg_.dv_P, cfg_.dv_I, cfg_.dv_D);
    this->dy_pid_->SetParameters(cfg_.dy_P, cfg_.dy_I, cfg_.dy_D);
    this->dpsi_pid_->SetParameters(cfg_.dpsi_P, cfg_.dpsi_I, cfg_.dpsi_D);
}

// Update the actual vehicle state
void TrajectoryControl::VehicleStateCallback(const definitions::FlatlandVehicleState::ConstPtr &msg)
{
    cur_vehicle_state_ = *msg;
    cur_vehicle_state_.yaw_rate *= -1.0;
    cur_vehicle_state_.steering_angle *= -1.0;
}

// Update the current trajectory
void TrajectoryControl::TrajectoryCallback(const definitions::IkaTpTrajectoryInterface::ConstPtr &msg)
{
    cur_trajectory_ = *msg;
    this->ResetOdometry();
}

// Perform the vehicle control cycle
void TrajectoryControl::VehicleCtrlCycle(const ros::TimerEvent &event)
{
    if (!this->InputSanityCheck()) // some inputs are not ok
    {
        //don't do anything
        return;
    }
    this->TrjDataProc();
    vhcl_ctrl_output_.steering_angle = this->LateralControl();
    vhcl_ctrl_output_.acceleration = this->LongitudinalControl();

    vhcl_ctrl_output_.header.stamp = ros::Time::now();
    vehicle_ctrl_pub_->publish(vhcl_ctrl_output_);
}

// Checks all input messages for actuality
bool TrajectoryControl::InputSanityCheck()
{
    double age;
    age = ros::Time::now().toSec() - cur_vehicle_state_.header.stamp.toSec();
    if (age > 0.1 || age < 0.0) // current vehicle state data older than 0.1s
    {
        return false;
    }
    age = ros::Time::now().toSec() - cur_trajectory_.timestamp;
    if (age > 4.0 || age < 0.0 || !cur_trajectory_.valid) // current Trajectory is older than 4.0 seconds or invalid
    {
        return false;
    }

    return true;
}

// Pre-Processing of trajectory data
void TrajectoryControl::TrjDataProc()
{
    // Calculate desired interpolation time for longitudinal values
    double des_time = ros::Time::now().toSec() - cur_trajectory_.timestamp + cfg_.T_LookAhead_Lon;
    // Interpolate longitudinal target values
    v_tgt_ = this->InterpolateTgtValue(cur_trajectory_.TIME, cur_trajectory_.V, des_time, cur_trajectory_.num_Elements);
    a_tgt_ = this->InterpolateTgtValue(cur_trajectory_.TIME, cur_trajectory_.A, des_time, cur_trajectory_.num_Elements);
    if(cur_trajectory_.S[cur_trajectory_.num_Elements]<1.25)
    {
        v_tgt_=0.0;
        a_tgt_=0.0;
        standstill_ = true;
    }
    else if(standstill_ && cur_trajectory_.S[cur_trajectory_.num_Elements]>10.0)
    {
        dv_pid_->Reset();
        dy_pid_->Reset();
        dpsi_pid_->Reset();
        standstill_ = false;
    }

    // Switch to desired interpolation time for lateral values
    des_time = des_time - cfg_.T_LookAhead_Lon + cfg_.T_LookAhead_Lat;
    // Interpolate lateral target values
    y_tgt_ = this->InterpolateTgtValue(cur_trajectory_.TIME, cur_trajectory_.Y, des_time, cur_trajectory_.num_Elements);
    psi_tgt_ = this->InterpolateTgtValue(cur_trajectory_.TIME, cur_trajectory_.THETA, des_time, cur_trajectory_.num_Elements);
    kappa_tgt_ = this->InterpolateTgtValue(cur_trajectory_.TIME, cur_trajectory_.KAPPA, des_time, cur_trajectory_.num_Elements);

    // CalcOdometry
    double dt = ros::Time::now().toSec() - vhcl_ctrl_output_.header.stamp.toSec();
    this->CalcOdometry(dt); //Cyclic Control with 100Hz

    // START TASK 2 CODE HERE
    // calculate vehicle deviations from the trajectory
    // use helping comments from Wiki
    dy_ = 0.0;
    dpsi_ = 0.0;
    // END TASK 2 CODE HERE
}

// interpolates a scalar target value (w.r.t time) from discrete states of the trajectory given as array
double TrajectoryControl::InterpolateTgtValue(std::vector<double> time_array, std::vector<double> val_array, double desired_time, unsigned int num_elements)
{
    if (desired_time < 0.0)
    {
        ROS_ERROR("TrajectoryControl: Desired Time for Trajectory Interpolation is negative!");
        return 0.0;
    }
    // loop over the array and search for sampling points
    int i;
    for (i = 0; i < num_elements; i++)
    {
        if (time_array[i] < desired_time)
        {
            continue;
        }
        else if (time_array[i] == desired_time)
        {
            return val_array[i];
        }
        else
        {
            break;
        }
    }
    if (i < num_elements - 1)
    {
        return val_array[i - 1] + ((val_array[i] - val_array[i - 1]) / (time_array[i] - time_array[i - 1])) * (desired_time - time_array[i - 1]);
    }
    else
    {
        return val_array[i] + ((val_array[i] - val_array[i - 1]) / (time_array[i] - time_array[i - 1])) * (desired_time - time_array[i]);
    }
}

// perform odometry
void TrajectoryControl::CalcOdometry(double dt)
{
    // START TASK 1 CODE HERE
    // use helping comments from Wiki
    double yawRate = cur_vehicle_state_.yaw_rate;
    double velocity = cur_vehicle_state_.velocity;
    odom_dy_ += 0.0;
    odom_dpsi_ += 0.0;
    // END TASK 1 CODE HERE
}

// reset odometry
void TrajectoryControl::ResetOdometry()
{
    odom_dpsi_ = 0.0;
    odom_dy_ = 0.0;
}

// lateral-control function
double TrajectoryControl::LateralControl()
{
    // Cascaded control
    // START TASK 5 CODE HERE
    // use helping comments from Wiki
    double dt = (ros::Time::now() - vhcl_ctrl_output_.header.stamp).toSec();
    double w_y = 0.0;
    double e_y = 0.0;
    double w_psi = 0.0;
    double e_psi = 0.0;
    double psi_dot_des = 0.0;
    // END TASK 5 CODE HERE


    double velocity = cur_vehicle_state_.velocity;
    // be sure v!=0 (to avoid division by zero)
    if (fabs(velocity) < 0.1)
    {
        if (velocity < 0.0)
        {
            velocity = -0.1;
        }
        else
        {
            velocity = 0.1;
        }
    }

    // START TASK 6 CODE HERE
    // use helping comments from Wiki
    double st_ang_pid = 0.0;
    // END TASK 6 CODE HERE

    // Ackermann feed-forward control with trajectory kappa
    double st_ang_ack = atan(wheelbase_ * kappa_tgt_);

    double st_ang = st_ang_pid + st_ang_ack * cfg_.k_FF_stAng;

    // Limit desired steering angle
    if (fabs(st_ang) > lat_max_st_ang_)
    {
        if (st_ang >= 0.0)
        {
            st_ang = lat_max_st_ang_;
        }
        else
        {
            st_ang = -lat_max_st_ang_;
        }
        dy_pid_->Reset();
        dpsi_pid_->Reset();
        ROS_WARN("TrajectoryControl: Steering-Angle limited!");
    }
    // Calculate steering rate with respect to latest target steering angle
    // Multipy vhcl_ctlr_output.steering_angle with -1 because of inverted coordinate system in flatland
    double st_rate = (st_ang - (-vhcl_ctrl_output_.steering_angle)) / dt;
    if (fabs(st_rate) > lat_max_st_rate_)
    {
        if (st_rate >= 0.0)
        {
            // Multipy vhcl_ctlr_output.steering_angle with -1 because of inverted coordinate system in flatland
            st_ang = -vhcl_ctrl_output_.steering_angle + lat_max_st_rate_ * dt;
        }
        else
        {
            // Multipy vhcl_ctlr_output.steering_angle with -1 because of inverted coordinate system in flatland
            st_ang = -vhcl_ctrl_output_.steering_angle - lat_max_st_rate_ * dt;
        }
        dy_pid_->Reset();
        dpsi_pid_->Reset();
        ROS_WARN("TrajectoryControl: Steering-rate limited!");
    }

    return -st_ang;
}

// longitudinal control function
double TrajectoryControl::LongitudinalControl()
{
    // Avoid driving Backwards
    if(standstill_ || cur_vehicle_state_.velocity <-0.1)
    {
        v_tgt_=0.0;
        a_tgt_=0.0;
    }

    // START TASK 4 CODE HERE
    // use helping comments from Wiki
    double dt = ros::Time::now().toSec() - vhcl_ctrl_output_.header.stamp.toSec();
    double velocity = cur_vehicle_state_.velocity;
    double w_v = v_tgt_;
    double e_v = 0.0;
    double a_fb_v = 0.0;
    // END TASK 4 CODE HERE

    double a_ctrl = a_fb_v + a_tgt_ * cfg_.k_FF_a;

    // Limit desired acceleration

    if (a_ctrl > lon_max_acc_)
    {
        a_ctrl = lon_max_acc_;
        if(velocity>1.0)
        {
            dv_pid_->Reset();
        }
        ROS_WARN("TrajectoryControl: Longitudinal acceleration limited!");
    }
    else if (a_ctrl < lon_min_acc_)
    {
        a_ctrl = lon_min_acc_;
        if(velocity>1.0)
        {
            dv_pid_->Reset();
        }
        ROS_WARN("TrajectoryControl: Longitudinal acceleration limited!");
    }

    // Calculate jerk with respect to last desired acceleration
    double jerk = (a_ctrl - vhcl_ctrl_output_.acceleration) / dt;
    if (fabs(jerk) > lon_max_jerk_)
    {
        if (jerk >= 0.0)
        {
            a_ctrl = vhcl_ctrl_output_.acceleration + lon_max_jerk_ * dt;
        }
        else
        {
            a_ctrl = vhcl_ctrl_output_.acceleration - lon_max_jerk_ * dt;
        }
        if(velocity>1.0)
        {
            dv_pid_->Reset();
        }
        ROS_WARN("TrajectoryControl: Longitudinal jerk limited!");
    }

    return a_ctrl;
}
