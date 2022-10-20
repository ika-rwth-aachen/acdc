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
 * @file TrajectoryCtrl.h
 * @author Guido Küppers
 * @brief  ROS-Node for trajectory control.
 * @details The trajector_ctrl_node receives an ika-Trajectory and generates the specific vehicle controls!
 */

#include <ros/ros.h>
#include <cmath>

#include <definitions/FlatlandVehicleState.h>
#include <definitions/IkaTpTrajectoryInterface.h>
#include <PID.h>

// Parameters
#include <dynamic_reconfigure/server.h>
#include <trajectory_ctrl/trj_ctrlConfig.h>

class TrajectoryControl
{
public:
    TrajectoryControl(const ros::NodeHandle &node_handle);
    ~TrajectoryControl();

private:
    void VehicleStateCallback(const definitions::FlatlandVehicleState::ConstPtr &msg);
    void TrajectoryCallback(const definitions::IkaTpTrajectoryInterface::ConstPtr &msg);
    void VehicleCtrlCycle(const ros::TimerEvent &event);
    bool InputSanityCheck();
    void TrjDataProc();
    double InterpolateTgtValue(std::vector<double> time_array, std::vector<double> val_array, double desired_time, unsigned int num_elements);
    void CalcOdometry(double dt);
    void ResetOdometry();
    double LateralControl();
    double LongitudinalControl();

    void callbackDynReconf(trajectory_ctrl::trj_ctrlConfig &config, uint32_t level);
    trajectory_ctrl::trj_ctrlConfig cfg_;
    dynamic_reconfigure::Server<trajectory_ctrl::trj_ctrlConfig> srv_;

    ros::NodeHandle node_handle_;

    ros::Subscriber *vehicle_state_sub_;
    ros::Subscriber *trajectory_sub_;

    ros::Publisher *vehicle_ctrl_pub_;

    ros::Timer *vhcl_ctrl_timer_;

    definitions::FlatlandVehicleState cur_vehicle_state_;
    definitions::IkaTpTrajectoryInterface cur_trajectory_;

    // TrajectoryControl Parameters
    double lat_t_lookahead_ = 0.1;
    double lon_t_lookahead_ = 0.1;

    const double lon_max_acc_ = 3.5;
    const double lon_min_acc_ = -5.0; //be sure that this value is negative
    const double lon_max_jerk_ = 10.0;

    const double lat_max_st_ang_ = 28.0*M_PI/180.0;
    const double lat_max_st_rate_ = 56.0*M_PI/180.0;

    // Vehicle Parameters
    const double wheelbase_ = 2.711;
    const double self_st_gradient_ = 0.002917853041365;

    // TrajectoryControl Variables
    double a_tgt_;
    double a_tgt_dv_;
    double v_tgt_;
    double y_tgt_;
    double psi_tgt_;
    double kappa_tgt_;

    // Control Deviations
    double dpsi_;
    double dy_;
    double dv_;

    // Controller
    PID *dv_pid_;
    PID *dy_pid_;
    PID *dpsi_pid_;

    // Control Output
    definitions::FlatlandVehicleState vhcl_ctrl_output_;

    // Odometry
    double odom_dy_ = 0.0;
    double odom_dpsi_ = 0.0;

    bool standstill_ = false;

}; //class TrajectoryControl
