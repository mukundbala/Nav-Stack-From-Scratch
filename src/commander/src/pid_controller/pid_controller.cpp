#include "pid_controller.h"

Controller::Controller(PIDParams &params)
{
    //load everything
    KP_LIN_ = params.KP_LIN;
    KI_LIN_ = params.KI_LIN;
    KD_LIN_ = params.KD_LIN;

    KP_ANG_ = params.KP_ANG;
    KI_ANG_ = params.KI_ANG;
    KD_ANG_ = params.KD_ANG;

    max_lin_vel_ = params.max_lin_vel;
    max_lin_acc_ = params.max_lin_acc;
    
    max_ang_vel_ = params.max_ang_vel;
    max_ang_acc_ = params.max_ang_acc;

    cmd_lin_vel_ = 0;
    cmd_ang_vel_ = 0;

    prev_time_ = 0;
    prev_linear_error_ = 0;
    prev_angular_error_ = 0;

    dt_ = 0;

    cumulative_angular_error_ = 0;
    cumulative_linear_error_ = 0;

    damping_limit_ = params.damping_limit;
    reverse_limit_ = params.reverse_limit; //in degrees

    damping_limit_ *= M_PI/180;
    reverse_limit_ *= M_PI / 180; //converted to radian

    ROS_INFO("[Commander - Controller]: PID Controller Prepared!");
}
