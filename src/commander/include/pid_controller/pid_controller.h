#ifndef TBOT__PID_CONTROLLER_H
#define TBOT__PID_CONTROLLER_H

#include "ros/ros.h"
#include "bot_utils/bot_utils.h"
#include "pid_params.h"

#include <cmath>
class Controller
{
private:
    double KP_LIN_;
    double KI_LIN_;
    double KD_LIN_;
    double KP_ANG_;
    double KI_ANG_;
    double KD_ANG_;
    
    double max_lin_vel_;
    double max_lin_acc_;

    double max_ang_vel_;
    double max_ang_acc_;

    double cmd_lin_vel_;
    double cmd_ang_vel_;

    double prev_time_;
    double prev_linear_error_;
    double prev_angular_error_;

    double cumulative_angular_error_;
    double cumulative_linear_error_;

    double dt_;

    double damping_limit_;
    double reverse_limit_;

    void prepareController();
    double dampingCoefficient(double angular_error); //damping

public:
    Controller(PIDParams &params);

};


#endif //PID_CONTROLLER_H