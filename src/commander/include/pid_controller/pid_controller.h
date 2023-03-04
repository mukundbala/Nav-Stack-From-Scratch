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
    std::string damping_function_name_;
    std::function<double(double)> damping_function_;

    double reverse_limit_;

    double dampingCos(double error_value);
    double dampingQuadratic(double error_value);
    double dampingPieceWise(double error_value);
    double dampingExp(double error_value);

public:
    Controller(PIDParams &params);
    void prepareController(bot_utils::Pos2D &robot_position , double robot_heading,  bot_utils::Pos2D &target_position , double time_now);
    bool updateDT(double time_now);
    std::pair<double,double> generate_cmdvel(bot_utils::Pos2D &robot_position , double robot_heading,  bot_utils::Pos2D &target_position);

    double getDt();
};


#endif //PID_CONTROLLER_H