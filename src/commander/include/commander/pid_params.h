#ifndef TBOT__PID_PARAMS_H
#define TBOT__PID_PARAMS_H

struct PIDParams
{
    double KP_LIN;
    double KI_LIN;
    double KD_LIN;
    double KP_ANG;
    double KI_ANG;
    double KD_ANG;
    
    double max_lin_vel;
    double max_lin_acc;

    double max_ang_vel;
    double max_ang_acc;

    double cmd_lin_vel;
    double cmd_ang_vel;

    double prev_time;
    double prev_linear_error;
    double prev_angular_error;

    double cumulative_angular_error;
    double cumulative_linear_error;

    double dt;

    double damping_limit;
    std::string damping_function;
    
    double reverse_limit;
};



#endif //TBOT__PID_PARAMS_H