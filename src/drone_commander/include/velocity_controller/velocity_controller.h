#ifndef HBOT__VELOCITY_CONTROLLER_H
#define HBOT__VELOCITY_CONTROLLER_H

#include "ros/ros.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "hector_uav_msgs/EnableMotors.h"
#include "std_msgs/Bool.h"
#include "bot_utils/bot_utils.h"
#include "velocity_controller_params.h"

#include <stdio.h>
#include <cmath>
#include <limits>
#include <array>
#include <errno.h>
#include <fstream>
#include <signal.h> 
#include <eigen3/Eigen/Dense>
#define NaN std::numeric_limits<double>::quiet_NaN()

class VelocityController
{
private:

    //gains for x-y planar motion
    double Kp_lin_; 
    double Ki_lin_;
    double Kd_lin_;

    //gains for z (up down) motion
    double Kp_z_;
    double Ki_z_;
    double Kd_z_;

    double yaw_rate_;
    
    double max_lin_vel_;
    double max_z_vel_;
    double planar_sat_;

    //VELOCITIES IN ROBOT FRAME!
    double cmd_vel_x_;
    double cmd_vel_y_;
    double cmd_vel_z_;
    double cmd_vel_ang_;
    /*
    Errors computed from target and position are in WORLD FRAME
    However, we will have to convert it to robot frame using Rotation Matrix R
    to the robot frame
    */
    double error_x_prev_;
    double error_y_prev_;
    double error_z_prev_;
    double error_x_cum_;
    double error_y_cum_;
    double error_z_cum_;

    //setup time
    double prev_time_;
    double dt_;

    bool verbose_;

public:
    VelocityController(ControllerParams &cont_params , bool verbose);

    void prepareController(double timenow);
    bool updateDt(double timenow);

    std::array<double,4> generate_velocities(bot_utils::Pos3D &hector_pos ,double hector_heading, bot_utils::Pos3D &target , bool rotate);

};
#endif //HBOT__VELOCITY_CONTROLLER_H