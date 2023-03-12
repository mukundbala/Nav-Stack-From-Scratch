#include "velocity_controller.h"


VelocityController::VelocityController(ControllerParams &cont_params , bool verbose)
{
    Kp_lin_ = cont_params.KP_LIN;
    Ki_lin_ = cont_params.KI_LIN;
    Kd_lin_ = cont_params.KD_LIN;

    Kp_z_ = cont_params.KP_Z;
    Ki_z_ = cont_params.KI_Z;
    Kd_z_ = cont_params.KD_Z;

    yaw_rate_ = cont_params.YAW_RATE;

    max_lin_vel_ = cont_params.MAX_LIN_VEL;
    max_z_vel_ = cont_params.MAX_Z_VEL;
    planar_sat_ = sqrt(max_lin_vel_);

    cmd_vel_x_ = 0;
    cmd_vel_y_ = 0;
    cmd_vel_z_ = 0;

    error_x_prev_ = 0;
    error_y_prev_ = 0;
    error_z_prev_ = 0;
    error_x_cum_ = 0;
    error_y_cum_ = 0;
    error_z_cum_ = 0;

    verbose_ = verbose;

    ROS_INFO("[DroneCommander - VelocityController]: Velocity Controller Prepared!");
}

void VelocityController::prepareController(double timenow)
{
    prev_time_ = timenow;
}

bool VelocityController::updateDt(double timenow)
{
    dt_ = timenow - prev_time_;
    
    if (dt_ == 0)
    {
        return false;
    }

    prev_time_ += dt_;
    return true;
}

std::array<double,4> VelocityController::generate_velocities(bot_utils::Pos3D &hector_pos , double hector_heading, bot_utils::Pos3D &target , bool rotate)
{
    double error_x_world = target.x - hector_pos.x;
    double error_y_world = target.y - hector_pos.y;

    Eigen::Matrix2d R_MATRIX; //matrix that converts from World Frame to Robot Frame

    R_MATRIX << std::cos(hector_heading),std::sin(hector_heading),
                -std::sin(hector_heading),std::cos(hector_heading);
    
    Eigen::Vector2d planar_error_world(error_x_world,error_y_world);
    Eigen::Vector2d planar_error_robot = R_MATRIX * planar_error_world;

    double error_x = planar_error_robot[0];
    double error_y = planar_error_robot[1];
    double error_z = target.z - hector_pos.z;

    double error_x_diff = error_x - error_x_prev_;
    double error_y_diff = error_y - error_y_prev_;
    double error_z_diff = error_z - error_z_prev_;

    error_x_cum_ += (error_x * dt_);
    error_y_cum_ += (error_y * dt_);
    error_z_cum_ += (error_z * dt_);

    double px = Kp_lin_ * error_x;
    double py = Kp_lin_ * error_y;
    double pz = Kp_lin_ * error_z;

    double ix = Ki_lin_ * error_x_cum_;
    double iy = Ki_lin_ * error_y_cum_;
    double iz = Ki_lin_ * error_z_cum_;

    double dx = Kd_lin_ * (error_x_diff / dt_);
    double dy = Kd_lin_ * (error_y_diff / dt_);
    double dz = Kd_lin_ * (error_z_diff / dt_);

    double raw_x = px + ix + dx;
    double raw_y = py + iy + dy;
    double raw_z = pz + iz + dz;

    double acc_x = (raw_x - cmd_vel_x_) / dt_;
    double est_x_vel = cmd_vel_x_ + (acc_x * dt_);

    double acc_y = (raw_y - cmd_vel_y_) / dt_;
    double est_y_vel = cmd_vel_y_ + (acc_y * dt_);

    double acc_z = (raw_z - cmd_vel_z_) / dt_;
    double est_z_vel = cmd_vel_z_ + (acc_z * dt_);

    cmd_vel_x_ = est_x_vel > planar_sat_ ? planar_sat_ : est_x_vel;
    cmd_vel_y_ = est_y_vel > planar_sat_ ? planar_sat_ : est_y_vel;
    cmd_vel_z_ = est_z_vel > max_z_vel_ ? max_z_vel_ : est_z_vel;
    cmd_vel_ang_ = rotate ? yaw_rate_ : 0;

    error_x_prev_ = error_x;
    error_y_prev_ = error_y;
    error_z_prev_ = error_z;

    if (verbose_)
    {
        ROS_INFO("[DroneCommander - Velocity Generator]:VELOCITIES:");
        ROS_INFO_STREAM("CMD_VEL_X: " << cmd_vel_x_);
        ROS_INFO_STREAM("CMD_VEL_Y: " << cmd_vel_y_);
        ROS_INFO_STREAM("CMD_VEL_Z: " << cmd_vel_z_);
        ROS_INFO_STREAM("CMD_ANG_VEL: " << cmd_vel_ang_);
    }
    std::array<double,4> vels = {cmd_vel_x_ , cmd_vel_y_ , cmd_vel_z_ , cmd_vel_ang_};
    
    return vels;

}