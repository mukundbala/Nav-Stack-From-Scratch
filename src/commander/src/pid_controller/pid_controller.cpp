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

    ROS_INFO("[Commander-Controller]: PID Controller Prepared!");
}

void Controller::prepareController(bot_utils::Pos2D &robot_position , double robot_heading , bot_utils::Pos2D &target , double now_time)
{  
    prev_time_ = now_time; //set prev time
    
    prev_linear_error_ = bot_utils::dist_euc(robot_position , target);
    prev_angular_error_ = bot_utils::limit_angle(atan2(target.y - robot_position.y , target.x - robot_position.x) - robot_heading);

    if (fabs(prev_angular_error_) > reverse_limit_)
    {
        prev_linear_error_ *= -1;
        prev_angular_error_ -= (bot_utils::sign(prev_angular_error_) * M_PI);
    }
    dt_ = 0;
}


bool Controller::updateDT(double now_time)
{
    dt_ = now_time - prev_time_;
    if (dt_ == 0)
    {
        ROS_INFO("[Commander-Controller]: Waiting for ros time");
        return false;
    }
    prev_time_ += dt_;
    return true;
}


double Controller::dampingCoefficient(double angular_error)
{
    if (fabs(angular_error) > damping_limit_)
    {
        return 0;
    }
    else
    {
        double coeff = std::cos(angular_error);
        ROS_WARN_COND(coeff <= 0 , "[Commander-Controller]: Something wrong with the damping coeff");
        return coeff;
    }
}


std::pair<double,double> Controller::generateCmdSignal(bot_utils::Pos2D &robot_position , double robot_heading , bot_utils::Pos2D &target_position)
{
    // ROS_INFO_STREAM("[Commander-Controller]: Target Pos: (" << target_position.x << "," << target_position.y<<")");
    // ROS_INFO_STREAM("[Commander-Controller]: Robot Position: (" << robot_position.x << "," << robot_position.y<<")");
    prev_time_ += dt_;
    //linear error
    double curr_linear_error = bot_utils::dist_euc(robot_position , target_position);
    cumulative_linear_error_ += (curr_linear_error * dt_);

    //linear gains
    double p_cmd_lin = KP_LIN_ * curr_linear_error;
    double i_cmd_lin = KI_LIN_ * cumulative_linear_error_;
    double d_cmd_lin = KD_LIN_ * (curr_linear_error - prev_linear_error_) / dt_;
    double raw_cmd_lin_vel = p_cmd_lin + i_cmd_lin + d_cmd_lin;

    // ROS_INFO_STREAM("[Commander-Controller]: Raw Cmd Lin: " << raw_cmd_lin_vel);

    //angular error
    double curr_angular_error = bot_utils::limit_angle(atan2(target_position.y - robot_position.y , target_position.x - robot_position.x) - robot_heading);

    //process the angular error to account for bidirectional motion
    if (fabs(curr_angular_error) > reverse_limit_)
    {
        //the target is behind the robot, we can reverse to the target
        raw_cmd_lin_vel *= -1;
        curr_angular_error -= (bot_utils::sign(curr_angular_error) * M_PI);
        //at this point, curr_angular_error is constrained between (-reverse_limit_ , +reverse_limit_)
    }
    ROS_WARN_COND(fabs(curr_angular_error) > reverse_limit_ , "[Commander-Controller]: Angular error has not be constrained properly!");
    cumulative_angular_error_ += (curr_angular_error * dt_); //add it to the cumulative angular error

    //angular gains
    double p_cmd_ang = KP_ANG_ * curr_angular_error;
    double i_cmd_ang = KI_ANG_ * cumulative_angular_error_;
    double d_cmd_ang = KD_ANG_ * (curr_angular_error - prev_angular_error_) / dt_;

    //coupled commands
    double coupled_cmd_lin_vel = raw_cmd_lin_vel * dampingCoefficient(curr_angular_error);
    double coupled_cmd_ang_vel = p_cmd_ang + i_cmd_ang + d_cmd_ang;

    //constrain linear acceleration
    double curr_lin_acc = (coupled_cmd_lin_vel - cmd_lin_vel_) / dt_;
    double sat_lin_acc = fabs(curr_lin_acc) > max_lin_acc_ ? bot_utils::sign(curr_lin_acc) * max_lin_acc_ : curr_lin_acc;

    //constrain linear velocity
    double linvel_from_sat_lin_acc = cmd_lin_vel_ + (sat_lin_acc * dt_);
    double sat_lin_vel = fabs(linvel_from_sat_lin_acc) > max_lin_vel_ ? bot_utils::sign(linvel_from_sat_lin_acc) * max_lin_vel_ : linvel_from_sat_lin_acc;

    //constrain angular acceleration
    double curr_ang_acc = (coupled_cmd_ang_vel - cmd_ang_vel_) / dt_;
    double sat_ang_acc = fabs(curr_ang_acc) > max_ang_acc_ ? bot_utils::sign(curr_ang_acc) * max_ang_acc_ : curr_ang_acc;

    //constrain linear velocity
    double angvel_from_sat_ang_acc = cmd_ang_vel_ + (sat_ang_acc * dt_);
    double sat_ang_vel = fabs(angvel_from_sat_ang_acc) > max_ang_vel_ ? bot_utils::sign(angvel_from_sat_ang_acc) * max_ang_vel_ : angvel_from_sat_ang_acc;

    //update previous store
    cmd_lin_vel_ = sat_lin_vel;
    cmd_ang_vel_ = sat_ang_vel;
    prev_linear_error_ = curr_linear_error;
    prev_angular_error_ = curr_angular_error;

    std::pair<double,double> control_commands = {cmd_lin_vel_ , cmd_ang_vel_};
    return control_commands;
}


double Controller::getPrevTime()
{
    return this->prev_time_;
}

double Controller::getDT()
{
    return this->dt_;
}