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
    damping_function_name_ = params.damping_function;

    reverse_limit_ *= M_PI / 180; //converted to radian

    if (damping_function_name_ == "Cos")
    {
        damping_function_ = std::bind(&Controller::dampingCos , this , std::placeholders::_1);
    }

    else if (damping_function_name_ == "Quad")
    {
        damping_function_ = std::bind(&Controller::dampingQuadratic , this , std::placeholders::_1);
    }

    else if (damping_function_name_ == "PieceWise")
    {
        damping_function_ = std::bind(&Controller::dampingPieceWise , this , std::placeholders::_1);
    }
    
    else if (damping_function_name_ == "Exp")
    {
        damping_function_ = std::bind(&Controller::dampingExp , this , std::placeholders::_1);
    }

    else
    {
        ROS_WARN("[Commander - Controller]: Valid damping function not found. Defaulting to PieceWise");
        damping_function_name_  = "PieceWise";
        damping_function_ = std::bind(&Controller::dampingPieceWise , this , std::placeholders::_1);
    }
    ROS_INFO_STREAM("[Commander - Controller]: Using " << damping_function_name_ << " function!");
    
    ROS_INFO("[Commander - Controller]: PID Controller Prepared!");   
}


void Controller::prepareController(bot_utils::Pos2D &robot_position , double robot_heading , bot_utils::Pos2D &target_position , double time_now)
{
    cmd_lin_vel_ = 0;
    cmd_ang_vel_ = 0;
    dt_ = 0;
    prev_time_ = time_now;

    prev_linear_error_ = bot_utils::dist_euc(robot_position , target_position);
    prev_angular_error_ = bot_utils::limit_angle(atan2(target_position.y - robot_position.y , target_position.x - robot_position.x) - robot_heading);
    
    if (fabs(prev_angular_error_) > M_PI / 2)
    {
        prev_angular_error_ -= (bot_utils::sign(prev_angular_error_) * M_PI);
    }

    cumulative_linear_error_ = 0;
    cumulative_angular_error_ = 0;
}

double Controller::dampingCos(double error_value)
{
    return std::cos(error_value);
}

double Controller::dampingPieceWise(double error_value)
{
    double kill_limit = M_PI / 3; //we can change this accordingly
    if (fabs(error_value) > kill_limit)
    {
        return 0;
    }
    else
    {
        double coeff = std::cos(error_value);
        ROS_WARN_COND(coeff <= 0 , "Warning, Something wrong with the damping coeff");
        return coeff; // a value between less than 1 
    }
}


double Controller::dampingQuadratic(double error_value)
{
    double value = (-1 / (0.25 * M_PI * M_PI)) * (error_value - M_PI / 2) * (error_value + M_PI / 2);
    return value;
}



double Controller::dampingExp(double error_value)
{
    double exp_arg = -((-std::fabs(error_value) + M_PI / 4.0));
    double denom = 1.0 + std::pow(std::exp(exp_arg),8);

    double coeff = 1.0 / denom;

    //safeguard
    if (coeff > 1 || coeff < 0)
    {
        ROS_WARN("[Tmove]: Exponential Coefficient out of bounds. Reverting to piecewise");
        coeff = dampingPieceWise(error_value);
    }
    
    return coeff;
}

std::pair<double,double> Controller::generate_cmdvel(bot_utils::Pos2D &robot_position , double robot_heading,  bot_utils::Pos2D &target_position)
{
            double curr_linear_error = dist_euc(robot_position , target_position);
            cumulative_linear_error_ += (curr_linear_error * dt_);

            //linear gains
            double p_cmd_lin = KP_LIN_ * curr_linear_error;
            double i_cmd_lin = KI_LIN_ * cumulative_linear_error_;
            double d_cmd_lin = KD_LIN_ * (curr_linear_error - prev_linear_error_) / dt_;
            double raw_cmd_lin_vel = p_cmd_lin + i_cmd_lin + d_cmd_lin; //the raw signal based on euc error

            //angular errors
            double curr_angular_error = bot_utils::limit_angle(atan2(target_position.y - robot_position.y , target_position.x - robot_position.x) - robot_heading); // [-pi,pi)

            //we want to check if the errors are greater than |pi/2| aka 90 degrees on either side
            if (fabs(curr_angular_error) > M_PI/2)
            {
                raw_cmd_lin_vel *= -1; //going to reverse to the target
                curr_angular_error -= (bot_utils::sign(curr_angular_error) * M_PI); //now, curr_angular error is constrained between (-pi/2 , pi/2)
            } //at this point, we have the final curr_angular_error!

            ROS_WARN_COND(fabs(curr_angular_error) > M_PI/2 , "Current angular error has not be constrained properly!");
            cumulative_angular_error_ += (curr_angular_error * dt_); //we can add in the cumulative angular error now
            
            double coupled_cmd_vel = raw_cmd_lin_vel * damping_function_(curr_angular_error); //apply the coefficient to raw cmd_vel to couple it to ang error

            double p_cmd_ang = KP_ANG_ * curr_angular_error;
            double i_cmd_ang = KI_ANG_ * cumulative_angular_error_;
            double d_cmd_ang = KD_ANG_ * (curr_angular_error - prev_angular_error_) / dt_ ;
            double coupled_cmd_ang = p_cmd_ang + i_cmd_ang + d_cmd_ang;

            double curr_lin_acc = (coupled_cmd_vel - cmd_lin_vel_) / dt_;
            double sat_lin_acc = fabs(curr_lin_acc) > max_lin_acc_ ? bot_utils::sign(curr_lin_acc) * max_lin_acc_ : curr_lin_acc;
            double linvel_from_sat_lin_acc = cmd_lin_vel_ + (sat_lin_acc * dt_);
            double sat_lin_vel = fabs(linvel_from_sat_lin_acc) > max_lin_vel_ ? bot_utils::sign(linvel_from_sat_lin_acc) * max_lin_vel_ : linvel_from_sat_lin_acc;

            double curr_ang_acc = (coupled_cmd_ang - cmd_ang_vel_) / dt_;
            double sat_ang_acc = fabs(curr_ang_acc) > max_ang_acc_ ? bot_utils::sign(curr_ang_acc) * max_ang_acc_ : curr_ang_acc;
            double angvel_from_sat_ang_acc = cmd_ang_vel_ + (sat_ang_acc * dt_);
            double sat_ang_vel = fabs(angvel_from_sat_ang_acc) > max_ang_vel_ ? bot_utils::sign(angvel_from_sat_ang_acc) * max_ang_vel_ : angvel_from_sat_ang_acc;
            
            
            cmd_lin_vel_ = sat_lin_vel;
            cmd_ang_vel_ = sat_ang_vel;
            prev_linear_error_ = curr_linear_error;
            prev_angular_error_ = curr_angular_error;

            std::pair<double,double> cmd_vel = {cmd_lin_vel_ , cmd_ang_vel_};
            return cmd_vel;
}

bool Controller::updateDT(double time_now)
{
    dt_ = time_now - prev_time_;
    if (dt_ == 0.0)
    {
        return false;
    }
    prev_time_ += dt_;
    return true;
}

double Controller::getDt()
{
    return this->dt_;
}