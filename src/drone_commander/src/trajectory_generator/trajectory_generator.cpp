#include "trajectory_generator.h"


TrajectoryGenerator::TrajectoryGenerator(double target_dt , double average_speed, std::string primary_traj, double cruise_height, double takeoff_height, double land_height, bool verbose):
target_dt_(target_dt),
average_speed_(average_speed),
primary_traj_(primary_traj),
cruise_height_(cruise_height),
takeoff_height_(takeoff_height_),
land_height_(land_height_),
verbose_(verbose)
{
    if (primary_traj_ == "Cubic")
    {
        SplineGenerator_ = std::bind(&TrajectoryGenerator::Cubic , this , std::placeholders::_1 , std::placeholders::_2 , std::placeholders::_3 , std::placeholders::_4 );
    }

    else if (primary_traj_ == "Quintic")
    {
        SplineGenerator_ = std::bind(&TrajectoryGenerator::Quintic , this , std::placeholders::_1 , std::placeholders::_2 , std::placeholders::_3 , std::placeholders::_4 );
    }

    else
    {
        primary_traj_ = "Cubic";
        SplineGenerator_ = std::bind(&TrajectoryGenerator::Quintic , this , std::placeholders::_1 , std::placeholders::_2 , std::placeholders::_3 , std::placeholders::_4 );
    }

    ROS_INFO("[DroneCommander- TrajectoryGenerator]: Trajectory Generator Prepared");
}


std::vector<bot_utils::Pos3D> TrajectoryGenerator::LinearVertTakeOff(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end)
{
    std::vector<bot_utils::Pos3D> segment_traj = {pos_begin};
    return segment_traj;
}

std::vector<bot_utils::Pos3D> TrajectoryGenerator::LinearVertLand(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end)
{
    std::vector<bot_utils::Pos3D> segment_traj = {pos_begin}; //landing position
    
    for (int i = 0 ; i < 30 ; ++i)
    {
        segment_traj.emplace_back(pos_begin.x , pos_begin.y , pos_end.z / 8);
    }

    for (int i = 0 ; i < 30 ; ++i)
    {
        segment_traj.emplace_back(pos_begin.x , pos_begin.y , pos_end.z / 4);
    }

    for (int i = 0 ; i < 30 ; ++i)
    {
        segment_traj.emplace_back(pos_begin.x , pos_begin.y , pos_end.z / 2);
    }

    return segment_traj;
    
}

std::vector<bot_utils::Pos3D> TrajectoryGenerator::LinearPlanar(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end)
{
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double d = sqrt ((Dx * Dx) + (Dy * Dy));

    std::vector<bot_utils::Pos3D> segment_traj = {pos_begin};

    for (double time = target_dt_ ; time < d ; time += target_dt_)
    {
        double x = pos_begin.x + Dx * time / d;
        double y = pos_begin.y + Dy * time / d;
        segment_traj.emplace_back(x , y , cruise_height_);
    }

    return segment_traj;
}

std::vector<bot_utils::Pos3D> TrajectoryGenerator::Cubic(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end , bot_utils::Pos3D &vel_begin , bot_utils::Pos3D &vel_end)
{
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double d =  sqrt(Dx * Dx + Dy * Dy) / average_speed_;

    std::vector<bot_utils::Pos3D> traj = {pos_begin};

    Eigen::MatrixXd M(4,4);
    M << 1.0, 0.0 , 0.0 , 0.0,
         0.0, 1.0 , 0.0 , 0.0,
         1.0, d , d*d , d*d*d,
         0.0, 1.0, 2.0*d, 3*d*d;
    
    Eigen::Vector4d in_x(pos_begin.x , vel_begin.x , pos_end.x , vel_end.x);
    Eigen::Vector4d in_y(pos_begin.y , vel_begin.y , pos_end.y , vel_end.y);

    Eigen::Vector4d ax = M.inverse() * in_x;
    Eigen::Vector4d by = M.inverse() * in_y;

    for (double time = target_dt_ ; time < d ; time += target_dt_)
    {
        double px = ax[0] + ax[1]*time + ax[2]*time*time + ax[3]*time*time*time;
        double py = by[0] + by[1]*time + by[2]*time*time + by[3]*time*time*time;
        traj.emplace_back(px,py,cruise_height_);   
    }

    return traj;
}


std::vector<bot_utils::Pos3D> TrajectoryGenerator::Quintic(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end , bot_utils::Pos3D &vel_begin , bot_utils::Pos3D &vel_end)
{
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double d =  sqrt(Dx * Dx + Dy * Dy) / average_speed_;

    std::vector<bot_utils::Pos3D> traj = {pos_begin};

    Eigen::MatrixXd M_inv(6,6);

    M_inv << 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0,
             0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0,
             0.0 , 0.0 , 0.5,  0.0 , 0.0 , 0.0,
             -10.0/(d*d*d),    -6.0/(d*d),     -3.0/(2.0*d),    10.0/(d*d*d),      -4.0/(d*d),      1.0/(2.0*d),
             15.0/(d*d*d*d),    8.0/(d*d*d),    3.0/(2.0*d*d), -15.0/(d*d*d*d),     7.0/(d*d*d) ,  -1.0/(d*d),
             -6.0/(d*d*d*d*d), -3.0/(d*d*d*d), -1.0/(2.0*d*d*d), 6.0/(d*d*d*d*d),  -3.0/(d*d*d*d),  1.0/(2.0*d*d*d);
    
    Eigen::VectorXd in_x(6);
    in_x << pos_begin.x , vel_begin.x , 0 , pos_end.x , vel_end.x , 0;

    Eigen::VectorXd in_y(6);
    in_y << pos_begin.y , vel_begin.y , 0 , pos_end.y , vel_end.y , 0;

    Eigen::VectorXd ax(6);
    Eigen::VectorXd by(6);

    ax = M_inv * in_x;
    by = M_inv * in_y;

    for (double time = target_dt_ ; time < d ; time += target_dt_)
    {
        double px = ax[0] + ax[1]*time + ax[2]*time*time + ax[3]*time*time*time + ax[4]*time*time*time*time + ax[5]*time*time*time*time*time;
        double py = by[0] + by[1]*time + by[2]*time*time + by[3]*time*time*time + by[4]*time*time*time*time + by[5]*time*time*time*time*time;
        traj.emplace_back(px,py,cruise_height_);   
    }
    return traj;

}   

void TrajectoryGenerator::trajectory_handler(
                        bot_utils::Pos3D current_goal,
                        bot_utils::Pos3D next_goal,
                        bot_utils::Pos3D h_pos, 
                        bot_utils::Pos3D h_vel,
                        bot_utils::SplineData3D &hspline,
                        mission_states::HectorState h_state,
                        mission_states::GoalState g_state)
{
    if (h_state == mission_states::HectorState::TAKEOFF)
    {
        hspline.spline.clear();
        hspline.spline = LinearVertTakeOff(current_goal,h_pos);
        hspline.curr_spline_id++;
    }

    else if (h_state == mission_states::HectorState::LAND)
    {
        hspline.spline.clear();
        hspline.spline = LinearVertLand(current_goal,h_pos);
        hspline.curr_spline_id++;
    }

    else if (h_state == mission_states::HectorState::TURTLE)
    {
        std::vector<bot_utils::Pos3D> spline_a;

        bot_utils::Pos3D vel_at_turtle;
        bot_utils::Pos2D dir_curr(h_pos.x - current_goal.x , h_pos.y - current_goal.y);
        bot_utils::Pos2D dir_next(current_goal.x - next_goal.x , current_goal.y - next_goal.y);

        double dir_next_heading = atan2(dir_next.y , dir_next.x);
        double dir_curr_heading = atan2(dir_curr.y , dir_curr.x);

        double heading_at_turtle = (dir_next_heading + dir_curr_heading) / 2.0;
        vel_at_turtle.x = average_speed_ * std::cos(heading_at_turtle);
        vel_at_turtle.y = average_speed_ * std::sin(heading_at_turtle);
        
        ROS_WARN_COND(g_state == mission_states::GoalState::GOTO , "Something wrong with Trajectory State Machine!");

        if (g_state == mission_states::GoalState::PREDICTION)
        {
            spline_a = SplineGenerator_(current_goal , h_pos, vel_at_turtle , h_vel);
        }

        else //(g_state == "CHASE")
        {
            spline_a = LinearPlanar(current_goal , h_pos);
        }
        hspline.spline.clear();
        hspline.spline = spline_a;
        hspline.curr_spline_id++;
    }

    else if (h_state == mission_states::HectorState::GOAL) 
    {
        ROS_INFO("[DroneCommander]: SERVING GOAL");
        bot_utils::Pos3D vel_at_final;
        bot_utils::Pos3D vel_at_start(0,0,0);

        bot_utils::Pos2D dir_curr(h_pos.x - current_goal.x , h_pos.y - current_goal.y);
        bot_utils::Pos2D dir_next(current_goal.x - next_goal.x , current_goal.y - next_goal.y);

        double dir_next_heading = atan2(dir_next.y , dir_next.x);
        double dir_curr_heading = atan2(dir_curr.y , dir_curr.x);

        double heading_at_final = (dir_next_heading + dir_curr_heading) / 2.0;
        vel_at_final.x = average_speed_ * std::cos(heading_at_final);
        vel_at_final.y = average_speed_ * std::sin(heading_at_final);
    
        std::vector<bot_utils::Pos3D> spline_a = SplineGenerator_(current_goal , h_pos , vel_at_final , h_vel);
        std::vector<bot_utils::Pos3D> spline_b = SplineGenerator_(next_goal, current_goal , vel_at_start , vel_at_final);
        
        for (auto &tgt : spline_a)
        {
            spline_b.push_back(tgt);
        }
        hspline.spline.clear();
        hspline.spline = spline_b;
        hspline.curr_spline_id++;
    }

    else if (h_state == mission_states::HectorState::START)
    {
        ROS_ERROR("Check statemachine. There is an issue if I am here");
        // bot_utils::Pos3D vel_at_start;
        
        // bot_utils::Pos2D dir_curr(h_pos.x - current_goal.x , h_pos.y - current_goal.y);
        // bot_utils::Pos2D dir_next(current_goal.x - next_goal.x , current_goal.y - next_goal.y);

        // double dir_next_heading = atan2(dir_next.y , dir_next.x);
        // double dir_curr_heading = atan2(dir_curr.y , dir_curr.x);

        // double heading_at_start = (dir_next_heading + dir_curr_heading) / 2.0;
        // vel_at_start.x = average_speed_ * std::cos(heading_at_start);
        // vel_at_start.y = average_speed_ * std::sin(heading_at_start);

        // std::vector<bot_utils::Pos3D> spline_a = SplineGenerator_(current_goal , h_pos , vel_at_start , h_vel);
        // hspline.spline.clear();
        // hspline.spline = spline_a;
        // hspline.curr_spline_id++;
    }

    else if (h_state == mission_states::HectorState::FOLLOW || h_state == mission_states::HectorState::HOME)
    {
        std::vector<bot_utils::Pos3D> spline_a;

        if (g_state == mission_states::GoalState::GOTO)
        {
            bot_utils::Pos2D dir(h_pos.x - current_goal.x , h_pos.y - current_goal.x);
            bot_utils::Pos3D vel;
            double heading = atan2(dir.y , dir.x);
            vel.x = average_speed_ * std::cos(heading);
            vel.y = average_speed_ * std::sin(heading);
            vel.z = 0;
            spline_a = SplineGenerator_(current_goal , h_pos , vel , h_vel);
            hspline.spline.clear();
            hspline.spline = spline_a;
            hspline.curr_spline_id++;
        }

        else if (g_state == mission_states::GoalState::CHASE)
        {
            spline_a = LinearPlanar(current_goal , h_pos);
            hspline.spline.clear();
            hspline.spline = spline_a;
            hspline.curr_spline_id++;
        }
    }
}