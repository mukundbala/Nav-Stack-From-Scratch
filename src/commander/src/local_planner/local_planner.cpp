#include "local_planner.h"


LocalPlanner::LocalPlanner(double target_dt , double average_speed , std::string traj_type) 
: target_dt_(target_dt) , average_speed_(average_speed) , traj_type_(traj_type) 
{
    if (traj_type_ != "Linear")
    {
        if (traj_type_ == "Cubic")
        {
            higher_order_trajectory = std::bind(&LocalPlanner::Cubic , this , std::placeholders::_1 , std::placeholders::_2 , std::placeholders::_3 , std::placeholders::_4 );
        }
        else if (traj_type_ == "Quintic")
        {
            higher_order_trajectory = std::bind(&LocalPlanner::Quintic , this , std::placeholders::_1 , std::placeholders::_2 , std::placeholders::_3 , std::placeholders::_4 );
        }
        else
        {
            higher_order_trajectory = std::bind(&LocalPlanner::Quintic , this , std::placeholders::_1 , std::placeholders::_2 , std::placeholders::_3 , std::placeholders::_4 );
            traj_type_ = "Quintic";
        }
    }
    ROS_INFO("[Commander - LocalPlanner]: Local Planner Prepared!");
    ROS_INFO_STREAM("[Commander - LocalPlanner]: Using " << traj_type_ << " trajectory.");
};


std::vector<bot_utils::Pos2D> LocalPlanner::Linear(bot_utils::Pos2D &pos_begin , bot_utils::Pos2D &pos_end)
{
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double d = sqrt((Dx * Dx) + (Dy * Dy)) / average_speed_;

    std::vector<bot_utils::Pos2D> segment_traj = {pos_begin};

    for (double time = target_dt_ ; time < d ; time += target_dt_)
    {
        double x = pos_begin.x + Dx * time / d;
        double y = pos_begin.y + Dy * time / d;
        segment_traj.emplace_back(x , y);
    }

    return segment_traj;
}

std::vector<bot_utils::Pos2D> LocalPlanner::generate_trajectory(std::vector<bot_utils::Pos2D> &path_array)
{
    //Path array: {goal , .... ,.... , start}
    ROS_ERROR_COND(traj_type_ != "Linear", "[Commander - LocalPlanner]: Generate trajectory for Linear is called, but differs from chosen trajectory. Reverting to linear!");
    if (traj_type_ != "Linear")
    {
        traj_type_ = "Linear";
    }
    std::vector<bot_utils::Pos2D> trajectory;

    if (path_array.size() == 0)
    {
        ROS_WARN("[Commander - LocalPlanner]: Empty path given. Return empty trajectory!");
        return trajectory;
    }
    
    else if (path_array.size() == 1)
    {
        ROS_WARN("[Commander - LocalPlanner]: Path size is 1. Returing trajectory with single target");
        trajectory.push_back(path_array.at(0));
        return trajectory;
    }

    else
    {
        for (int i = 1 ; i < path_array.size() ; ++i)
        {
            bot_utils::Pos2D &next = path_array.at(i-1);
            bot_utils::Pos2D &cur = path_array.at(i);

            std::vector<bot_utils::Pos2D> segment_traj = Linear(next , cur);

            for (auto &p : segment_traj)
            {
                trajectory.push_back(p);
            }
        }
        return trajectory;
    }
}

std::vector<bot_utils::Pos2D> LocalPlanner::generate_trajectory(std::vector<bot_utils::Pos2D> &path_array , double mf_vel , double robot_heading)
{
    double initial_vel_x = mf_vel * std::cos(robot_heading);
    double initial_vel_y = mf_vel * std::sin(robot_heading);
    bot_utils::Pos2D initial_vel_vec(initial_vel_x , initial_vel_y);

    std::vector<bot_utils::Pos2D> velocities = {initial_vel_vec};

    for (int i = 1; i < path_array.size()-1 ; ++i)
    {
        bot_utils::Pos2D dir = path_array.at(i+1) - path_array.at(i-1);
        double velocity_heading = atan2(dir.y , dir.x);

        bot_utils::Pos2D vel(average_speed_ * std::cos(velocity_heading) , average_speed_ * std::sin(velocity_heading));
        velocities.push_back(vel);
    }

    velocities.push_back(bot_utils::Pos2D(0,0));

    std::vector<bot_utils::Pos2D> trajectory;

    if (path_array.size() == 0)
    {
        ROS_WARN("[Commander - LocalPlanner]: Empty path given. Return empty trajectory!");
        return trajectory;
    }
    
    else if (path_array.size() == 1)
    {
        ROS_WARN("[Commander - LocalPlanner]: Path size is 1. Returing trajectory with single target");
        trajectory.push_back(path_array.at(0));
        return trajectory;
    }

    else
    {
        for (int i = 1 ; i < path_array.size() ; ++i)
        {
            bot_utils::Pos2D next_pos = path_array.at(i-1);
            bot_utils::Pos2D cur_pos  = path_array.at(i);  
            bot_utils::Pos2D next_vel = velocities.at(i-1);
            bot_utils::Pos2D cur_vel = velocities.at(i);
            std::vector<bot_utils::Pos2D> segment_traj = higher_order_trajectory(next_pos , cur_pos , next_vel , cur_vel);
            for (auto &p : segment_traj)
            {
                trajectory.push_back(p);
            }
        }
        return trajectory;
    }
}

std::vector<bot_utils::Pos2D> LocalPlanner::Cubic(bot_utils::Pos2D &pos_begin , bot_utils::Pos2D &pos_end , bot_utils::Pos2D &vel_begin , bot_utils::Pos2D &vel_end)
{
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double d =  sqrt(Dx * Dx + Dy * Dy) / average_speed_;

    std::vector<bot_utils::Pos2D> traj = {pos_begin};

    double M[4][4] = {{1.0 , 0 , 0 , 0},
                      {0 , 1.0 , 0 , 0},
                      {-3.0 / (d*d) , -2.0 / d , 3.0 / (d*d) , -1 / d},
                      {2 / (d*d*d) , 1 / (d*d) , -2 / (d*d*d) , 1 / (d*d)}
                     };

    std::array<double,4> in_x = {pos_begin.x , vel_begin.x , pos_end.x , vel_end.x};
    std::array<double,4> in_y = {pos_begin.y , vel_begin.y , pos_end.y , vel_end.y};

    std::array<double,4> ax = {0 , 0 , 0 , 0};
    std::array<double,4> by = {0 , 0 , 0 , 0};

    for (int i = 0 ; i < 4 ; ++i)
    {
        for (int j = 0 ; j < 4; ++j)
        {
            ax[i] += (M[i][j] * in_x[j]);
            by[i] += (M[i][j] * in_y[j]);
        }
    }

    for (double time = target_dt_ ; time < d ; time += target_dt_)
    {
        double px = ax[0] + ax[1]*time + ax[2]*time*time + ax[3]*time*time*time;
        double py = by[0] + by[1]*time + by[2]*time*time + by[3]*time*time*time;
        traj.emplace_back(px,py);   
    }

    return traj;
}


std::vector<bot_utils::Pos2D> LocalPlanner::Quintic(bot_utils::Pos2D &pos_begin , bot_utils::Pos2D &pos_end , bot_utils::Pos2D &vel_begin , bot_utils::Pos2D &vel_end)
{
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double d = sqrt(Dx * Dx + Dy * Dy) / average_speed_;
    
    std::vector<bot_utils::Pos2D> traj = {pos_begin};

    double M[6][6] = { {1.0 , 0 , 0 , 0 , 0 , 0},
                       {0 , 1.0 , 0 , 0 , 0 , 0},
                       {0 , 0 , 0.5, 0 , 0 , 0},
                       {-10.0 / (d*d*d) , -6.0 / (d*d) , -3.0 / (2.0 * d) , 10.0 / (d*d*d) , -4.0 / (d*d) , 1.0 / (2.0 * d)},
                       {15.0 / (d*d*d*d) , 8.0 / (d*d*d) , 3.0 / (2.0 * (d*d)) , -15.0 / (d*d*d*d) , 7.0 / (d*d*d), -1.0 / (d*d)},
                       {-6.0 / (d*d*d*d*d) , -3.0 / (d*d*d*d) , -1.0 / (2.0 * (d*d*d)) , 6.0 / (d*d*d*d*d) , -3.0 / (d*d*d*d), 1.0 / (2.0 * (d*d*d))}
                     };
    
    std::array<double,6> in_x = {pos_begin.x , vel_begin.x , 0 , pos_end.x , vel_end.x , 0};
    std::array<double,6> in_y = {pos_begin.y, vel_begin.y , 0 , pos_end.y , vel_end.y , 0};

    std::array<double,6> ax = {0 , 0 , 0 , 0 , 0 , 0};
    std::array<double,6> by = {0 , 0 , 0 , 0 , 0 , 0};

    for (int i = 0 ; i < 6 ; ++i)
    {
        for (int j = 0 ; j < 6; ++j)
        {
            ax[i] += (M[i][j] * in_x[j]);
            by[i] += (M[i][j] * in_y[j]);
        }
    }

    for (double time = target_dt_ ; time < d ; time += target_dt_)
    {
        double px = ax[0] + ax[1]*time + ax[2]*time*time + ax[3]*time*time*time + ax[4]*time*time*time*time + ax[5]*time*time*time*time*time;
        double py = by[0] + by[1]*time + by[2]*time*time + by[3]*time*time*time + by[4]*time*time*time*time + by[5]*time*time*time*time*time;
        traj.emplace_back(px,py);   
    }
    return traj;
}