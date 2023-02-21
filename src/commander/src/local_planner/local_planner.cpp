#include "local_planner.h"


LocalPlanner::LocalPlanner(): 
target_dt_(0.04) , 
average_speed_(0.16) , 
traj_type_("interpolate"){};

void LocalPlanner::updateParams(double target_dt , double average_speed , std::string traj_type)
{
    this->target_dt_ = target_dt;
    this->average_speed_ = average_speed;
    this->traj_type_ = traj_type;

    ROS_INFO_STREAM("[Commander]: Local Planner Prepared! Using "<<traj_type);
}

std::deque<bot_utils::Pos2D> LocalPlanner::generateTrajectory(std::vector<bot_utils::Pos2D> &path_array)
{
    if (traj_type_ == "interpolate")
    {
        return Interpolator(path_array);
    }
    else if (traj_type_ == "cubic")
    {
        return Cubic(path_array);
    }
    else
    {
        return Interpolator(path_array);
    }
}
std::deque<bot_utils::Pos2D> LocalPlanner::Interpolator(std::vector<bot_utils::Pos2D> &path_array)
{
    if (path_array.size() == 1) //only one, the goal itself
    {
        std::deque<bot_utils::Pos2D> traj;
        traj.push_back(path_array.at(0));
        return traj;
    }
    
    else
    {
        std::deque<bot_utils::Pos2D> traj;

        //our path array starts at the robot and ends at the goal
        for (int i = 0 ; i < path_array.size()-1 ; ++i)
        {
            bot_utils::Pos2D& curr = path_array[i];
            bot_utils::Pos2D& next = path_array[i+1];

            double Dx = next.x - curr.x;
            double Dy = next.y - curr.y;
            double duration = sqrt(Dx * Dx + Dy * Dy) / average_speed_;

            for (double time = target_dt_ ; time < duration ; time += target_dt_)
            {
                //we step by dt

                traj.emplace_back(curr.x + Dx * time / duration , curr.y + Dy * time / duration);
            }
        }
        ROS_INFO("[Commander-LocalPlanner]: Trajectory Generated!");
        return traj;
    }
}


std::deque<bot_utils::Pos2D> LocalPlanner::Cubic(std::vector<bot_utils::Pos2D> &path_array)
{
    
}
