#ifndef TBOT__LOCAL_PLANNER_H
#define TBOT__LOCAL_PLANNER_H

#include "ros/ros.h"
#include "bot_utils/bot_utils.h"

#include <vector>
#include <functional>

class LocalPlanner
{
private:
    double target_dt_;
    double average_speed_;
    std::string traj_type_;
    std::function<std::vector<bot_utils::Pos2D>(bot_utils::Pos2D &pos_begin , 
                                                bot_utils::Pos2D &pos_end , 
                                                bot_utils::Pos2D &vel_begin , 
                                                bot_utils::Pos2D &vel_end)> higher_order_trajectory;
public:
    LocalPlanner(double target_dt , double average_speed , std::string traj_type);

    std::vector<bot_utils::Pos2D> generate_trajectory(std::vector<bot_utils::Pos2D> &path_array);

    std::vector<bot_utils::Pos2D> generate_trajectory(std::vector<bot_utils::Pos2D> &path_array , double mf_vel , double robot_heading);

    std::vector<bot_utils::Pos2D> Linear(bot_utils::Pos2D &pos_begin , bot_utils::Pos2D &pos_end);

    std::vector<bot_utils::Pos2D> Cubic(bot_utils::Pos2D &pos_begin , bot_utils::Pos2D &pos_end , bot_utils::Pos2D &vel_begin , bot_utils::Pos2D &vel_end);
    
    std::vector<bot_utils::Pos2D> Quintic(bot_utils::Pos2D &pos_begin , bot_utils::Pos2D &pos_end , bot_utils::Pos2D &vel_begin , bot_utils::Pos2D &vel_end);
};


#endif //TBOT__LOCAL_PLANNER_H