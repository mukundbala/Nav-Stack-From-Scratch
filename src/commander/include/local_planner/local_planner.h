#ifndef TBOT__LOCAL_PLANNER_H
#define TBOT__LOCAL_PLANNER_H

#include "ros/ros.h"
#include "bot_utils/bot_utils.h"

#include <vector>
#include <deque>
#include <functional>

/*
The local planner, aka trajectory generator, takes in a path array
of type std::vector<bot_utils::Pos2D> and returns a 
std::deque<bot_utils::Pos2D> of targets.

We have access to 2 types of trajectories: Interpolator and Cubic


*/
class LocalPlanner
{
private:
    double target_dt_;
    double average_speed_;
    std::string traj_type_;

public:
    LocalPlanner();

    void updateParams(double target_dt , double average_speed , std::string traj_type);

    std::deque<bot_utils::Pos2D> Interpolator(std::vector<bot_utils::Pos2D> &path_array); //interpolator

    std::deque<bot_utils::Pos2D> Cubic(std::vector<bot_utils::Pos2D> &path_array); //cubic

    std::deque<bot_utils::Pos2D> generateTrajectory(std::vector<bot_utils::Pos2D> &path_array); //wrapper for both

        
};


#endif //TBOT__LOCAL_PLANNER_H