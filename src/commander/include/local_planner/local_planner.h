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

public:
    LocalPlanner(double target_dt , double average_speed , std::string traj_type);
 
};


#endif //TBOT__LOCAL_PLANNER_H