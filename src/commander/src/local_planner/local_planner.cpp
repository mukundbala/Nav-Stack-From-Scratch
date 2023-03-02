#include "local_planner.h"


LocalPlanner::LocalPlanner(double target_dt , double average_speed , std::string traj_type) 
: target_dt_(target_dt) , average_speed_(average_speed) , traj_type_(traj_type) 
{
    ROS_INFO("[Commander - Controller]: PID Controller Prepared!");
};
