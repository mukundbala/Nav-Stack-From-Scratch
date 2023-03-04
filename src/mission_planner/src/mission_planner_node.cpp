#include "mission_planner.h"


int main(int argc, char** argv)
{
    ros::init(argc , argv , "mission_planner_node");
    ros::NodeHandle nh;
    MissionPlanner mp(nh);
    mp.run();
    ROS_INFO("[MissionPlanner]: Halting Mission Planner Node!");
    return 0;
}