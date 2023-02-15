#include "global_planner.h"

int main(int argc, char** argv)
{
    ros::init(argc , argv,"global_planner_node");
    ros::NodeHandle nh;
    GlobalPlanner gp(nh);
    gp.run();
    ROS_INFO("[GlobalPlanner]: Halting Global Planner Node!");
    return 0;
}