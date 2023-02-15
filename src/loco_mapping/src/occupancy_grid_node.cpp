#include "occupancy_grid.h"

int main(int argc, char** argv)
{
    ros::init(argc , argv , "occupancy_grid_node");
    ros::NodeHandle nh;
    OccupancyGrid og(nh);
    og.run();
    ROS_INFO("[OccupancyGrid]: Halting Occupancy Grid Node!");
    return 0;
}