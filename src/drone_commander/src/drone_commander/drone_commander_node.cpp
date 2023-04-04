#include "drone_commander.h"


int main(int argc, char** argv)
{
    ros::init(argc , argv, "drone_commander_node");
    ros::NodeHandle nh;
    DroneCommander dcmder(nh);
    dcmder.run();
    ROS_INFO("[DroneCommander]: Halting DroneCommander Node!");
    return 0;
}