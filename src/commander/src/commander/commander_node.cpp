#include "commander.h"

int main(int argc, char** argv)
{
    ros::init(argc , argv, "commander_node");
    ros::NodeHandle nh;
    Commander cmder(nh);
    cmder.run();
    ROS_INFO("[Commander]: Halting Commander Node!");
}