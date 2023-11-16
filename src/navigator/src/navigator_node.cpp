#include "navigator.h"


int main(int argc, char** argv)
{
    ros::init(argc , argv , "navigator_node");
    ros::NodeHandle nh;
    Navigator ngt(nh);
    ngt.run();
    ROS_INFO("[Navigator]: Halting Navigator Node!");
    return 0;
}