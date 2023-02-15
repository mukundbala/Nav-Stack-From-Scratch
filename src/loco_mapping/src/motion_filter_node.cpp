#include "motion_filter.h"

int main(int argc, char** argv)
{
    ros::init(argc , argv,"motion_filter_node");
    ros::NodeHandle nh;
    MotionFilter mf(nh);
    mf.run();
    ROS_INFO("[MotionFilter]: Halting Motion Filter Node!");
    return 0;
}