#include "motion_filter.h"

int main(int argc, char** argv)
{
    ros::init(argc , argv,"motion_filter_node");
    ros::NodeHandle nh;
    MotionFilter mf(nh);
    mf.run();
    return 0;
}