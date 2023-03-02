#ifndef COMMON_HPP
#define COMMON_HPP
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
struct Index
{
    int i=0, j=0;
    Index();
    Index(int i, int j);
};
struct Position
{
    double x=0, y=0;
    Position();
    Position(double x, double y);
};
double sign(double value); // sign function not defined in cmath
double dist_oct(Index src, Index tgt);
double dist_oct(Position src, Position tgt);
double dist_oct(double src_x, double src_y, double tgt_x, double tgt_y);
double dist_euc(Index src, Index tgt);
double dist_euc(Position src, Position tgt);
double dist_euc(double src_x, double src_y, double tgt_x, double tgt_y);
double heading(Position src, Position tgt); // overload
double limit_angle(double angle);
double headingFromQuat(geometry_msgs::PoseStamped &pose);
double dampingCos(double error_value);
double dampingQuadratic(double error_value);
//kill limit means that between |[0,kill_limit]|, damping follows cos, after which it returns 0, therefore killing cmd_lin_vel to 0
double dampingPieceWise(double error_value);

class timeLogger
{
private:
    std::chrono::high_resolution_clock clock_;
    std::chrono::high_resolution_clock::time_point start_;
    std::chrono::high_resolution_clock::time_point end_;
    std::chrono::duration<double,std::milli> elapsed_time_;
    std::chrono::duration<double,std::milli> total_duration_;
    double count_;

public:
    timeLogger();
    void start();
    void stop();
};

#endif