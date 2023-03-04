#ifndef COMMON_HPP
#define COMMON_HPP
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <chrono>
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

    Position operator + (Position &rhs)const
    {
        Position val(this->x + rhs.x , this->y + rhs.y);
        return val;
    }

    Position operator - (Position &rhs)const
    {
        Position val(this->x - rhs.x , this->y - rhs.y);
        return val;
    }

    double mag()const
    {
        double mag = sqrt((this->x * this->x) + (this->y * this->y));
        return mag;
    }

    Position unit_vec()const
    {
        double vec_mag = this->mag();
        double unit_x = this->x / vec_mag;
        double unit_y = this->y / vec_mag;
        return Position(unit_x , unit_y);
    }

    Position operator * (double scalar)const
    {
        double x_scaled = this->x * scalar;
        double y_scaled = this->y * scalar;
        return Position(x_scaled , y_scaled);
    }

    Position avg(Position &rhs)const
    {
        Position sum(this->x + rhs.x , this->y + rhs.y);
        Position avg = sum * 0.5;
        return avg;
    }
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
double dampingPieceWise(double error_value);
double dampingExp(double error_value);

std::vector<Index> bresenham_los(Index& src, Index& tgt);

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