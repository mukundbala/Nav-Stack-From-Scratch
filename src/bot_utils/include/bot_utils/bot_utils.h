#ifndef TBOT__BOT_UTILS_H
#define TBOT__BOT_UTILS_H
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
#include <cmath>
#include <deque>
#include <chrono>
#include <functional>
#include <utility>
namespace bot_utils
{//bot_utils namespace

//Position objects
struct Pos2D
{
    double x;

    double y;

    Pos2D(double x_, double y_);

    Pos2D();

    double EPS_ = 1e-6;

    void setCoords(double x_, double y_);
    
    bool operator == (Pos2D &rhs) const
    {
       return (fabs(this->x - rhs.x) < EPS_) && (fabs(this->y - rhs.y) < EPS_);
    }
    
    bool operator != (Pos2D &rhs)const
    {
        return !(fabs(this->x - rhs.x) < EPS_) && (fabs(this->y - rhs.y) < EPS_);
    }

    Pos2D operator + (Pos2D &rhs)const
    {
        Pos2D val(this->x + rhs.x , this->y + rhs.y);
        return val;
    }

    Pos2D operator - (Pos2D &rhs)const
    {
        Pos2D val(this->x - rhs.x , this->y - rhs.y);
        return val;
    }

    double mag()const
    {
        double mag = sqrt((this->x * this->x) + (this->y * this->y));
        return mag;
    }

    Pos2D unit_vec()const
    {
        double vec_mag = this->mag();
        double unit_x = this->x / vec_mag;
        double unit_y = this->y / vec_mag;
        return Pos2D(unit_x , unit_y);
    }

    Pos2D operator * (double scalar)const
    {
        double x_scaled = this->x * scalar;
        double y_scaled = this->y * scalar;
        return Pos2D(x_scaled , y_scaled);
    }

    void print()
    {
        ROS_INFO_STREAM("(" << x << "," << y << ")");
    }
};

struct Pos3D
{
    double x;
    double y;
    double z;
    Pos3D(double x_, double y_, double z_);
    Pos3D();
    void setCoords(double x_, double y_ , double z_);

    void print()
    {
        ROS_INFO_STREAM("(" << x << "," << y << "," << z << ")");
    }
};

//Index object
struct Index
{
    int i;
    int j;
    Index(int i_ , int j_);
    Index();
    void setIdx(int x_ , int y_);
};

double sign(double value); // sign function not defined in cmath. This is okay to use on ints too due to typecasting...

//octile distances
double dist_oct(Index src, Index tgt);
double dist_oct(Pos2D src, Pos2D tgt);
double dist_oct(double src_x, double src_y, double tgt_x, double tgt_y);

//euclidean distance
double dist_euc(Index src, Index tgt);
double dist_euc(Pos2D src, Pos2D tgt);
double dist_euc(double src_x, double src_y, double tgt_x, double tgt_y);

//math stuff
double heading(Pos2D src, Pos2D tgt); // overload
double limit_angle(double angle);
double headingFromQuat(geometry_msgs::PoseStamped &pose);
double dampingCos(double error_value);
double dampingQuadratic(double error_value);
//kill limit means that between |[0,kill_limit]|, damping follows cos, after which it returns 0, therefore killing cmd_lin_vel to 0
double dampingPieceWise(double error_value , double kill_limit); 

//Line of sight algorithm
std::vector<Index> bresenham_los(Index &src, Index &tgt);

//testing stuff
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
}//bot_utils namespace
#endif //TBOT__BOT_UTILS_H