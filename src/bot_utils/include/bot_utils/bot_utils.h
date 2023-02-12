#ifndef TBOT__BOT_UTILS_H
#define TBOT__BOT_UTILS_H
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
#include <cmath>
#include <deque>

namespace bot_utils
{//bot_utils namespace

//Position objects
struct Pos2D
{
    double x;
    double y;
    Pos2D(double x_, double y_);
    Pos2D();
    void setCoords(double x_, double y_);
};

struct Pos3D
{
    double x;
    double y;
    double z;
    Pos3D(double x_, double y_, double z_);
    Pos3D();
    void setCoords(double x_, double y_ , double z_);
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
std::deque<Index> bresenham_los(Index &src, Index &tgt);

}//bot_utils namespace
#endif //TBOT__BOT_UTILS_H