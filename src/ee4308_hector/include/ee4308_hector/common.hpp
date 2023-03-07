#ifndef COMMON_HPP
#define COMMON_HPP

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
double heading(double src_x, double src_y, double tgt_x, double tgt_y);
double limit_angle(double angle);

#endif