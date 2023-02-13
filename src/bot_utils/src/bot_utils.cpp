#include "bot_utils/bot_utils.h"

bot_utils::Index::Index() : i(0), j(0) {};

bot_utils::Index::Index(int i_, int j_) : i(i_), j(j_) {};

void bot_utils::Index::setIdx(int i_ , int j_)
{
    this->i = i_;
    this->j = j_;
}

bot_utils::Pos2D::Pos2D() : x(0), y(0) {};

bot_utils::Pos2D::Pos2D(double x_, double y_) : x(x_), y(y_) {};

void bot_utils::Pos2D::setCoords(double x_ , double y_)
{
    this->x = x_;
    this->y = y_;
}

bot_utils::Pos3D::Pos3D() : x(0), y(0) , z(0) {};

bot_utils::Pos3D::Pos3D(double x_, double y_ , double z_) : x(x_), y(y_) ,z(z_){};

void bot_utils::Pos3D::setCoords(double x_ , double y_ , double z_)
{
    this->x = x_;
    this->y = y_;
    this->z = z_;
}

double bot_utils::sign(double value)
{
    if (value > 0)
        return 1;
    else if (value < 0)
        return -1;
    else
        return 0;
}

double bot_utils::dist_oct(double src_x, double src_y, double tgt_x, double tgt_y)
{
    double abs_Dx = abs(tgt_x - src_x);
    double abs_Dy = abs(tgt_y - src_x);
    double ordinals, cardinals;
    if (abs_Dx > abs_Dy)
    {
        ordinals = abs_Dy; // ordinal (diagonal)
        cardinals = abs_Dx - abs_Dy; // cardinal (vertical or horizontal)
    }
    else
    {
        ordinals = abs_Dx;
        cardinals = abs_Dy - abs_Dx;
    }
    return ordinals * M_SQRT2 + cardinals; // M_SQRT2 is from cmath
}

double bot_utils::dist_oct(Index src, Index tgt)
{
    return dist_oct(src.i, src.j, tgt.i, tgt.j);
}

double bot_utils::dist_oct(Pos2D src, Pos2D tgt)
{
    return dist_oct(src.x, src.y, tgt.x, tgt.y);
}

double bot_utils::dist_euc(Index src, Index tgt)
{
    return dist_euc(src.i, src.j, tgt.i, tgt.j);
}

double bot_utils::dist_euc(Pos2D src, Pos2D tgt)
{
    return dist_euc(src.x, src.y, tgt.x, tgt.y);
}

double bot_utils::dist_euc(double src_x, double src_y, double tgt_x, double tgt_y)
{
    double Dx = tgt_x - src_x;
    double Dy = tgt_y - src_y;
    return sqrt(Dx*Dx + Dy*Dy);
}

double bot_utils::heading(Pos2D src, Pos2D tgt)
{
    double Dx = tgt.x - src.x;
    double Dy = tgt.y - src.y;

    return atan2(Dy, Dx);
}

double bot_utils::limit_angle(double angle)
{
    // return fmod(angle + M_PI, M_PI*2) - M_PI;
    double result = fmod(angle + M_PI, M_PI*2); // fmod rounds remainders to zero. we want remainders to be +ve like mod() in matlab and % in python
    return result >= 0 ? result - M_PI : result + M_PI;
}

double bot_utils::headingFromQuat(geometry_msgs::PoseStamped &pose)
{
    double siny_cosp = 2 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (pose.pose.orientation.y * pose.pose.orientation.y + pose.pose.orientation.z * pose.pose.orientation.z);
    double heading = atan2(siny_cosp, cosy_cosp);
    return heading; //atan2 constrains this
}

double bot_utils::dampingCos(double error_value)
{
    return cos(error_value);
}

double bot_utils::dampingQuadratic(double error_value)
{
    double value = (-1 / (0.25 * M_PI * M_PI)) * (error_value - M_PI) * (error_value + M_PI);
    return value;
}

double bot_utils::dampingPieceWise(double error_value , double kill_limit)
{
    if (fabs(error_value) > kill_limit)
    {
        return 0;
    }
    else
    {
        double coeff = std::cos(error_value);
        ROS_WARN_COND(coeff <= 0 , "Warning, Something wrong with the damping coeff");
        return coeff; // a value between less than 1 
    }
}

std::vector<bot_utils::Index> bot_utils::bresenham_los(Index &src , Index& tgt)
{
    int Di = tgt.i - src.i;
    int Dj = tgt.j - src.j;
    int Dl;
    int Ds;
    int l;
    int s;
    int lf;
    int sf;
    std::function<std::pair<int,int>(int l , int s)> func;
    std::vector<bot_utils::Index> ray;
    ray.emplace_back(src);

    if (abs(Di) > abs(Dj))
    {
        Dl = Di;
        Ds = Dj;
        l = src.i;
        s = src.j;
        lf = tgt.i;
        sf = tgt.j;
        func = [=](int l , int s){return std::pair<int,int>(l,s);};
    }
    else
    {
        Dl = Dj;
        Ds = Di;
        l = src.j;
        s = src.i;
        lf = tgt.j;
        sf = tgt.i;
        func = [=](int l , int s){return std::pair<int,int>(s,l);};
    }

    int ds = sign(Ds);
    int dl = sign(Dl);
    int abs_Dl = std::abs(Dl);
    int d_esl = std::abs(Dl) * ds;
    int esl = 0;

    while(l != lf || s != sf)
    {
        l += dl;
        esl += Ds;
        if (2 * std::abs(esl) >= abs_Dl)
        {
            esl -= d_esl;
            s += ds;
        }
        auto [i,j] = func(l,s);
        bot_utils::Index pt(i,j);
        ray.emplace_back(pt);
    }
    return ray;
}


bot_utils::timeLogger::timeLogger()
{
    count_ = 0;
}

void bot_utils::timeLogger::start()
{
    start_ = clock_.now();
    count_++;
}
void bot_utils::timeLogger::stop()
{
    end_ = clock_.now();
    elapsed_time_ = end_ - start_;
    total_duration_ += elapsed_time_;
    ROS_INFO_STREAM("Time taken this loop: " << elapsed_time_.count()<<"ms");
    ROS_INFO_STREAM("Average loop time: " << total_duration_.count() / count_<<"ms");
}