#ifndef H_TRAJECTORY_HPP
#define H_TRAJECTORY_HPP
#include "bot_utils/bot_utils.h"
#include <vector>
#define NaN std::numeric_limits<double>::quiet_NaN()

struct SplineData2D
{
    std::vector<bot_utils::Pos2D> spline;
    int curr_spline_id = -1;
    double avg_speed = NaN;
    double target_dt = NaN;
    int get_num_targets();

    int find_pos_id(bot_utils::Pos2D &pos);

};

struct SplineData3D
{
    std::vector<bot_utils::Pos3D> spline;
    int curr_spline_id = -1;
    double avg_speed = NaN;
    double target_dt = NaN;
    int get_num_targets();

    int find_pos_id(bot_utils::Pos3D &pos);
};


std::pair<bot_utils::Pos2D,int> get_best_goal(SplineData2D &tspline , bot_utils::Pos3D &hector_pos , bot_utils::Pos2D &turtle_pos,  double h_avg_speed);


std::vector<bot_utils::Pos3D> LinearVert(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end);

std::vector<bot_utils::Pos3D> LinearPlanar(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end , double average_speed , double target_dt , double height , double close_enuf);

std::vector<bot_utils::Pos3D> Cubic(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end , bot_utils::Pos3D &vel_begin , bot_utils::Pos3D &vel_end , double average_speed , double target_dt , double height , double close_enuf);

std::vector<bot_utils::Pos3D> Quintic(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end , bot_utils::Pos3D &vel_begin , bot_utils::Pos3D &vel_end);

void TrajectoryGenerationHandler(bot_utils::Pos3D current_goal , 
                                 bot_utils::Pos3D next_goal , 
                                 bot_utils::Pos3D h_pos , 
                                 bot_utils::Pos3D h_vel,
                                 SplineData3D &tspline,
                                 double average_speed , 
                                 double target_dt , 
                                 double height ,
                                 double close_enuf,
                                 int hector_state , int goal_state);



#endif //H_TRAJECTORY_HPP