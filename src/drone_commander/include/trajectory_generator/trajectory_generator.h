#ifndef HBOT__TRAJECTORY_GENERATOR_H
#define HBOT__TRAJECTORY_GENERATOR_H
#include "bot_utils/bot_utils.h"
#include "bot_utils/spline_data.h"
#include "mission_states.h"
#include <vector>
#include <functional>
#include <eigen3/Eigen/Dense>
#include <iostream>
class TrajectoryGenerator
{
private:
    double target_dt_;
    double average_speed_;
    std::string primary_traj_;
    double cruise_height_;
    double takeoff_height_;
    double land_height_;
    bool verbose_;

    std::function<std::vector<bot_utils::Pos3D>(bot_utils::Pos3D &pos_begin , 
                                                bot_utils::Pos3D &pos_end , 
                                                bot_utils::Pos3D &vel_begin , 
                                                bot_utils::Pos3D &vel_end)> SplineGenerator_;

public:
    TrajectoryGenerator(double target_dt , double average_speed, std::string primary_traj, double cruise_height, double takeoff_height, double land_height, bool verbose);

    void trajectory_handler(bot_utils::Pos3D current_goal , 
                            bot_utils::Pos3D next_goal , 
                            bot_utils::Pos3D h_pos , 
                            bot_utils::Pos3D h_vel ,
                            bot_utils::SplineData3D& hspline,
                            mission_states::HectorState h_state);

    std::vector<bot_utils::Pos3D> LinearVertTakeOff(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end);

    std::vector<bot_utils::Pos3D> LinearVertLand(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end);

    std::vector<bot_utils::Pos3D> LinearPlanar(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end);

    std::vector<bot_utils::Pos3D> Cubic(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end , bot_utils::Pos3D &vel_begin , bot_utils::Pos3D &vel_end);

    std::vector<bot_utils::Pos3D> Quintic(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end , bot_utils::Pos3D &vel_begin , bot_utils::Pos3D &vel_end);


};

#endif //HBOT__TRAJECTORY_GENERATOR_H