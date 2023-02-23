#include "common.hpp"
#include "grid.hpp"
#include <algorithm>
#include <array>
#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
std::vector<Position> post_process(std::vector<Position> path, Grid &grid); // returns the turning points
std::vector<Position> generate_velocities(std::vector<Position>& path , double inital_vel , double initial_rbt_angle , double average_speed);
std::vector<Position> generate_trajectory(Position pos_begin, Position pos_end, Position vel_begin , Position vel_end , double average_speed, double target_dt , Grid & grid);
bool is_safe_trajectory(std::vector<Position> trajectory, Grid & grid);

#endif