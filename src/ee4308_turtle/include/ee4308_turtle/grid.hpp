#include "ros/ros.h"
#include <cmath>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include "los.hpp"
#include "common.hpp"
#include <nav_msgs/OccupancyGrid.h>

#ifndef GRID_HPP
#define GRID_HPP
class Grid
{
    private:
        std::vector<int> grid_log_odds; // signed char is int but 4 times smaller in memory
        int log_odds_thresh, log_odds_cap;
        std::vector<int> grid_inflation;
        Position origin;
        double cell_size;
        std::vector<Index> mask_inflation;
        double DEG2RAD_LUT[360];
        double MAX_SCAN_RANGE = 3.499999; 

        void generate_mask_inflation(double inflation_radius);
        void change_inflation(bool inflate, Index idx);
        void change_log_odds(bool occupy, Index idx);
    public:
        Index size;
        LOS los;

        Grid(Position pos_min, Position pos_max, double cell_size, double inflation_radius, int log_odds_thresh, int log_odds_cap);

        void update(Position pos, double ang_rbt, const std::vector<float> & ranges);
        bool out_of_map(Index idx);
        int get_key(Index idx);
        bool get_cell(Index idx); // for planners
        bool get_cell(Position pos);

        void write_to_msg(nav_msgs::OccupancyGrid &msg_grid_lo, nav_msgs::OccupancyGrid & msg_grid_inf);

        Index pos2idx(Position pos);
        Position idx2pos(Index idx);
};
#endif

