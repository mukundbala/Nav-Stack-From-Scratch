#ifndef TBOT__MAP_DATA_H
#define TBOT__MAP_DATA_H
#include <vector>
#include "bot_utils/bot_utils.h"
struct MapData
{
    std::vector<int> grid_inflation_;
    std::vector<int> grid_logodds_;
    int lo_thresh_;
    int lo_cap_;
    bot_utils::Pos2D origin_; //origin of grid 0,0 in global frame  ok
    bot_utils::Pos2D pos_min_; //minimum position the map can take ok 
    bot_utils::Pos2D pos_max_; //maximum position the map can take ok
    bot_utils::Index map_size_; //number of grids in length x number of grids in width ok 
    double cell_size_; //cell_size_ x cell_size_ ok 
    int total_cells_; //the total number of cells (map_size.i * map_size.j) ok
};

#endif //#ifndef TBOT__MAP_DATA_H
#define TBOT__MAP_DATA_H