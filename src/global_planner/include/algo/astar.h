#ifndef TBOT__ASTAR_H
#define TBOT__ASTAR_H
#include "bot_utils/bot_utils.h"
#include "grid_planner_core.h"
#include <array>
#include <deque>
#include <vector>

class Astar : public GridPlannerCore
{
private:

    std::vector<bot_utils::Pos2D> plan(bot_utils::Index idx_start, bot_utils::Index idx_goal , bot_utils::MapData &map_data) override;
    
    std::vector<bot_utils::Pos2D> plan(bot_utils::Pos2D pos_start, bot_utils::Pos2D pos_goal , bot_utils::MapData &map_data) override;

    std::vector<bot_utils::Pos2D> post_process_path(std::vector<bot_utils::Pos2D>& raw_path , bot_utils::MapData &map_data) override;

public:

    Astar();

    Astar(std::string cost_mode, bot_utils::MapData &map_data);

    ~Astar() override;
};

#endif //TBOT__ASTAR_H