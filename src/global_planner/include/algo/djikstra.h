#ifndef TBOT__DJIKSTRA_H
#define TBOT__DJIKSTRA_H

#include "bot_utils/bot_utils.h"
#include "grid_planner_core.h"
#include <array>
#include <deque>
#include <vector>


class Djikstra : public GridPlannerCore
{
private:

    std::vector<bot_utils::Pos2D> plan(bot_utils::Index idx_start, bot_utils::Index idx_goal , bot_utils::MapData &map_data) override;
    
    std::vector<bot_utils::Pos2D> plan(bot_utils::Pos2D pos_start, bot_utils::Pos2D pos_goal , bot_utils::MapData &map_data) override;

    std::vector<bot_utils::Pos2D> post_process_path(std::vector<bot_utils::Pos2D>& raw_path , bot_utils::MapData &map_data) override;

public:

    Djikstra();

    Djikstra(std::string cost_mode, bot_utils::MapData &map_data);

    ~Djikstra() override;

    bot_utils::Pos2D find_better_point(bot_utils::Index idx_bad , bot_utils::MapData &map_data);

    bot_utils::Pos2D find_better_point(bot_utils::Pos2D pos_bad , bot_utils::MapData &map_data);

};

#endif //TBOT__DJIKSTRA_H