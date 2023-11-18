#ifndef TBOT__INFLATEDASTAR_H
#define TBOT__INFLATEDASTAR_H
#include "bot_utils/bot_utils.h"
#include "grid_planner_core.h"
#include <array>
#include <deque>
#include <vector>

class InflatedAstar : public GridPlannerCore
{
private:

    std::vector<bot_utils::Pos2D> plan(bot_utils::Index idx_start, bot_utils::Index idx_goal , bot_utils::MapData &map_data) override;
    
    std::vector<bot_utils::Pos2D> plan(bot_utils::Pos2D pos_start, bot_utils::Pos2D pos_goal , bot_utils::MapData &map_data) override;

    std::vector<bot_utils::Pos2D> post_process_path(std::vector<bot_utils::Pos2D>& raw_path , bot_utils::MapData &map_data) override;
    
    std::vector<bot_utils::Pos2D> path_refinement(bot_utils::MapData &map_data);
    
    //Inflation Factor, following paper's nomenclature of EPSILON
    double e_ = 2.8;
public:

    InflatedAstar();

    InflatedAstar(std::string cost_mode, bot_utils::MapData &map_data);

    ~InflatedAstar() override;
};

#endif //TBOT__INFLATEDASTAR_H