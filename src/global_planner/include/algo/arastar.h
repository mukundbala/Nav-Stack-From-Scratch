#ifndef TBOT__ARASTAR_H
#define TBOT__ARASTAR_H
#include "bot_utils/bot_utils.h"
#include "grid_planner_core.h"
#include <array>
#include <deque>
#include <vector>

class ARAstar : public GridPlannerCore
{
private:

    std::vector<bot_utils::Pos2D> plan(bot_utils::Index idx_start, bot_utils::Index idx_goal , bot_utils::MapData &map_data) override;
    
    std::vector<bot_utils::Pos2D> plan(bot_utils::Pos2D pos_start, bot_utils::Pos2D pos_goal , bot_utils::MapData &map_data) override;

    std::vector<bot_utils::Pos2D> post_process_path(std::vector<bot_utils::Pos2D>& raw_path , bot_utils::MapData &map_data) override;

    std::vector<bot_utils::Pos2D> path_refinement(bot_utils::MapData &map_data) override;

    void updateEpsilon();

    std::vector<bot_utils::Index> extractPath(bot_utils::Index &par);

    //Inflation factor, following paper's nomenclature of EPSILON. Init values also similar to paper
    const double e_init = 4.0;
    const double e_delta = 0.4;
    double e_ = 4.0;

public:

    ARAstar();

    ARAstar(std::string cost_mode, bot_utils::MapData &map_data);



    ~ARAstar() override;
};

#endif //TBOT__ARASTAR_H