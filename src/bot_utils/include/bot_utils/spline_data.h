#ifndef TBOT__SPLINE_DATA_H
#define TBOT__SPLINE_DATA_H

#include "bot_utils/bot_utils.h"
#include <vector>
#define NaN std::numeric_limits<double>::quiet_NaN()


namespace bot_utils
{
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
}

#endif //#ifndef TBOT__SPLINE_DATA_H

