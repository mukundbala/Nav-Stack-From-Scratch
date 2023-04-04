#include "bot_utils/spline_data.h"




int bot_utils::SplineData2D::get_num_targets()
{
    return spline.size();
}

int bot_utils::SplineData2D::find_pos_id(bot_utils::Pos2D &pos) //this works
{
    double best_dist = 1e6;
    double best_id = -1;

    for (int i = 0 ; i < spline.size() ; ++i)
    {
        double dist = bot_utils::dist_euc(pos , spline.at(i));
        if (dist < best_dist)
        {
            best_dist = dist;
            best_id = i;
        }

        if (dist > best_dist) //an optimization. Once distance starts increasing, just stop looking
        {
            break;
        }
    }
    
    return best_id;
}

int bot_utils::SplineData3D::get_num_targets()
{
    return spline.size();
}


//this method assumes that the height for each target is the same!
int bot_utils::SplineData3D::find_pos_id(bot_utils::Pos3D &pos)
{
    double best_dist = 1e6;
    double best_id = -1;

    for (int i = 0 ; i < spline.size() ; ++i)
    {
        double dist = bot_utils::dist_euc(pos.x , pos.y , spline.at(i).x , spline.at(i).y);

        if (dist < best_dist)
        {
            best_dist = dist;
            best_id = i;
        }

        if (dist > best_dist)
        {
            break;
        }
    }

    return best_id;
}