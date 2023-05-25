#include "djikstra.h"

Djikstra::Djikstra():GridPlannerCore()
{
    ROS_INFO("[Djikstra]: Djikstra ready for prep");
}

Djikstra::Djikstra(std::string cost_mode, bot_utils::MapData &map_data):
GridPlannerCore(cost_mode,map_data)
{
    ROS_INFO("[Djikstra]: Djikstra Ready");
}

Djikstra::~Djikstra()
{
    ROS_INFO("[Djikstra]: Destroying Djikstra");
}

std::vector<bot_utils::Pos2D> Djikstra::plan(bot_utils::Pos2D idx_start, bot_utils::Pos2D idx_goal , bot_utils::MapData &map_data)
{
    auto id_start = pos2idx(idx_start);
    auto id_goal = pos2idx(idx_goal);
    
    return plan(id_start , id_goal , map_data);
}

std::vector<bot_utils::Pos2D> Djikstra::plan(bot_utils::Index idx_start, bot_utils::Index idx_goal , bot_utils::MapData &map_data)
{
    //path in indices
    std::vector<bot_utils::Index> path_idx;

    //assigning start and end indexes to the planners internal store of this data
    start_ = idx_start;
    goal_ = idx_goal;

    //reset all h and g costs for all the nodes
    for (Node &node : nodes)
    {
        node.h = 0;
        node.g = 1e5; // a reasonably large number. You can use infinity in clims as well, but clims is not included
        node.visited = false;
    }

    //get the 1D flattened key of the start index
    int k = flatten(start_);

    //Retrieve a pointer to the start node from our node store and update the g cost to 0 as the start node gcost = 0
    Node * node = &(nodes[k]);
    node->g = 0;

    //push the start node into the OpenList
    pushToOpenList(node);

    //we loop until we empty our open_list, at which point no path seems to be available
    while(open_list_.getSizeQ() != 0)
    {   
        //pop the node from the open_list
        node = popFromOpenList();

        //skip this node if it has been visited
        if (node->visited)
        {   
            continue; 
        }

        //if it is not visited, set it to visited
        node->visited = true;

        //check if this node is our goal
        if (node->idx.i == goal_.i && node->idx.j == goal_.j)
        {   
            path_idx.push_back(node->idx);
            ROS_INFO("[Djikstra]: Found Goal!");
            ROS_INFO("[Djikstra]: Backtracking to Start...");

            //backtrack
            while (node->idx.i != start_.i || node->idx.j != start_.j)
            {   
                k = flatten(node->parent);
                node = &(nodes[k]);
                
                path_idx.push_back(node->idx);
                
            }
            ROS_INFO("[Djikstra]: Found Path");
            break;
        }

        bool is_cardinal = true;
        for (int dir = 0; dir < 8; ++dir)
        {   
            bot_utils::Index &idx_nb_relative = NB_LUT[dir];
            bot_utils::Index idx_nb(
                node->idx.i + idx_nb_relative.i,
                node->idx.j + idx_nb_relative.j
            );

            //TestPos will return false if the index is on an inflated/occupied/oob cell. Skip neibor if false
            if (!testPos(idx_nb , map_data))
            {
                
                continue;
            }

            double g_nb = node->g;

            g_nb += is_cardinal ? 1 : M_SQRT2;

            int nb_k = flatten(idx_nb);

            Node & nb_node = nodes[nb_k]; 

            if (nb_node.g > g_nb + 1e-5)
            {   
                nb_node.g = g_nb;
                nb_node.parent = node->idx;
                pushToOpenList(&nb_node);
            }

            is_cardinal = !is_cardinal;
        }
    }
    //nuke the open list
    open_list_.clearQ();

    if (path_idx.empty())
    {
        std::vector<bot_utils::Pos2D> path_world;
        auto pos = idx2pos(start_);
        path_world.push_back(pos);
        return path_world;
    }
    
    //create a path_world vector and allocate space to speed up push backs
    std::vector<bot_utils::Pos2D> path_world;
    path_world.reserve(path_idx.size());

    for (auto& id:path_idx)
    {
        auto pos = idx2pos(id);
        path_world.push_back(pos);
    }

    return path_world;
}

std::vector<bot_utils::Pos2D> Djikstra::post_process_path(std::vector<bot_utils::Pos2D>& raw_path , bot_utils::MapData &map_data)
{
    if (raw_path.size() <= 2)
    {   
        ROS_INFO("[Djikstra]: Path only contains 2 points or less. Returning Path!");
        return raw_path;  
    }

    ROS_INFO("[Djikstra]: Sparsifying");
    std::vector<bot_utils::Pos2D> sparse_path = sparsifyPath(raw_path);

    ROS_INFO("[Djikstra]: Making Any Angle!");
    std::vector<bot_utils::Pos2D> any_angle_path = makeAnyAnglePath(sparse_path , map_data);

    ROS_INFO("[Djikstra]: Returning Final Path");
    return any_angle_path;
}

bot_utils::Pos2D Djikstra::find_better_point(bot_utils::Index idx_bad , bot_utils::MapData &map_data)
{
    ROS_INFO_STREAM("Clearing Q. Open List Size :" << open_list_.getSizeQ());
    open_list_.clearQ();
    ROS_INFO_STREAM("Q Cleared. Open List Size :" << open_list_.getSizeQ());

    //initialise g and h costs for all nodes, and set their visited to false
    for (Node &node : nodes)
    {
        node.h = 0;
        node.g = 1e5; // a reasonably large number. You can use infinity in clims as well, but clims is not included
        node.visited = false;
    }
    //set the start index as the bad index
    start_ = idx_bad;
    
    //get the 1D flattened key of the start index
    int k = flatten(start_);

    //Retrieve a pointer to the start node from our node store and update the g cost to 0 as the start node gcost = 0
    Node * node = &(nodes[k]);
    node->g = 0;

    //push the start node into the OpenList
    pushToOpenList(node);

    while(open_list_.getSizeQ() != 0)
    {
        //pop the node from the open_list
        node = popFromOpenList();

        //skip this node if it has been visited
        if (node->visited)
        {   
            continue; 
        }

        //if it is not visited, set it to visited
        node->visited = true;
    
        if (testPos(node -> idx , map_data))
        {   // reached the goal, return the path
            //ROS_INFO("[GlobalPlanner - Djikstra]: Generating new point");
            auto better_point_pos = idx2pos(node->idx);
            ROS_INFO("[DjiktraFallback]: Better Point Found! Returning");
            return better_point_pos;
        }

        bool is_cardinal = true;
        for (int dir = 0; dir < 8; ++dir)
        {   
            bot_utils::Index &idx_nb_relative = NB_LUT[dir];
            bot_utils::Index idx_nb(
                node->idx.i + idx_nb_relative.i,
                node->idx.j + idx_nb_relative.j
            );

            if (oob(idx_nb))
            {
                //we skip if the index is out of bounds
                continue;
            }

            double g_nb = node->g;

            if (!testPos(idx_nb , map_data))
            {   // if not, move to next nb
                //continue;
                int k = flatten(idx_nb);
                
                if (map_data.grid_inflation_[k] > 0)
                {
                    //in map,inflated
                    g_nb += 100;
                }
                    
                else if (map_data.grid_logodds_[k] > map_data.lo_thresh_)
                {
                    // in map, not inflated, and log odds occupied
                    g_nb += 1000; 
                }    
            }

            else 
            {
                g_nb += is_cardinal ? 1 : M_SQRT2;
            }

            int nb_k = flatten(idx_nb);

            Node & nb_node = nodes[nb_k];

            // previous cost was more expensive, rewrite with current
            if (nb_node.g > g_nb + 1e-5)
            {   
                nb_node.g = g_nb;
                nb_node.parent = node->idx;
                pushToOpenList(&nb_node); // & a reference means getting the pointer (address) to the reference's object.
            }

            // toggle is_cardinal
            is_cardinal = !is_cardinal;
        }
    }
    open_list_.clearQ();
    //I really shouldnt be here
    ROS_WARN("[DjiktraFallback]: Unable to find better point");
    return idx2pos(idx_bad);
}

bot_utils::Pos2D Djikstra::find_better_point(bot_utils::Pos2D pos_bad , bot_utils::MapData &map_data)
{
    auto idx_bad = pos2idx(pos_bad);
    ROS_INFO_STREAM("IDX BAD" << idx_bad.i << "," << idx_bad.j);
    return find_better_point(idx_bad , map_data);
}