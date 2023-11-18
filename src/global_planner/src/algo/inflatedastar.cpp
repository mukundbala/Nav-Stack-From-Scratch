#include "inflatedastar.h"

InflatedAstar::InflatedAstar():GridPlannerCore()
{
    ROS_INFO("[GridPlannerCore - InflatedAstar]: InflatedAstar ready for prep");
}

InflatedAstar::InflatedAstar(std::string cost_mode,bot_utils::MapData &map_data):
GridPlannerCore(cost_mode,map_data)
{
    ROS_INFO_STREAM("[GridPlannerCore - InflatedAstar]: Using Inflation Factor: " << e_);
    ROS_INFO("[GridPlannerCore - InflatedAstar]: InflatedAstar Ready");
}

InflatedAstar::~InflatedAstar()
{
    ROS_INFO("[Astar]: Destroying InflatedAstar");
}

std::vector<bot_utils::Pos2D> InflatedAstar::plan(bot_utils::Pos2D idx_start, bot_utils::Pos2D idx_goal , bot_utils::MapData &map_data)
{
    auto id_start = pos2idx(idx_start);
    auto id_goal = pos2idx(idx_goal);
    
    return plan(id_start , id_goal , map_data);
}

std::vector<bot_utils::Pos2D> InflatedAstar::plan(bot_utils::Index idx_start, bot_utils::Index idx_goal , bot_utils::MapData &map_data)
{   
    //path in indices
    std::vector<bot_utils::Index> path_idx;

    //assigning start and end indexes to the planners internal store of this data
    start_ = idx_start;
    goal_ = idx_goal;

    //reset all h and g costs for all the nodes
    for (Node &node : nodes)
    {
        node.h = bot_utils::dist_oct(node.idx, goal_);
        node.g = 1e5; // a reasonably large number. You can use infinity in clims as well, but clims is not included
        node.visited = false;
        node.state = NodeState::UNVISITED;
    }

    //get the 1D flattened key of the start index
    int k = flatten(start_);

    //Retrieve a pointer to the start node from our node store and update the g cost to 0 as the start node gcost = 0
    Node * node = &(nodes[k]);
    node->g = 0;

    //Set to Open
    node->state = NodeState::OPEN;
    
    //push the start node into the OpenList
    pushToOpenList(node);

    while (open_list_.getSizeQ() != 0)
    {
        node = popFromOpenList();

        node->state = NodeState::CLOSED;

        //check if this node is our goal
        if (node->idx.i == goal_.i && node->idx.j == goal_.j)
        {   
            path_idx.push_back(node->idx);
            ROS_INFO("[InflatedAstar]: Found Goal!");
            ROS_INFO("[InflatedAstar]: Backtracking to Start...");

            //backtrack
            while (node->idx.i != start_.i || node->idx.j != start_.j)
            {   
                k = flatten(node->parent);
                node = &(nodes[k]);
                
                path_idx.push_back(node->idx);
                
            }
            ROS_INFO("[InflatedAstar]: Found Path");
            break;
        }

        //loop over the neighbors
        bool is_cardinal = true;
        for (int dir = 0; dir < 8; ++dir)
        {   
            bot_utils::Index &idx_nb_relative = NB_LUT[dir];
            bot_utils::Index idx_nb(
                node->idx.i + idx_nb_relative.i,
                node->idx.j + idx_nb_relative.j
            );
            
            if (!testPos(idx_nb , map_data))
            {
                //TestPos will return false if the index is on an inflated/occupied/oob cell. Skip it
                continue;
            }

            //get the g_cost of the neibor node
            double g_nb = node->g;

            //update the neibor that we go from the current node to the neibor
            g_nb += is_cardinal ? 1 : M_SQRT2;

            //get the key of the neibor
            int nb_k = flatten(idx_nb);

            //extract a reference to the neibor node from our Node store
            Node & nb_node = nodes[nb_k]; 

            //check if the gcost of our current knowledge of the neibor node is higher than what we just updated, we update our store
            if (nb_node.g > g_nb + 1e-5)
            {   
                nb_node.g = g_nb;
                nb_node.parent = node->idx;
                nb_node.state = NodeState::OPEN;
                pushToOpenList(&nb_node , e_);
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

std::vector<bot_utils::Pos2D> InflatedAstar::path_refinement(bot_utils::MapData &map_data)
{
    std::vector<bot_utils::Pos2D> empty;
    return empty;
}

std::vector<bot_utils::Pos2D> InflatedAstar::post_process_path(std::vector<bot_utils::Pos2D>& raw_path , bot_utils::MapData &map_data)
{
    if (raw_path.size() <= 2)
    {   
        ROS_INFO("[InflatedAstar]: Path only contains 2 points or less. Returning Path!");
        return raw_path;  
    }
    ROS_INFO("[InflatedAstar]: Sparsifying");
    std::vector<bot_utils::Pos2D> sparse_path = sparsifyPath(raw_path);

    ROS_INFO("[InflatedAstar]: Making Any Angle!");
    std::vector<bot_utils::Pos2D> any_angle_path = makeAnyAnglePath(sparse_path , map_data);

    ROS_INFO("[InflatedAstar]: Returning Final Path");
    return any_angle_path;
}
