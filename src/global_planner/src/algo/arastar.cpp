#include "arastar.h"

ARAstar::ARAstar():GridPlannerCore()
{
    ROS_INFO("[GridPlannerCore - ARAstar]: ARAstar ready for prep");
}

ARAstar::ARAstar(std::string cost_mode, bot_utils::MapData &map_data):
GridPlannerCore(cost_mode,map_data)
{
    ROS_INFO("[GridPlannerCore - ARAstar]: ARAstar Ready");
}

ARAstar::~ARAstar()
{
    ROS_INFO("[ARAstar]: Destroying ARAstar");
}

std::vector<bot_utils::Pos2D> ARAstar::plan(bot_utils::Pos2D idx_start, bot_utils::Pos2D idx_goal , bot_utils::MapData &map_data)
{
    auto id_start = pos2idx(idx_start);
    auto id_goal = pos2idx(idx_goal);
    
    return plan(id_start , id_goal , map_data);
}

std::vector<bot_utils::Pos2D> ARAstar::plan(bot_utils::Index idx_start, bot_utils::Index idx_goal , bot_utils::MapData &map_data)
{   
    /*
    This is the first planning step of ARA start that quickly generates
    a suboptimal path and returns it.
    */

    //path in indices
    std::vector<bot_utils::Index> path_idx;
    
    //assigning start and end indexes to the planners internal store of this data
    start_ = idx_start;
    goal_ = idx_goal;

    //reset epsilon
    e_ = e_init;
    //nuke the open list
    open_list_.clearQ();
    ROS_WARN("[GlobalPlanner]: Nuking OpenList!");
    /*
    * This is the init step of ARAStar:
      1. Resets each node in our map in preparation for a NEW planning sequence
        - Computes all the h-costs of the nodes to the goal
        - Sets all g cost of all non-start nodes to infinity. In practice, 1e5 is fine
        - Sets g cost of start node to 0
        - Sets node state to UNVISITED. This means that OPENSET = CLOSEDSET = INCONS = NULL SET
        - Inserts the start node into the OpenList, and sets its node state to OPEN
    */

    for (Node &node : nodes)
    {
        node.h = bot_utils::dist_oct(node.idx, goal_);
        node.g = 1e5; // a reasonably large number.
        node.state = NodeState::UNVISITED;
    }
    
    //get the 1D flattened key of the start index
    int k = flatten(start_);

    //Retrieve a pointer to the start node from our node store and update the g cost to 0 as the start node gcost = 0
    Node * node = &(nodes[k]);
    node->g = 0;

    //Set the node state as OPEN before pushing
    node->state = NodeState::OPEN;

    //push the start node into the openList
    pushToOpenList(node,e_);
    
    //first call to improve path finds the e' suboptimal path
    while (true)
    {
        
        //check the smallest value in the min heap
        auto min_f_val = open_list_.getTop();

        //Get the key of the goal
        int k_goal = flatten(goal_);

        //Extract and g and h cost from the goal node
        double goal_g = nodes[k_goal].g;
        double goal_h = nodes[k_goal].h;

        //f(s) = g(s) + e_*h(s)
        double goal_f = goal_g + e_ * goal_h;

        //break if the f score of goal is greater than the smallest in the minheap
        if (min_f_val >= goal_f)
        {
            break;
        }

        //pop the node from the openList, and set it to closed
        node = popFromOpenList();
        node->state = NodeState::CLOSED;

        //at this point, the node variable is the one we just popped

        //loop over the neighbors
        bool is_cardinal = true;
        for (int dir = 0 ; dir < 8 ; ++dir)
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

            //Compute the new g cost of the neibor
            /*
            If Cardinal Neibour --> add 1 to the current cost
            Otherwise --> add c , where c= sqrt(1**2 + 1**2) = SQRT(2)
            */
            double new_g_nb = is_cardinal ? node->g + 1 : node->g + M_SQRT2;

            //get the key of the neibor
            int k_nb = flatten(idx_nb);

            //Extract the reference to the neibor, and the g cost
            Node &nb_node = nodes[k_nb];
            double curr_g_nb = nb_node.g;

            
            if (new_g_nb < curr_g_nb)
            {
                //update the neibors cost
                nb_node.g = new_g_nb;

                //set the neibors cost to the parent node which expanded it
                nb_node.parent = node->idx;

                //if the neibor is closed, we send him to the Inconsistency Set
                if (nb_node.state == NodeState::CLOSED)
                {
                    nb_node.state == NodeState::INCONS;
                }

                else
                {
                    nb_node.state = NodeState::OPEN;
                    pushToOpenList(&nb_node,e_);
                    
                }
            }
            is_cardinal = !is_cardinal;
        }
        
    }
    ROS_INFO("Extracting");
    path_idx = extractPath(goal_);
    ROS_INFO("Extracted");
    //update epsilon
    updateEpsilon();

    ROS_INFO("[ARAstar]: Found Initial Path");

    if (path_idx.empty())
    {
        ROS_INFO("[ARAstar]: Path Empty!");
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

std::vector<bot_utils::Pos2D> ARAstar::path_refinement(bot_utils::MapData &map_data)
{
    if (e_ <= 1) //if it is less than one, no need to refine
    {
        std::vector<bot_utils::Pos2D> empty_pth;
        bot_utils::Pos2D dummy(-1,-1);
        empty_pth.push_back(dummy);
        return empty_pth;

    }
    else //Do Refinement
    {
        //reduce epsilon
        e_ -= e_delta;
        //clamp e to 0.99 so that refinement will stop
        if (e_<=1){e_ = 0.99;}
        ROS_WARN_STREAM("[ARAStar]: Contracted e_: " << e_);

        //move all states from INCONS to OPEN
        for (auto &nd : nodes)
        {
            //empty the CLOSED set to allow for a new search iteration
            if (nd.state == NodeState::CLOSED)
            {
                nd.state = NodeState::UNVISITED;
            }

            //move all the INCONS states into Open!
            else if (nd.state == NodeState::INCONS)
            {
                Node *incon = &nd;
                pushToOpenList(incon,e_);
            }
        }

        Node * node;
        while (true)
        {
            
            //check the smallest value in the min heap
            auto min_f_val = open_list_.getTop();

            //Get the key of the goal
            int k_goal = flatten(goal_);

            //Extract and g and h cost from the goal node
            double goal_g = nodes[k_goal].g;
            double goal_h = nodes[k_goal].h;

            //f(s) = g(s) + e_*h(s)
            double goal_f = goal_g + e_ * goal_h;

            //break if the f score of goal is less than the smallest in the minheap
            if (min_f_val >= goal_f)
            {
                ROS_INFO("[ARAStar]: Refined Path!");
                break;
            }

            //pop the node from the openList, and set it to closed
            node = popFromOpenList();
            node->state = NodeState::CLOSED;

            //at this point, the node variable is the one we just popped

            //loop over the neighbors
            bool is_cardinal = true;
            for (int dir = 0 ; dir < 8 ; ++dir)
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

                //Compute the new g cost of the neibor
                /*
                If Cardinal Neibour --> add 1 to the current cost
                Otherwise --> add c , where c= sqrt(1**2 + 1**2) = SQRT(2)
                */
                double new_g_nb = is_cardinal ? node->g + 1 : node->g + M_SQRT2;

                //get the key of the neibor
                int k_nb = flatten(idx_nb);

                //Extract the reference to the neibor, and the g cost
                Node &nb_node = nodes[k_nb];
                double curr_g_nb = nb_node.g;

                
                if (new_g_nb < curr_g_nb)
                {
                    //update the neibors cost
                    nb_node.g = new_g_nb;

                    //set the neibors cost to the parent node which expanded it
                    nb_node.parent = node->idx;

                    //if the neibor is closed, we send him to the Inconsistency Set
                    if (nb_node.state == NodeState::CLOSED)
                    {
                        nb_node.state == NodeState::INCONS;
                    }

                    else
                    {
                        nb_node.state = NodeState::OPEN;
                        pushToOpenList(&nb_node,e_);
                        
                    }
                }
                is_cardinal = !is_cardinal;
            }
            
        }//Broke From While Loop
            std::vector<bot_utils::Index> path_idx;
            ROS_INFO("Extracting Refined Path");
            path_idx = extractPath(goal_);
            ROS_INFO("Extracted Refined Path");
            //update epsilon
            // updateEpsilon();

            ROS_INFO("[ARAstar]: Found Initial Path");

            if (path_idx.empty())
            {
                ROS_INFO("[ARAstar]: Path Empty!");
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
}


std::vector<bot_utils::Index> ARAstar::extractPath(bot_utils::Index &par)
{
    //start from the parent
    std::vector<bot_utils::Index> pthidx;
    pthidx.push_back(par);

    auto curr = par;
    int counter = 1;
    while (curr.i != start_.i || curr.j != start_.j)
    {
        auto flat = flatten(curr);
        curr = nodes[flat].parent;
        pthidx.push_back(curr);
        counter++;
    }
    ROS_INFO_STREAM("RAW PATH HAS: " << counter);
    return pthidx;
}


void ARAstar::updateEpsilon()
{   
    //the minimum f(s) = g(s) + h(s) 
    double upper_bound = 1e5;
    double min_OPEN = 1e5;
    double min_INCONS = 1e5;

    for (auto& node: nodes)
    {
        if (node.state == NodeState::OPEN)
        {
            double f_cost = node.g + node.h;
            if (f_cost < min_OPEN)
            {
                min_OPEN = f_cost;
            }
        }

        if (node.state == NodeState::INCONS)
        {
            double f_cost = node.g + node.h;
            if (f_cost < min_INCONS)
            {
                min_INCONS = f_cost;
            }
        }
    }
    //The minimum value of both sets
    double min_set = std::min(min_OPEN,min_INCONS);
    
    //Minimum of minset and upper_bound
    upper_bound = std::min(upper_bound,min_set);
    
    //get the gcost of the goal
    auto k_goal = flatten(goal_);
    double g_goal = nodes[k_goal].g;
    
    double decay_limit = g_goal / upper_bound;
    ROS_WARN_STREAM("GOAL G-COST: " << g_goal);
    ROS_WARN_STREAM("UPPER BOUND: " << upper_bound);
    ROS_WARN_STREAM("DECAY_LIMIT: " << decay_limit);
    /*
      * If the decay limit is less than e_, set decayed_e 
        to m
      * Else, just set it back to e_

    */
    ROS_WARN_STREAM("Original e_: " << e_);
    double decayed_e =  std::min(e_,decay_limit);
    ROS_WARN_STREAM("DECAYING TO: " << decayed_e);
    
    e_ = decayed_e;

    ROS_WARN_COND(e_<=1,"Warning!Decay limit is <=1");

    if (e_<=1){e_ = 1.01;}
}

std::vector<bot_utils::Pos2D> ARAstar::post_process_path(std::vector<bot_utils::Pos2D>& raw_path , bot_utils::MapData &map_data)
{
    if (raw_path.size() <= 2)
    {   
        ROS_INFO("[ARAstar]: Path only contains 2 points or less. Returning Path!");
        return raw_path;  
    }
    //ROS_INFO("[ARAstar]: Sparsifying");
    std::vector<bot_utils::Pos2D> sparse_path = sparsifyPath(raw_path);

    //ROS_INFO("[ARAstar]: Making Any Angle!");
    std::vector<bot_utils::Pos2D> any_angle_path = makeAnyAnglePath(sparse_path , map_data);

    ROS_INFO("[ARAstar]: Returning Final Path");
    return any_angle_path;
}