#include "djikstra.h"

Djikstra::Node::Node() 
    : g(0), h(0), visited(false), idx(-1, -1), parent(-1, -1) {}

Djikstra::FOpen::FOpen() 
    : f(0), idx(-1, -1) {}

Djikstra::FOpen::FOpen(double f, bot_utils::Index idx) 
    : f(f), idx(idx) {}

Djikstra::Djikstra(bot_utils::MapData & map): start(-1,-1) , goal(-1,-1), map_(map) , nodes(map.map_size_.i * map.map_size_.j), open_list()
{
    // write the nodes' indices
    int k = 0;
    for (int i = 0; i < map_.map_size_.i; ++i)
    {
        for (int j = 0; j < map_.map_size_.j; ++j)
        {
            nodes[k].idx.i = i;
            nodes[k].idx.j = j;
            ++k;
        }
    }    
}

bot_utils::Index Djikstra::plan(bot_utils::Index idx_start)
{
    std::vector<bot_utils::Index> path;
    
    //set up all our nodes

    for (Node & node : nodes)
    {
        node.h = 0;
        node.g = 1e5; 
        node.visited = false;
    }

    int k = flatten(idx_start);
    ROS_INFO("idx_start %d %d", idx_start.i, idx_start.j);
    Node * node = &(nodes[k]);
    node->g = 0;

    add_to_open(node);

    while (!open_list.empty())
    {
        // (1) poll node from open
        node = poll_from_open();

        // (2) check if node was visited, and mark it as visited
        if (node->visited)
        {   // if node was already visited ==> cheapest route already found, no point expanding this anymore
            continue; // go back to start of while loop, after checking if open list is empty
        }
        node->visited = true; // mark as visited, so the cheapest route to this node is found


        // (3) return path if node is a free cell
        if (checkCell(node -> idx))
        {   // reached the goal, return the path
            ROS_INFO("Replan next starting position");

            return node->idx;
        }

        // (4) check neighbors and add them if cheaper
        bool is_cardinal = true;
        for (int dir = 0; dir < 8; ++dir)
        {   // for each neighbor in the 8 directions

            // get their index
            bot_utils::Index & idx_nb_relative = NB_LUT[dir];
            bot_utils::Index idx_nb(
                node->idx.i + idx_nb_relative.i,
                node->idx.j + idx_nb_relative.j
            );

            double g_nb = node->g;
            // check if in map and accessible
            if (!checkCell(idx_nb))
            {   // if not, move to next nb
                //continue;
                int k = flatten(idx_nb);
                
                if (map_.grid_inflation_[k] > 0)
                    g_nb += 100;
                else if (map_.grid_logodds_[k] > map_.lo_thresh_)
                    g_nb += 1000; ; // in map, not inflated, and log odds occupied

            }
            else {
            // get the cost if accessing from node as parent
            // double g_nb = node->g;
            if (is_cardinal) 
                g_nb += 1;
            else
                g_nb += M_SQRT2;
            }
            // the above if else can be condensed using ternary statements: g_nb += is_cardinal ? 1 : M_SQRT2;

            // compare the cost to any previous costs. If cheaper, mark the node as the parent
            int nb_k = flatten(idx_nb);
            Node & nb_node = nodes[nb_k]; // use reference so changing nb_node changes nodes[k]
            if (nb_node.g > g_nb + 1e-5)
            {   // previous cost was more expensive, rewrite with current
                nb_node.g = g_nb;
                nb_node.parent = node->idx;

                // add to open
                add_to_open(&nb_node); // & a reference means getting the pointer (address) to the reference's object.
            }

            // toggle is_cardinal
            is_cardinal = !is_cardinal;
        }
    }
    // clear open list
    open_list.clear();
    return node->idx;   
}


bot_utils::Pos2D Djikstra::plan(bot_utils::Pos2D pos_start)
{
    bot_utils::Index path_idx = plan(pos2idx(pos_start));
    return idx2pos(path_idx);
}


bot_utils::Index Djikstra::pos2idx(bot_utils::Pos2D &pos)
{
    int i = round((pos.x - map_.origin_.x) / map_.cell_size_);
    int j = round((pos.y - map_.origin_.y) / map_.cell_size_);
    return bot_utils::Index(i,j);
}

bot_utils::Pos2D Djikstra::idx2pos(bot_utils::Index &idx)
{
    double x = idx.i * map_.cell_size_ + map_.origin_.x;
    double y = idx.j * map_.cell_size_ + map_.origin_.y;
    return bot_utils::Pos2D(x,y); 
}

bool Djikstra::oob(bot_utils::Index &idx)
{
    return (idx.i <=0 || idx.i >= map_.map_size_.i || idx.j < 0 && idx.j >= map_.map_size_.j);
}

bool Djikstra::oob(bot_utils::Pos2D &pos)
{
    bot_utils::Index idx = pos2idx(pos);
    return oob(idx);
}

int Djikstra::flatten(bot_utils::Index &idx)
{
    int flat = idx.i * map_.map_size_.j + idx.j;
    return flat;
}

bool Djikstra::checkCell(bot_utils::Index &idx)
{
    if (oob(idx))
    {
        return false;
    }

    int k = flatten(idx);

    if (map_.grid_inflation_.at(k) > 0)
    {
        return false; //in map, inflated
    }

    else if (map_.grid_logodds_.at(k) > map_.lo_thresh_)
    {
        return false; //not inflated, lo occuped
    }

    else
    {
        return true;
    }
}

bool Djikstra::checkCell(bot_utils::Pos2D &pos)
{
    bot_utils::Index idx = pos2idx(pos);
    return checkCell(idx);
}


void Djikstra::add_to_open(Node * node)
{   // sort node into the open list
    double node_f = node->g + node->h;

    for (int n = 0; n < open_list.size(); ++n)
    {
        FOpen & open_node = open_list[n];
        if (open_node.f > node_f + 1e-5)
        {   // the current node in open is guaranteed to be more expensive than the node to be inserted ==> insert at current location            
            open_list.emplace(open_list.begin() + n, node_f, node->idx);

            // emplace is equivalent to the below but more efficient:
            // Open new_open_node = Open(node_f, node->idx);
            // open_list.insert(open_list.begin() + n, new_open_node);
            return;
        }
    }
    // at this point, either open_list is empty or node_f is more expensive that all open nodes in the open list
    open_list.emplace_back(node_f, node->idx);
}


Djikstra::Node * Djikstra::poll_from_open()
{   
    bot_utils::Index &idx = open_list.front().idx; //ref is faster than copy
    int k = flatten(idx);
    Node *node = &(nodes[k]);
    open_list.pop_front();

    return node;
}