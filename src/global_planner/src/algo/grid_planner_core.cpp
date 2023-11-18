#include "grid_planner_core.h"

//########################GridPlannerCore Protected#################//

///////////////////////////////////////////
/*
//Node Struct//
 * Node struct stores and tracks the indices of a node, its parent and f and g costs
 * Node constructor initialises g,h costs to 0 and index to (-1,-1)
*/

GridPlannerCore::Node::Node():
g(0),
h(0),
idx(-1 , -1),
parent(-1,-1) {};
/////////////////EndNode////////////////////

////////////////////////////////////////////
/*
//OpenNode Struct//
 * OpenNode struct stores the indices of a node that has become "open" and its costs
 * OpenNode constructor initialises g,h costs to 0 and index to (-1,-1)
*/
GridPlannerCore::OpenNode::OpenNode():
f(0),
g(0),
idx(-1 , -1){};

//Other OpenNode constructors
GridPlannerCore::OpenNode::OpenNode(int f_arg , int g_arg , int i_arg , int j_arg):
f(f_arg),
g(g_arg),
idx(i_arg,j_arg){};


GridPlannerCore::OpenNode::OpenNode(int f_arg , int g_arg , bot_utils::Index& idx_arg):
f(f_arg),
g(g_arg),
idx(idx_arg) {};
/////////////////EndOpenNode//////////////////////

//////////////////////////////////////////////////
/*
//OpenList//
 * This implementation of OpenList uses a min-heap to sort OpenNodes 
   either by fcost,gcost or fcost then gcost.
 * The choice of cost is loaded through a parameter file and passed into the OpenList's
   constructor. The appropriate comparator is polymorphically assigned to the OpenList
 * Push and Pop operations are both log(n)
*/

//private Openlist stuff
//f cost comparator
bool GridPlannerCore::OpenList::f_comp::operator()(const OpenNode& nd_1 , const OpenNode& nd_2)
{
    return nd_1.f > nd_2.f;
}

//g cost comparator
bool GridPlannerCore::OpenList::g_comp::operator()(const OpenNode& nd_1 , const OpenNode& nd_2)
{
    return nd_1.g > nd_2.g;
}

//f cost, then ties broken using g cost
bool GridPlannerCore::OpenList::f_g_comp::operator()(const OpenNode& nd_1 , const OpenNode& nd_2)
{
            
    if (nd_1.f == nd_2.f)
    {
        return nd_1.g > nd_2.g;
    }
    return nd_1.f > nd_2.f;
}

//popping from the OpenList that uses a fg sorting priority queue min-heap in log(n)
GridPlannerCore::OpenNode GridPlannerCore::OpenList::popFrontFG()
{
        
    OpenNode nd = pq_fg.top();
    pq_fg.pop();
    return nd;
        
}

//popping from the OpenList that uses a f sorting priority queue min-heap in log(n)
GridPlannerCore::OpenNode GridPlannerCore::OpenList::popFrontF()
{
        
    OpenNode nd = pq_f.top();
    pq_f.pop();
    return nd;
        
}

//popping from the OpenList that uses a g sorting priority queue min-heap in log(n)
GridPlannerCore::OpenNode GridPlannerCore::OpenList::popFrontG()
{
        
    OpenNode nd = pq_g.top();
    pq_g.pop();
    return nd;
}

//pushing to a OpenList that uses a fg sorting priority queue min-heap in log(n)
void GridPlannerCore::OpenList::pushFG(OpenNode node)
{
    pq_fg.push(node);
}

//pushing to a OpenList that uses a f sorting priority queue min-heap in log(n)
void GridPlannerCore::OpenList::pushF(OpenNode node)
{
    pq_f.push(node);
}

//pushing to a OpenList that uses a g sorting priority queue min-heap in log(n)
void GridPlannerCore::OpenList::pushG(OpenNode node)
{
    pq_g.push(node);
}

//clearing an OpenList that uses a fg sorting priority queue min-heap in O(n)
void GridPlannerCore::OpenList::clearQFG()
{
    std::priority_queue<OpenNode , std::vector<OpenNode>, f_g_comp> reset_pq_fg;
    pq_fg = reset_pq_fg;
}

//clearing an OpenList that uses a f sorting priority queue min-heap in O(n)
void GridPlannerCore::OpenList::clearQF()
{
    std::priority_queue<OpenNode , std::vector<OpenNode>, f_comp> reset_pq_f;
    pq_f = reset_pq_f;
}

//clearing an OpenList that uses a g sorting priority queue min-heap in O(n)
void GridPlannerCore::OpenList::clearQG()
{
    std::priority_queue<OpenNode , std::vector<OpenNode>, g_comp> reset_pq_g;
    pq_g = reset_pq_g;
}

//getting the f score at the top of the fg sorting priority queue min-heap in O(1)
double GridPlannerCore::OpenList::getFTopFG()
{
    return pq_fg.top().f;
}

//getting the f score at the top of the f sorting priority queue min-heap in O(1)
double GridPlannerCore::OpenList::getFTopF()
{
    return pq_f.top().f;
}

//getting the g score at the top of the g sorting priority queue min-heap in O(1)
double GridPlannerCore::OpenList::getGTopG()
{
    return pq_g.top().g;
}

//printing a fg OpenList in nlogn (use only for debugging) in nlogn
void GridPlannerCore::OpenList::printFG()
{

    std::priority_queue<OpenNode , std::vector<OpenNode>, f_g_comp> pq_fg_copy = pq_fg;
    
    while (!pq_fg_copy.empty())
    {
        OpenNode n = pq_fg_copy.top();
        std::cout<<"(" << n.f << "," << n.g <<" " << "[" << n.idx.i << "," << n.idx.j<<"]) ";
        pq_fg_copy.pop();
    }

    std::cout<<"\n";
}

//printing a f OpenList in nlogn (use only for debugging) in nlogn
void GridPlannerCore::OpenList::printF()
{

    std::priority_queue<OpenNode , std::vector<OpenNode>, f_comp> pq_f_copy = pq_f;
    
    while (!pq_f_copy.empty())
    {
        OpenNode n = pq_f_copy.top();
        std::cout<<"(" << n.f << "," << n.g <<" " << "[" << n.idx.i << "," << n.idx.j<<"]) ";
        pq_f_copy.pop();
    }

    std::cout<<"\n";
}

//printing a g OpenList in nlogn (use only for debugging) in nlogn
void GridPlannerCore::OpenList::printG()
{
    std::priority_queue<OpenNode , std::vector<OpenNode>, g_comp> pq_g_copy = pq_g;
    
    while (!pq_g_copy.empty())
    {
        OpenNode n = pq_g_copy.top();
        std::cout<<"(" << n.f << "," << n.g <<" " << "[" << n.idx.i << "," << n.idx.j<<"]) ";
        pq_g_copy.pop();
    }

    std::cout<<"\n";
}
//end private OpenList stuff

//public OpenList stuff
GridPlannerCore::OpenList::OpenList()
{
    size_ = 0;
    ROS_WARN("[GridPlannerCore]: OpenList requires cost_mode to initialise. Please call setMode function with a selected cost");
    openlist_ready_ = 0;
}

GridPlannerCore::OpenList::OpenList(std::string &cost_mode)
{
    this->cost_mode_ = cost_mode;

    if (cost_mode == "fg")
    {
        pop_front_callable = std::bind(&OpenList::popFrontFG , this);
        push_callable = std::bind(&OpenList::pushFG , this , std::placeholders::_1);
        print_callable = std::bind(&OpenList::printFG , this);
        clearQ_callable = std::bind(&OpenList::clearQFG , this);
        getTopVal_callable = std::bind(&OpenList::getFTopFG,this);
        ROS_INFO("[GridPlannerCore]: OpenList Ready. Using FG cost mode");
    }

    else if (cost_mode == "f")
    {
        pop_front_callable = std::bind(&OpenList::popFrontF , this);
        push_callable = std::bind(&OpenList::pushF , this , std::placeholders::_1);
        print_callable = std::bind(&OpenList::printF , this);
        clearQ_callable = std::bind(&OpenList::clearQF , this);
        getTopVal_callable = std::bind(&OpenList::getFTopF , this);
        ROS_INFO("[GridPlannerCore]: OpenList Ready. Using F cost mode");
    }

    else if (cost_mode == "g")
    {
        pop_front_callable = std::bind(&OpenList::popFrontG , this);
        push_callable = std::bind(&OpenList::pushG , this , std::placeholders::_1);
        print_callable = std::bind(&OpenList::printG , this);
        clearQ_callable = std::bind(&OpenList::clearQG, this);
        getTopVal_callable = std::bind(&OpenList::getGTopG, this);
        ROS_INFO("[GridPlannerCore]: OpenList Ready. Using G cost mode");
    }

    else
    {
        pop_front_callable = std::bind(&OpenList::popFrontFG , this);
        push_callable = std::bind(&OpenList::pushFG , this , std::placeholders::_1);
        print_callable = std::bind(&OpenList::printFG , this);
        clearQ_callable = std::bind(&OpenList::clearQFG , this);
        getTopVal_callable = std::bind(&OpenList::getFTopFG,this);
        ROS_INFO("[GridPlannerCore]: OpenList Ready. Using FG cost mode");
    }
    openlist_ready_ = 1;  
}

void GridPlannerCore::OpenList::setMode(std::string &cost_mode)
{
    this->cost_mode_ = cost_mode;

    if (cost_mode == "fg")
    {
        pop_front_callable = std::bind(&OpenList::popFrontFG , this);
        push_callable = std::bind(&OpenList::pushFG , this , std::placeholders::_1);
        print_callable = std::bind(&OpenList::printFG , this);
        clearQ_callable = std::bind(&OpenList::clearQFG , this);
        getTopVal_callable = std::bind(&OpenList::getFTopFG,this);
        ROS_INFO("[GridPlannerCore]: OpenList Ready. Using FG cost mode");
    }

    else if (cost_mode == "f")
    {
        pop_front_callable = std::bind(&OpenList::popFrontF , this);
        push_callable = std::bind(&OpenList::pushF , this , std::placeholders::_1);
        print_callable = std::bind(&OpenList::printF , this);
        clearQ_callable = std::bind(&OpenList::clearQF , this);
        getTopVal_callable = std::bind(&OpenList::getFTopF , this);
        ROS_INFO("[GridPlannerCore]: OpenList Ready. Using F cost mode");
    }

    else if (cost_mode == "g")
    {
        pop_front_callable = std::bind(&OpenList::popFrontG , this);
        push_callable = std::bind(&OpenList::pushG , this , std::placeholders::_1);
        print_callable = std::bind(&OpenList::printG , this);
        clearQ_callable = std::bind(&OpenList::clearQG, this);
        getTopVal_callable = std::bind(&OpenList::getGTopG, this);
        ROS_INFO("[GridPlannerCore]: OpenList Ready. Using G cost mode");
    }

    else
    {
        pop_front_callable = std::bind(&OpenList::popFrontFG , this);
        push_callable = std::bind(&OpenList::pushFG , this , std::placeholders::_1);
        print_callable = std::bind(&OpenList::printFG , this);
        clearQ_callable = std::bind(&OpenList::clearQFG , this);
        getTopVal_callable = std::bind(&OpenList::getFTopFG,this);
        ROS_INFO("[GridPlannerCore]: OpenList Ready. Using FG cost mode");
    }
    openlist_ready_ = 1;
}

//popNode function exposed to the user
GridPlannerCore::OpenNode GridPlannerCore::OpenList::popNode()
{
    if (!openlist_ready_){throw std::domain_error("OpenList Not Ready!");}

    if (size_ == 0)
    {
        if (!openlist_ready_){throw std::length_error("OpenList Empty!");}
    }

    size_--;
    return pop_front_callable();
}

//pushNode function exposed to the user
void GridPlannerCore::OpenList::pushNode(OpenNode node)
{
    if (!openlist_ready_){throw std::domain_error("OpenList Not Ready!");}

    push_callable(node);
    size_++;
}

double GridPlannerCore::OpenList::getTop()
{
    return getTopVal_callable();
}

//printQ function exposed to the user
void GridPlannerCore::OpenList::printQ()
{
    if (size_ == 0)
    {
        ROS_INFO("[GridPlannerCore]: OpenList is empty!");
    }

    else
    {
        print_callable();
    }
    
}

//get size function exposed to the user
int GridPlannerCore::OpenList::getSizeQ()
{
    if (size_ < 0)
    {
        throw std::range_error("Size less than 0!");
    }

    return size_;
}

//clear function exposed to the user
void GridPlannerCore::OpenList::clearQ()
{
    clearQ_callable();
}
////////////////////EndOpenList//////////////////////////////

GridPlannerCore::GridPlannerCore()
{
    cost_mode_ = "f";
    
    start_.setIdx(-1,-1);
    goal_.setIdx(-1,-1);

    open_list_.setMode(cost_mode_);

    ROS_INFO("[GridPlannerCore]: Base Planner Created");
}

GridPlannerCore::GridPlannerCore(std::string cost_mode, bot_utils::MapData &map_data)
{
    cost_mode_ = cost_mode;

    map_height_ = map_data.map_size_.i;
    map_width_ = map_data.map_size_.j;
    cell_size_ = map_data.cell_size_;
    map_origin_.setCoords(map_data.origin_.x , map_data.origin_.y);
    
    start_.setIdx(-1,-1);
    goal_.setIdx(-1,-1);

    nodes.reserve(map_width_ * map_height_);

    int k = 0;

    for (int i = 0 ; i < map_height_ ; ++i)
    {
        for (int j = 0 ; j < map_width_ ; ++j)
        {
            Node my_node;
            my_node.idx.i = i;
            my_node.idx.j = j;
            nodes.push_back(my_node);
        }
    }

    open_list_.setMode(cost_mode_);

    ROS_INFO("[GridPlannerCore]: Base Planner ready!");   
}

void GridPlannerCore::pushToOpenList(Node *node)
{
    double node_f = (cost_mode_ == "g") ? 0 : node->g + node->h;
    double node_g = node->g;
    OpenNode next_node(node_f , node_g , node->idx);
    open_list_.pushNode(next_node);
}

void GridPlannerCore::pushToOpenList(Node *node , double inflation_factor)
{
    double node_f = (cost_mode_ == "g") ? 0 : node->g + inflation_factor * node->h;
    double node_g = node->g;
    OpenNode next_node(node_f , node_g , node->idx);
    open_list_.pushNode(next_node);
}

GridPlannerCore::Node * GridPlannerCore::popFromOpenList()
{
    OpenNode open_nd = open_list_.popNode();
    bot_utils::Index& idx = open_nd.idx;
    int k = flatten(idx);

    Node * node = &(nodes[k]);
    return node;
}


int GridPlannerCore::flatten(bot_utils::Index &idx)
{
    return idx.i * this->map_width_ + idx.j;
}

bot_utils::Index GridPlannerCore::pos2idx(bot_utils::Pos2D &pos)
{
    int i = round((pos.x - map_origin_.x) / cell_size_);
    int j = round((pos.y - map_origin_.y) / cell_size_);
    return bot_utils::Index(i,j);
}

bot_utils::Pos2D GridPlannerCore::idx2pos(bot_utils::Index &idx)
{
    double x = idx.i * cell_size_ + map_origin_.x;
    double y = idx.j * cell_size_ + map_origin_.y;
    return bot_utils::Pos2D(x,y);
}

bool GridPlannerCore::oob(bot_utils::Index &idx)
{
    return (idx.i < 0 || idx.i >= map_height_ || idx.j < 0 && idx.j >= map_width_);
}

bool GridPlannerCore::oob(bot_utils::Pos2D &pos)
{
    auto idx = pos2idx(pos);
    return oob(idx);
}

bool GridPlannerCore::testPos(bot_utils::Index &idx , bot_utils::MapData &map_data)
{
    int key = flatten(idx);

    if (oob(idx))
    {
        return false; //test fail
    }

    if (map_data.grid_inflation_.at(key) > 0)
    {
        return false; //in map, but on inflated
    }
    else if (map_data.grid_logodds_.at(key) > map_data.lo_thresh_)
    {
        return false; //in map, not inflated but log odds occupied
    }
    else
    {
        return true; //in map, not inflated not lo occupied
    }
}

bool GridPlannerCore::testPos(bot_utils::Pos2D &pos , bot_utils::MapData &map_data)
{
    bot_utils::Index idx = pos2idx(pos);
    int key = flatten(idx);

    if (oob(idx))
    {
        return false; //test fail
    }

    if (map_data.grid_inflation_.at(key) > 0)
    {
        return false; //in map, but on inflated
    }
    else if (map_data.grid_logodds_.at(key) > map_data.lo_thresh_)
    {
        return false; //in map, not inflated but log odds occupied
    }
    else
    {
        return true; //in map, not inflated not lo occupied
    }
}

bool GridPlannerCore::isLos(bot_utils::Index &idx_src, bot_utils::Index &idx_end, bot_utils::MapData &map_data)
{
    auto ray = bot_utils::bresenham_los(idx_src , idx_end);

    for (auto &index : ray)
    {
        bool indexOk = testPos(index, map_data);

        if (!indexOk)
        {
            return false;
        }
    }

    return true;
}

bool GridPlannerCore::isLos(bot_utils::Pos2D &pos_src, bot_utils::Pos2D &pos_end,bot_utils::MapData &map_data)
{
    auto idx_src = pos2idx(pos_src);
    auto idx_end = pos2idx(pos_end);
    return isLos(idx_src , idx_end , map_data);
}


std::vector<bot_utils::Pos2D> GridPlannerCore::sparsifyPath(std::vector<bot_utils::Pos2D> &path_world)
{
    if (path_world.size() <= 2)
    {
        return path_world;
    }

    std::vector<bot_utils::Pos2D> turning_points = {path_world.front()};

    for (int n = 2; n < path_world.size(); ++n)
    {
        bot_utils::Pos2D &pos_next = path_world[n];
        bot_utils::Pos2D &pos_cur = path_world[n - 1];
        bot_utils::Pos2D &pos_prev = path_world[n - 2];

        double Dx_next = pos_next.x - pos_cur.x;
        double Dy_next = pos_next.y - pos_cur.y;
        double Dx_prev = pos_cur.x - pos_prev.x;
        double Dy_prev = pos_cur.y - pos_prev.y;

        // use 2D cross product to check if there is a turn around pos_cur
        if (abs(Dx_next * Dy_prev - Dy_next * Dx_prev) > 1e-5)
        {   // cross product is small enough ==> straight
            turning_points.push_back(pos_cur);
        }
    }
    turning_points.push_back(path_world.back());

    return turning_points;
}

std::vector<bot_utils::Pos2D> GridPlannerCore::makeAnyAnglePath(std::vector<bot_utils::Pos2D> &path_turning_points, bot_utils::MapData &map_data)
{
    std::vector<bot_utils::Pos2D> any_angle_path;
    any_angle_path.push_back(path_turning_points.front());
    int x = 0;
    bot_utils::Index curr = pos2idx(path_turning_points.at(x));

    for (int i = 1 ; i < path_turning_points.size() - 1 ; ++i)
    {
        bot_utils::Index to_test = pos2idx(path_turning_points.at(i+1));
        bool is_los_avail = isLos(curr , to_test , map_data);

        if (!is_los_avail)
        {
            x++;
            any_angle_path.push_back(path_turning_points.at(i));
            curr = pos2idx(any_angle_path.at(x));
        }
    }
    any_angle_path.push_back(path_turning_points.back());
    return any_angle_path;
}

//########################End GridPlannerCore Protected#################//

//########################GridPlannerCore Public########################//

GridPlannerCore::~GridPlannerCore()
{
    nodes.clear();
    ROS_INFO("[GridPlannerCore]: Destroying Base Planner");
}

void GridPlannerCore::prepPlanner(std::string cost_mode, bot_utils::MapData &map_data)
{
    cost_mode_ = cost_mode;

    map_height_ = map_data.map_size_.i;
    map_width_ = map_data.map_size_.j;
    cell_size_ = map_data.cell_size_;
    map_origin_.setCoords(map_data.origin_.x , map_data.origin_.y);
    
    start_.setIdx(-1,-1);
    goal_.setIdx(-1,-1);

    nodes.reserve(map_width_ * map_height_);

    int k = 0;

    for (int i = 0 ; i < map_height_ ; ++i)
    {
        for (int j = 0 ; j < map_width_ ; ++j)
        {
            Node my_node;
            my_node.idx.i = i;
            my_node.idx.j = j;
            nodes.push_back(my_node);
        }
    }

    open_list_.setMode(cost_mode_);

    ROS_INFO("[GridPlannerCore]: Base Planner ready!"); 
}


std::vector<bot_utils::Pos2D> GridPlannerCore::generatePath(bot_utils::Index &idx_start, bot_utils::Index &idx_goal , bot_utils::MapData &map_data)
{
    auto raw_path_world = plan(idx_start , idx_goal , map_data);
    auto final_path_world = post_process_path(raw_path_world , map_data);
    return final_path_world;
}

std::vector<bot_utils::Pos2D> GridPlannerCore::generatePath(bot_utils::Pos2D &pos_start , bot_utils::Pos2D &pos_goal , bot_utils::MapData &map_data)
{
    auto idx_start = pos2idx(pos_start);
    auto idx_end  = pos2idx(pos_goal);
    return generatePath(idx_start  , idx_end , map_data);
}

std::vector<bot_utils::Pos2D> GridPlannerCore::RefinePathRoutine(bot_utils::MapData &map_data)
{
    return path_refinement(map_data);
}