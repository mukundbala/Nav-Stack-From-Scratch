#ifndef TBOT__GRID_PLANNER_CORE_H
#define TBOT__GRID_PLANNER_CORE_H

#include "bot_utils/bot_utils.h"
#include "bot_utils/map_data.h"
#include "ros/ros.h"

#include <queue>
#include <vector>
#include <functional>
#include <string>
#include <iostream>
#include <stdexcept>

/*
GridPlannerCore is the core interface that will be inherited by all grid based planners
like Astar,Djikstra,ThetaStar etc
*/

class GridPlannerCore
{

//##############Protected Data and Functions######################
protected:

    struct Node
    {
        double g, h;
        bool visited;
        bot_utils::Index idx;
        bot_utils::Index parent;
        Node();
    };

    struct OpenNode
    {
        double f;
        double g;
        bot_utils::Index idx;

        OpenNode();

        OpenNode(int f_arg ,int g_arg ,int i_arg ,int j_arg);

        OpenNode(int f_arg ,int g_arg , bot_utils::Index& idx_arg);
    };

    //#############################OpenList#############################
    class OpenList
    {
    private:

        struct f_g_comp
        {
            bool operator()(const OpenNode& nd_1 , const OpenNode& nd_2);

        };

        struct f_comp
        {
            bool operator()(const OpenNode& nd_1 , const OpenNode& nd_2);

        };

        struct g_comp
        {
            bool operator()(const OpenNode& nd_1 , const OpenNode& nd_2);
        };

        OpenNode popFrontFG();

        OpenNode popFrontF();

        OpenNode popFrontG();

        void pushFG(OpenNode node);

        void pushF(OpenNode node);

        void pushG(OpenNode node);

        void clearQFG();

        void clearQF();

        void clearQG();

        void printFG();

        void printF();

        void printG();


        //The priority queue data structure
        std::priority_queue<GridPlannerCore::OpenNode , std::vector<GridPlannerCore::OpenNode>, f_g_comp> pq_fg;

        std::priority_queue<GridPlannerCore::OpenNode , std::vector<GridPlannerCore::OpenNode>, f_comp> pq_f;

        std::priority_queue<GridPlannerCore::OpenNode , std::vector<GridPlannerCore::OpenNode>, g_comp> pq_g;
        
        //polymorphic objects 
        std::function<GridPlannerCore::OpenNode()> pop_front_callable;

        std::function<void(GridPlannerCore::OpenNode node_to_add)> push_callable;

        std::function<void()> print_callable;

        std::function<void()> clearQ_callable;

        //sort_by f only , g only, or f then g
        std::string cost_mode_;

        //readiness
        bool openlist_ready_;

        //size
        int size_;

    public:
        OpenList();

        OpenList(std::string &cost_mode);

        void setMode(std::string &cost_mode);

        GridPlannerCore::OpenNode popNode();

        void pushNode(GridPlannerCore::OpenNode node);

        void printQ();

        int getSizeQ();

        void clearQ();
    };
    //#############################OpenList#############################
    
    //Protected constructors to enforce usage as interface
    GridPlannerCore();

    GridPlannerCore(std::string cost_mode, bot_utils::MapData &map_data);

    //Pure Planning Interface Methods that derived planners must implement
    
    virtual std::vector<bot_utils::Pos2D> plan(bot_utils::Index idx_start, bot_utils::Index idx_goal , bot_utils::MapData &map_data) = 0;

    virtual std::vector<bot_utils::Pos2D> plan(bot_utils::Pos2D pos_start, bot_utils::Pos2D pos_goal , bot_utils::MapData &map_data) = 0;

    virtual std::vector<bot_utils::Pos2D> post_process_path(std::vector<bot_utils::Pos2D>& raw_path , bot_utils::MapData &map_data) = 0;

    //openlist push
    void pushToOpenList(Node *node);

    void pushToOpenList(Node *node, double inflation_factor);

    //openlist pop
    Node* popFromOpenList();
    
    //Post Processing Methods. 2 are given here
    std::vector<bot_utils::Pos2D> sparsifyPath(std::vector<bot_utils::Pos2D> &path_world);

    std::vector<bot_utils::Pos2D> makeAnyAnglePath(std::vector<bot_utils::Pos2D> &path_turning_points, bot_utils::MapData &map_data);

    /*
        Supporting Utility Functions
    */
    int flatten(bot_utils::Index &idx);

    bot_utils::Index pos2idx(bot_utils::Pos2D &pos);

    bot_utils::Pos2D idx2pos(bot_utils::Index &idx);

    bool oob(bot_utils::Pos2D &pos);

    bool oob(bot_utils::Index &idx);

    bool testPos(bot_utils::Pos2D &pos, bot_utils::MapData &map_data);

    bool testPos(bot_utils::Index &idx, bot_utils::MapData &map_data);

    bool isLos(bot_utils::Index &idx_src, bot_utils::Index &idx_end,bot_utils::MapData &map_data);

    bool isLos(bot_utils::Pos2D &pos_src, bot_utils::Pos2D &pos_end,bot_utils::MapData &map_data);

    //A record of all nodes (grids) on the map with their associated costs.
    std::vector<GridPlannerCore::Node> nodes;

    //Start Index
    bot_utils::Index start_;
    
    //Goal Index
    bot_utils::Index goal_;

    //Open List min-heap implementation that sorts based on f or g or f then g costs
    OpenList open_list_;

    //Neighbor mask
    bot_utils::Index NB_LUT[8] = {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};


    int map_height_;

    int map_width_;

    double cell_size_;

    bot_utils::Pos2D map_origin_;

    std::string cost_mode_;
//##############Protected Data and Functions######################

//##############Public Data and Functions#########################
public:

    /*
     * Protected methods plan(.) and post_process_path(.) must be implemented by derived classes.
     * Once this is done, generatePath(.) will automatically call plan(.) and post_process_path(.) sequentially 

    */

    //Virtual Destructor
    virtual ~GridPlannerCore();

    void prepPlanner(std::string cost_mode, bot_utils::MapData &map_data);

    std::vector<bot_utils::Pos2D> generatePath(bot_utils::Index &idx_start, bot_utils::Index &idx_goal , bot_utils::MapData &map_data);

    std::vector<bot_utils::Pos2D> generatePath(bot_utils::Pos2D &pos_start, bot_utils::Pos2D &pos_goal , bot_utils::MapData &map_data);

};
//##############Public Data and Functions#########################


#endif //TBOT__GRID_PLANNER_CORE_H