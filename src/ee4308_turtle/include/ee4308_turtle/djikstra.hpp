#include "ros/ros.h"
#include "grid.hpp"
#include "common.hpp"
#include <vector>
#include <deque>

#ifndef TBOT__Djikstra_H
#define TBOT__Djikstra_H

class Djikstra
{
public:
    
    struct Node
    {
        double g;
        bool visited;
        Index idx;
        Index parent;
        Node();
    };

    struct GOpen
    {
        double g;
        Index idx;
        GOpen();
        GOpen(double g, Index idx);
    };

    Index start, goal;
    Grid & grid; 
    Djikstra(Grid &grid);

    std::vector<Index> plan(Index idx_start, Index idx_goal);

    std::vector<Position> plan(Position pos_start, Position pos_goal);

    Index e_plan(Index idx_curr, Index idx_bad_goal);

    //Insertion sort algorithm to sort nodes in GOpen list based on ascending g-cost
    void insertionSort(std::deque<GOpen> &list, Node *node);
    //Insertion sort algorithm to sort neighbouring cells based on ascending g-cost
    void insertionSort(std::vector<Node> &list, Node *node);

private:
        std::vector<Node> nodes; // keeps a record of the cheapest cost of every cell in the grid, as well as their parents
        std::deque<GOpen> open_list; // List to sort the unvisited cells based on their g-cost
        Index NB_LUT[8] = {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};

        void add_to_open(Node * node);
        Node * poll_from_open();
};



#endif //TBOT__Djikstra_H