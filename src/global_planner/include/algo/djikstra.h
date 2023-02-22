#ifndef TBOT__DJIKSTRA_H
#define TBOT__DJIKSTRA_H
#include "bot_utils/map_data.h"

class Djikstra
{
public:

    struct Node
    {
        double g, h;
        bool visited;
        bot_utils::Index idx;
        bot_utils::Index parent;
        Node();
    };
    struct FOpen
    {
        double f;
        bot_utils::Index idx;
        FOpen();
        FOpen(double f, bot_utils::Index idx);
    };

    Djikstra(bot_utils::MapData &map);

    bot_utils::Index start;

    bot_utils::Index goal;

    bot_utils::MapData &map_;

    bot_utils::Index plan(bot_utils::Index idx_start);

    bot_utils::Pos2D plan(bot_utils::Pos2D pos_start);

private:

    std::vector<Node> nodes; // keeps a record of the cheapest cost of every cell in the grid, as well as their parents
    
    std::deque<FOpen> open_list;

    bot_utils::Index NB_LUT[8] = {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}, {0,-1}, {1,-1}};

    void add_to_open(Node * node);

    Node * poll_from_open();


    bot_utils::Index pos2idx(bot_utils::Pos2D &pos);

    bot_utils::Pos2D idx2pos(bot_utils::Index &idx);

    bool oob(bot_utils::Index &idx);

    bool oob(bot_utils::Pos2D &pos);

    int flatten(bot_utils::Index &idx);

    bool checkCell(bot_utils::Index &idx);

    bool checkCell(bot_utils::Pos2D &pos);
};

#endif //TBOT__DJIKSTRA_H