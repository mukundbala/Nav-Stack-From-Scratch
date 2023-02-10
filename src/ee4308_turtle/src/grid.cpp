#include "grid.hpp"

// constructor
Grid::Grid(Position pos_min, Position pos_max, double cell_size, double inflation_radius, int log_odds_thresh, int log_odds_cap)
    : size(round((pos_max.x - pos_min.x) / cell_size), round((pos_max.y - pos_min.y)) / cell_size),
      origin(pos_min), cell_size(cell_size), log_odds_thresh(log_odds_thresh), log_odds_cap(log_odds_cap)
{
    // log_odds_thresh and log_odds_cap must both be less than 100
    grid_log_odds = std::vector<int>(size.i * size.j, 0);  // initialise grid_log_odds of size (size_i) with zero values
    grid_inflation = std::vector<int>(size.i * size.j, 0); // initialise grid_inflation with size (...) with false values
    for (int i = 0; i < 360; ++i)
        DEG2RAD_LUT[i] = M_PI * i / 180.;
    
    generate_mask_inflation(inflation_radius);
    //
}
int Grid::get_key(Index idx)
{ // combine idx.i and idx.j into one number for more efficient access
    return idx.i * size.j + idx.j;
}
bool Grid::get_cell(Index idx)
{ // returns true if accessible, false otherwise
    if (out_of_map(idx))
        return false; // out of map

    int k = get_key(idx);

    if (grid_inflation[k] > 0)
        return false; // in map, and is inflated
    else if (grid_log_odds[k] > log_odds_thresh)
        return false; // in map, not inflated, and log odds occupied
    else
        return true; // in map, not inflated, and is log odds free
}
bool Grid::get_cell(Position pos)
{
    return get_cell(pos2idx(pos));
}

void Grid::generate_mask_inflation(double inflation_radius)
{
    mask_inflation.clear();
    double num_cells_in_radius = round(inflation_radius / cell_size);
    double n2 = inflation_radius * inflation_radius / cell_size / cell_size;

    for (double i = -num_cells_in_radius; i <= num_cells_in_radius; ++i)
    {
        for (double j = -num_cells_in_radius; j <= num_cells_in_radius; ++j)
        {
            if (i*i + j*j <= n2)
            {                                      // centre of cell at (i,j) is <= inflation_radius away from (0, 0)
                mask_inflation.emplace_back(i, j); // emplace means use constructor from Index --> Index(i, j)
            }
        }
    }
}

void Grid::change_inflation(bool inflate, Index idx)
{ // modify inflation count on surrounding cells of (i, j);

    int inc = inflate ? 1 : -1;

    for (int m = 0; m < mask_inflation.size(); ++m)
    {
        Index nb(
            mask_inflation[m].i + idx.i,
            mask_inflation[m].j + idx.j);

        if (out_of_map(nb))
            continue; // skip (i,j) that is out of map

        int nb_k = get_key(nb);
        grid_inflation[nb_k] += inc;
    }
}

bool Grid::out_of_map(Index idx)
{
    return (idx.i <= 0 || idx.i >= size.i || idx.j < 0 && idx.j >= size.j);
}

void Grid::change_log_odds(bool occupy, Index idx)
{

    if (out_of_map(idx))
        return; // ignore points that are out of map

    int inc = occupy ? 1 : -1; // 1 if observed to be occupied, -1 otherwise

    int k = get_key(idx);
    int prev_log_odds = grid_log_odds[k]; // copy of previous log odds value

    if (occupy && prev_log_odds < log_odds_cap)
    { // add 1 if less than cap and cell is observed to be occupied
        ++grid_log_odds[k];
    }
    else if (!occupy && prev_log_odds > -log_odds_cap)
    { // subtract 1 if more than -ve cap and cell is observed to be free
        --grid_log_odds[k];
    }

    if (prev_log_odds < log_odds_thresh && grid_log_odds[k] >= log_odds_thresh)
    {                                // unknown log odds becomes occupied log odds
        change_inflation(true, idx); // add inflation due to new occupied status at cell (i, j)
    }
    else if (prev_log_odds >= log_odds_thresh && grid_log_odds[k] < log_odds_thresh)
    {                                 // occupied log odds becomes unknown log odds
        change_inflation(false, idx); // remove inflation from previous occupied status at cell (i, j)
    }
}

void Grid::update(Position pos_rbt, double ang_rbt, const std::vector<float> &ranges)
{
    Index idx_rbt = pos2idx(pos_rbt); // idx of robot, derived from its pos

    for (int deg = 0; deg < 360; ++deg)
    {
        double rad = DEG2RAD_LUT[deg] + ang_rbt;
        double range = ranges[deg];

        // check if any thing visible along ray
        bool no_obstacles_visible = range > MAX_SCAN_RANGE; // max scan range derived from LaserScan message

        // limit range for correct calculations
        if (no_obstacles_visible)
            range = MAX_SCAN_RANGE; // the range will be infinity in sim; or some value higher than 3.5 in real situations.

        // get idx and pos at the edge of the scan
        Position pos_edge(
            pos_rbt.x + range * cos(rad),
            pos_rbt.y + range * sin(rad));
        Index idx_edge = pos2idx(pos_edge);

        // iterate over the ray using the line algorithms (los)
        std::vector<Index> ray = los.get(idx_rbt, idx_edge);

        if (no_obstacles_visible)
        { // all cells to edge are observed to be free
            for (Index &idx : ray)
            {
                change_log_odds(false, idx);
            }
        }
        else
        { // all cells to edge are free, but cell on edge is observed to be occupied
            // (1) for all cells that are not on the edge, set as free
            for (int m = 0; m < ray.size() - 1; ++m)
            {
                Index idx = ray[m];
                change_log_odds(false, idx);
            }
            // (2) for the cell on teh edge, set as occupied
            change_log_odds(true, ray.back()); //path.back() is path[path.size()-1]
        }
    }
}
Index Grid::pos2idx(Position pos)
{
    return Index(
        round((pos.x - origin.x) / cell_size), //i
        round((pos.y - origin.y) / cell_size)  //j
    );
}

Position Grid::idx2pos(Index idx)
{
    return Position(
        idx.i * cell_size + origin.x, //x
        idx.j * cell_size + origin.y  //y
    );
}

void Grid::write_to_msg(nav_msgs::OccupancyGrid &msg_grid_lo, nav_msgs::OccupancyGrid & msg_grid_inf)
{ // make sure msg_grid.data is already rsized to correct size --> msg_grid.data.size() == size.i * size.j
    for (int k = 0; k < grid_log_odds.size(); ++k)
    {
        if (grid_inflation[k] > 0) // is inflated
            msg_grid_inf.data[k] = std::min(grid_inflation[k] - 1, 3);
        else
            msg_grid_inf.data[k] = -1;
        msg_grid_lo.data[k] = (((double) grid_log_odds[k]) / log_odds_cap + 1) * 50;
    }
}