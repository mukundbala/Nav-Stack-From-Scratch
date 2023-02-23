#include "trajectory.hpp"

std::vector<Position> post_process(std::vector<Position> path, Grid &grid) // returns the turning points
{
    // LOS los;
    // (1) obtain turning points
    if (path.size() <= 2)
    { // path contains 0 to elements. Nothing to process
        return path;
    }

    // add path[0] (goal) to turning_points
    std::vector<Position> turning_points = {path.front()}; 
    // add intermediate turning points
    for (int n = 2; n < path.size(); ++n)
    {
        Position &pos_next = path[n];
        Position &pos_cur = path[n - 1];
        Position &pos_prev = path[n - 2];

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
    // add path[path.size()-1] (start) to turning_points
    turning_points.push_back(path.back());

    std::vector<Position> post_process_path;
    post_process_path.push_back(turning_points.front()); //the front of the array is th end
    int x = 0;
    Index curr = grid.pos2idx(post_process_path.at(x));

    for (int i = 1 ; i < turning_points.size()-1 ; ++i)
    {
        Index to_test = grid.pos2idx(turning_points.at(i+1));
        std::vector<Index> ray = bresenham_los(curr , to_test);
        bool islos = true;

        for (auto &id : ray)
        {
            if (!grid.get_cell(id))
            {
                islos = false;
                break;
            }
        }
        if (!islos)
        {
            x++;
            post_process_path.push_back(turning_points.at(i));
            curr = grid.pos2idx(post_process_path.at(x));
        }
    }
    post_process_path.push_back(turning_points.back());
    // for (Position &p : post_process_path)
    // {
    //     if (grid.get_cell(p))
    //     {
    //         ROS_WARN("POST PROCESS PATH CUTS THROUGH OBSTACLES!");
    //     }
    // }
    return post_process_path;
}

std::vector<Position> generate_velocities(std::vector<Position>& path , double initial_vel , double initial_rbt_angle , double average_speed)
{
    //Index 0 is the goal , Last index is the start position. Left of the vector is later points, right of the vector is prev points
    //Velocities must always point towards the next point. So , it must always be Next Point - Prev Point for the right direction

    // std::vector<Position> velocities;
    double initial_vel_x = initial_vel * std::cos(initial_rbt_angle);
    double initial_vel_y = initial_vel * std::sin(initial_rbt_angle);
    Position initial_vel_vec(initial_vel_x , initial_vel_y);
    std::vector<Position> velocities = {initial_vel_vec};
    // for (int m = 1 ; m < path.size() ; ++m) maybe not the best method to generate velocities...
    // {
    //     Position &turn_pt_next = path[m - 1];
    //     Position &turn_pt_cur = path[m];

    //     //ROS_INFO_STREAM("Next: (" << turn_pt_next.x << "," << turn_pt_next.y<<")");
    //     //ROS_INFO_STREAM("Curr: (" << turn_pt_cur.x << "," << turn_pt_cur.y<<")");
    //     Position dir_with_mag = turn_pt_cur - turn_pt_next;
    //     //ROS_INFO_STREAM("Dir with mag: (" << dir_with_mag.x << "," << dir_with_mag.y << ")");
    //     //ROS_INFO_STREAM("Magnitude: " << dir_with_mag.mag());
    //     Position dir_unit = dir_with_mag.unit_vec();
    //     //ROS_INFO_STREAM("Dir Unit: (" << dir_unit.x << "," << dir_unit.y << ")");
    //     Position final_velocity = dir_unit * average_speed;
    //     //ROS_INFO_STREAM("Final :" << final_velocity.x << "," << final_velocity.y <<")");
    //     velocities.push_back(final_velocity);
    // }
    for (int i = 1 ; i < path.size() - 1 ; ++i)
    {
        Position dir = path.at(i+1) - path.at(i-1);
        double velocity_heading = atan2(dir.y , dir.x);
        Position vel(average_speed * std::cos(velocity_heading), average_speed * std::sin(velocity_heading));
        velocities.push_back(vel);
    }
    velocities.push_back(Position(0,0)); //we place in an empty velocity at the end
    // velocities.push_back(initial_vel_vec);
    return velocities;
}
std::vector<Position> generate_trajectory(Position pos_begin, Position pos_end, Position vel_begin , Position vel_end , double average_speed, double target_dt ,  Grid & grid)
{
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double d = sqrt(Dx * Dx + Dy * Dy) / average_speed;
    //maybe?
    vel_end = vel_begin.avg(vel_end);
    //maybe?
    std::vector<Position> traj = {pos_begin};

    double M[6][6] = { {1 , 0 , 0 , 0 , 0 , 0},
                       {0 , 1 , 0 , 0 , 0 , 0},
                       {0 , 0 , 0.5, 0 , 0 , 0},
                       {-10.0 / (d*d*d) , -6.0 / (d*d) , -3.0 / (2.0 * d) , 10.0 / (d*d*d) , -4.0 / (d*d) , 1.0 / (2.0 * d)},
                       {15.0 / (d*d*d*d) , 8.0 / (d*d*d) , 3.0 / (2.0 * (d*d)) , -15.0 / (d*d*d*d) , 7.0 / (d*d*d), -1.0 / (d*d)},
                       {-6.0 / (d*d*d*d*d) , -3.0 / (d*d*d*d) , -1.0 / (2.0 * (d*d*d)) , 6.0 / (d*d*d*d*d) , -3.0 / (d*d*d*d), 1.0 / (2.0 * (d*d*d))}
                     };
    
    std::array<double,6> in_x = {pos_begin.x , vel_begin.x , 0 , pos_end.x , vel_end.x , 0};
    std::array<double,6> in_y = {pos_begin.y, vel_begin.y , 0 , pos_end.y , vel_end.y , 0};

    std::array<double,6> ax = {0 , 0 , 0 , 0 , 0 , 0};
    std::array<double,6> by = {0 , 0 , 0 , 0 , 0 , 0};

    for (int i = 0 ; i < 6 ; ++i)
    {
        for (int j = 0 ; j < 6; ++j)
        {
            ax[i] += (M[i][j] * in_x[j]);
            by[i] += (M[i][j] * in_y[j]);
        }
    }

    for (double time = target_dt ; time < d ; time += target_dt)
    {
        double px = ax[0] + ax[1]*time + ax[2]*time*time + ax[3]*time*time*time + ax[4]*time*time*time*time + ax[5]*time*time*time*time*time;
        double py = by[0] + by[1]*time + by[2]*time*time + by[3]*time*time*time + by[4]*time*time*time*time + by[5]*time*time*time*time*time;
        // if (!grid.get_cell(Position(px,py))) maybe an optimization?
        // {
        //     break;
        // }
        traj.emplace_back(px,py);
        // traj.emplace_back(
        //     pos_begin.x + Dx*time / d,
        //     pos_begin.y + Dy*time / d
        // );
        
    }
    return traj;
}

bool is_safe_trajectory(std::vector<Position> trajectory, Grid & grid)
{   // returns true if the entire path is accessible; false otherwise
    if (trajectory.size() == 0)
    {   // no path
        return false; 
    } 
    else if (trajectory.size() == 1)
    {   // goal == start
        return grid.get_cell(trajectory.front()); // depends on the only cell in the path
    }

    // if there are more than one turning points. Trajectory must be fine enough.
    for (int n=1; n<trajectory.size(); ++n)
    {
        if (!grid.get_cell(trajectory[n]))
            return false;
        /* // Use this if the trajectory points are not fine enough (distance > cell_size)
        Index idx_src = grid.pos2idx(trajectory[n-1]);
        Index idx_tgt = grid.pos2idx(trajectory[n]);
        
        grid.los.reset(idx_src, idx_tgt); // interpolate a straight line between points; can do away with los if points are fine enough.
        Index idx = idx_src;
        while (idx.i != idx_tgt.i || idx.j != idx_tgt.j)
        {
            if (!grid.get_cell(idx))
            {
                return false;
            }
            idx = grid.los.next();
        }
        */
    }
    return true;
}


Index pos2idx(Position pos , Position &pos_min , double cell_size)
{
    return Index(
        round((pos.x - pos_min.x) / cell_size), //i
        round((pos.y - pos_min.y) / cell_size)  //j
    );
}