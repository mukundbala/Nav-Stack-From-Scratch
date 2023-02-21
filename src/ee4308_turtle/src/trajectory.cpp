#include "trajectory.hpp"

std::vector<Position> post_process(std::vector<Position> path, Grid &grid) // returns the turning points
{
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
        bool los = true;

        for (auto &id : ray)
        {
            if (!grid.get_cell(id))
            {
                los = false;
                break;
            }
        }
        if (!los)
        {
            x++;
            post_process_path.push_back(turning_points.at(i));
            curr = grid.pos2idx(post_process_path.at(x));
        }
    }
    post_process_path.push_back(turning_points.back());
    return post_process_path;
}

std::vector<Position> generate_trajectory(Position pos_begin, Position pos_end, double average_speed, double target_dt, Grid & grid, double robot_angle , double initial_vel)
{
    //This is a quntic hermite spline implemented to allow the robot to have smooth and continuous motion with little jerks
    double turning_velocity = 0.06; //this is the velocity magnitude of the spline at its end points when its going to turn
    double allowance = 0.05; //we use this to segment 2 splines into 5 chunks using this buffer allowance

    double x_buffer = allowance * std::cos(robot_angle);
    double y_buffer = allowance * std::sin(robot_angle);

    Position buffered_start(pos_begin.x + x_buffer , pos_begin.y + x_buffer); //our buffered start
    double buffer_heading1 = limit_angle(heading(pos_begin , buffered_start));
    ROS_WARN_COND(fabs(buffer_heading1) > M_PI/2, "Buffer heading not constrained!");

    std::vector<Position> trajectory = {pos_begin};
//###########################First Chunk#####################################
    //velocities
    double x_vel_i = initial_vel * std::cos(robot_angle);
    double y_vel_i = initial_vel * std::sin(robot_angle);
    double x_vel_f = turning_velocity * std::cos(buffer_heading1);
    double y_vel_f = turning_velocity * std::sin(buffer_heading1);
    double x_acc = 0;
    double y_acc = 0;

    //duration
    double Dx = buffered_start.x - pos_begin.x;
    double Dy = buffered_start.y - pos_begin.y;
    double duration = std::sqrt((Dx*Dx) + (Dy*Dy)) / average_speed;
    
    //coefficients of x
    double a0 = pos_begin.x;
    double a1 = x_vel_i;
    double a2 = 0.5 * x_acc;
    double a3 = (pos_begin.x * -10 / pow(duration, 3)) + (x_vel_i * -6 / pow(duration, 2))+  
                ((-3 / 2 * duration) * x_acc) + (10 / pow(duration, 3) * buffered_start.x)+ 
                (-4 / pow(duration, 2) * x_vel_f) + ((1 / 2 * duration) * x_acc);
    
    double a4 = (15 / pow(duration, 4) * pos_begin.x) + (8 / pow(duration, 3) * x_vel_i) + 
                (3 / 2 / pow(duration, 2) * x_acc) + (-15 / pow(duration, 4) * buffered_start.x) + 
                (7 / pow(duration, 3) * x_vel_f) + (-1 / pow(duration, 2) * x_acc);
    
    double a5 = (-6 / pow(duration, 5) * pos_begin.x) + (-3 / pow(duration, 4) * x_vel_i) + 
                (-1 / 2 / pow(duration, 3) * x_acc) + (6 / pow(duration, 5) *buffered_start.x) + 
                (-3 / pow(duration, 4) * x_vel_f) + (1 / 2 / pow(duration, 3) * x_acc);

    double b0 = pos_begin.y;
    double b1 = y_vel_i;
    double b2 = 0.5 * y_acc;
    double b3 = (pos_begin.y * -10 / pow(duration, 3)) + (y_vel_i * -6 / pow(duration, 2))+  
                ((-3 / 2 * duration) * y_acc) + (10 / pow(duration, 3) * buffered_start.y)+ 
                (-4 / pow(duration, 2) * y_vel_f) + ((1 / 2 * duration) * y_acc);
    
    double b4 = (15 / pow(duration, 4) * pos_begin.y) + (8 / pow(duration, 3) * y_vel_i) + 
                (3 / 2 / pow(duration, 2) * y_acc) + (-15 / pow(duration, 4) * buffered_start.y) + 
                (7 / pow(duration, 3) * y_vel_f) + (-1 / pow(duration, 2) * y_acc);
    
    double b5 = (-6 / pow(duration, 5) * pos_begin.y) + (-3 / pow(duration, 4) * y_vel_i) + 
                (-1 / 2 / pow(duration, 3) * y_acc) + (6 / pow(duration, 5) * buffered_start.y) + 
                (-3 / pow(duration, 4) * y_vel_f) + (1 / 2 / pow(duration, 3) * y_acc);

    for (double time = target_dt ; time < duration ; time += target_dt)
    {
        double x = a0 + a1 * time + a2 * pow(time, 2) + a3 * pow(time, 3) + a4 * pow(time, 4) + a5 * pow(time, 5);
        double y = b0 + b1 * time + b2 * pow(time, 2) + b3 * pow(time, 3) + b4 * pow(time, 4) + b5 * pow(time, 5);
        trajectory.emplace_back(x,y);
    }
    //###########################First Chunk Done#####################################

    double end_x = allowance * std::cos(buffer_heading1);
    double end_y = allowance * std::sin(buffer_heading1);
    Position buffered_end(pos_end.x - end_x , pos_end.y - end_y);
    double buffer_heading2 = limit_angle(heading(buffered_end, pos_end));

    //###########################Second Chunk#####################################
    x_vel_i = initial_vel * std::cos(buffer_heading1);
    y_vel_i = initial_vel * std::sin(buffer_heading1);
    x_vel_f = turning_velocity * std::cos(buffer_heading2);
    y_vel_f = turning_velocity * std::sin(buffer_heading2);
    Dx = buffered_end.x - buffered_start.x;
    Dy = buffered_end.y - buffered_start.y;
    duration = std::sqrt((Dx*Dx) + (Dy*Dy));

    //generate an interpolation based trajectory for this chunk
    // double time = target_dt;
    for (double time = target_dt;time<duration ; time+=target_dt)
    {
        double x = buffered_start.x + Dx * time / duration;
        double y = buffered_start.y + Dy * time / duration;
    }
    //###########################Second Chunk Done#####################################

    Dx = pos_end.x - buffered_end.x;
    Dy = pos_end.y - buffered_end.y;
    duration = std::sqrt((Dx*Dx) + (Dy*Dy));

    //###########################Third Chunk#####################################
    a0 = buffered_end.x;
    a1 = x_vel_i;
    a2 = 0.5 * x_acc;
    a3 = (-10 / pow(duration, 3) * buffered_end.x) + (-6 / pow(duration, 2) * x_vel_i) + 
         ((-3 / 2 * duration) * x_acc) + (10 / pow(duration, 3) * pos_end.x) + 
         (-4 / pow(duration, 2) * x_vel_f) + ((1 / 2 * duration) * x_acc);
    a4 = (15 / pow(duration, 4) * buffered_end.x) + (8 / pow(duration, 3) * x_vel_i) + 
         (3 / 2 / pow(duration, 2) * x_acc) + (-15 / pow(duration, 4) * pos_end.x) + 
         (7 / pow(duration, 3) * x_vel_i) + (-1 / pow(duration, 2) * x_acc);
    a5 = (-6 / pow(duration, 5) * buffered_end.x) + (-3 / pow(duration, 4) * x_vel_i) + 
         (-1 / 2 / pow(duration, 3) * x_acc) + (6 / pow(duration, 5) * pos_end.x) + 
         (-3 / pow(duration, 4) * x_vel_f) + (1 / 2 / pow(duration, 3) * x_acc);

    b0 = buffered_end.y;
    b1 = y_vel_i;
    b2 = 0.5 * y_acc;
    b3 = (-10 / pow(duration, 3) * buffered_end.y) + (-6 / pow(duration, 2) * y_vel_i) + 
         ((-3 / 2 * duration) * y_acc) + (10 / pow(duration, 3) * pos_end.y) + 
         (-4 / pow(duration, 2) *y_vel_f) + ((1 / 2 * duration) * y_acc);
    b4 = (15 / pow(duration, 4) * buffered_end.y) + (8 / pow(duration, 3) * y_vel_i) + 
         (3 / 2 / pow(duration, 2) * y_acc) + (-15 / pow(duration, 4) * pos_end.y) + 
         (7 / pow(duration, 3) * y_vel_i) + (-1 / pow(duration, 2) * y_acc);
    b5 = (-6 / pow(duration, 5) * buffered_end.y) + (-3 / pow(duration, 4) * y_vel_i) + 
         (-1 / 2 / pow(duration, 3) * y_acc) + (6 / pow(duration, 5) * pos_end.y) + 
         (-3 / pow(duration, 4) * y_vel_f) + (1 / 2 / pow(duration, 3) * y_acc);
    
    for (double time = target_dt ; time < duration ; time += target_dt)
    {
        double x = a0 + a1 * time + a2 * pow(time, 2) + a3 * pow(time, 3) + a4 * pow(time, 4) + a5 * pow(time, 5);
        double y = b0 + b1 * time + b2 * pow(time, 2) + b3 * pow(time, 3) + b4 * pow(time, 4) + b5 * pow(time, 5);
    }
    //###########################Third Chunk Done#####################################
    return trajectory;
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