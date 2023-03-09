#include "hector_trajectory.hpp"


int SplineData2D::get_num_targets()
{
    return spline.size();
}

int SplineData2D::find_pos_id(bot_utils::Pos2D &pos) //this works
{
    double best_dist = 1e6;
    double best_id = -1;

    for (int i = 0 ; i < spline.size() ; ++i)
    {
        double dist = bot_utils::dist_euc(pos , spline.at(i));
        if (dist < best_dist)
        {
            best_dist = dist;
            best_id = i;
        }

        if (dist > best_dist) //an optimization. Once distance starts increasing, just stop looking
        {
            break;
        }
    }

    return best_id;
}

int SplineData3D::get_num_targets()
{
    return spline.size();
}



std::pair<bot_utils::Pos2D,int> get_best_goal(SplineData2D &tspline , bot_utils::Pos3D &hector_pos , bot_utils::Pos2D &turtle_pos, double h_avg_speed)
{
    bot_utils::Pos2D h_planar_pos(hector_pos.x , hector_pos.y);
    std::pair<bot_utils::Pos2D,int> res;
    int tbot_id = tspline.find_pos_id(turtle_pos);
    assert(tbot_id != -1);

    double head_start = 3.0; //in seconds
    /*
    The idea here is that targets are generated at regular interval dt. So, from one target to another, it takes dt.
    Using this idea, we can estimate how long the turtlebot will take to go from current target to target n.
    The best target n is the one that the drone can reach as quickly as the turtlebot. There can be a few cases here

    1. The turtlebot can reach the last target in the spline faster than the drone can. In this case, we just return the last target in the spline and leave the state machine to figure it out
    2. We find a point n at which the drone can reach about as fast as the turtle bot. Return that point
    */
   double tbot_time = 0;

   for (int i = tbot_id ; i < tspline.spline.size() -1 ; ++i)
   {
        tbot_time += tspline.target_dt; //turtle will reach i to i+1 within target_dt

        double hector_dist = bot_utils::dist_euc(hector_pos.x , hector_pos.y , tspline.spline.at(i+1).x , tspline.spline.at(i+1).y); //the distance hector needs to fly to the i+1 th index

        double hector_time = hector_dist / h_avg_speed;

        if (tbot_time - hector_time >= 1.0)
        {
            ROS_INFO("Found a prediction!");
            res = {tspline.spline.at(i+1) , i+1};
            res.first.print();
            ROS_INFO_STREAM(res.second);
            ROS_INFO("###");
            return res;
        }
        //we continue searching otherwise
   }
   
   //if we came here without returning anything, this means we couldnt find a good enough time

   ROS_INFO("Returning last target##");
   res = {tspline.spline.at(tspline.spline.size()-1) , tspline.spline.size() - 1};
   res.first.print();
   ROS_INFO("###");
   return res; //return the last value in the spline
}

std::vector<bot_utils::Pos3D> LinearVert(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end)
{
    //we can just manually do this. Tbh its pretty much what trajectory generation does
    if (pos_begin.z > pos_end.z)
    {
        std::vector<bot_utils::Pos3D> segment_traj = {pos_begin};
        for (int i = 0 ; i < 3 ; ++ i)
        {
            segment_traj.emplace_back(pos_begin.x ,pos_begin.y , pos_begin.z - 0.5 * (i+1));
        }
        return segment_traj;
    }

    else
    {
        std::vector<bot_utils::Pos3D> segment_traj = {pos_begin};
        for (int i = 0 ; i < 3 ; ++ i)
        {
            segment_traj.emplace_back(pos_begin.x ,pos_begin.y , pos_begin.z + 0.5 * (i+1));
        }
        return segment_traj;
    }
    std::vector<bot_utils::Pos3D> segment_traj = {pos_begin};
    return segment_traj;
}

std::vector<bot_utils::Pos3D> LinearPlanar(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end , double average_speed , double target_dt , double height , double close_enuf)
{
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double d = sqrt ((Dx * Dx) + (Dy * Dy));

    std::vector<bot_utils::Pos3D> segment_traj = {pos_begin};

    for (double time = target_dt ; time < d ; time += target_dt)
    {
        double x = pos_begin.x + Dx * time / d;
        double y = pos_begin.y + Dy * time / d;
        segment_traj.emplace_back(x , y , height);
    }

    return segment_traj;
}

std::vector<bot_utils::Pos3D> Cubic(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end , bot_utils::Pos3D &vel_begin , bot_utils::Pos3D &vel_end , double average_speed , double target_dt , double height , double close_enuf)
{
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double d =  sqrt(Dx * Dx + Dy * Dy) / average_speed;

    std::vector<bot_utils::Pos3D> traj = {pos_begin};

    double M[4][4] = {{1.0 , 0 , 0 , 0},
                      {0 , 1.0 , 0 , 0},
                      {-3.0 / (d*d) , -2.0 / d , 3.0 / (d*d) , -1 / d},
                      {2 / (d*d*d) , 1 / (d*d) , -2 / (d*d*d) , 1 / (d*d)}
                     };

    std::array<double,4> in_x = {pos_begin.x , vel_begin.x , pos_end.x , vel_end.x};
    std::array<double,4> in_y = {pos_begin.y , vel_begin.y , pos_end.y , vel_end.y};

    std::array<double,4> ax = {0 , 0 , 0 , 0};
    std::array<double,4> by = {0 , 0 , 0 , 0};

    for (int i = 0 ; i < 4 ; ++i)
    {
        for (int j = 0 ; j < 4; ++j)
        {
            ax[i] += (M[i][j] * in_x[j]);
            by[i] += (M[i][j] * in_y[j]);
        }
    }

    for (double time = target_dt ; time < d ; time += target_dt)
    {
        double px = ax[0] + ax[1]*time + ax[2]*time*time + ax[3]*time*time*time;
        double py = by[0] + by[1]*time + by[2]*time*time + by[3]*time*time*time;
        traj.emplace_back(px,py,height);   
    }

    return traj;
}
//This handler will return trajectories in reverse order!
void TrajectoryGenerationHandler(bot_utils::Pos3D current_goal , 
                                 bot_utils::Pos3D next_goal , 
                                 bot_utils::Pos3D h_pos , 
                                 bot_utils::Pos3D h_vel,
                                 SplineData3D &tspline,
                                 double average_speed , 
                                 double target_dt , 
                                 double height ,
                                 double close_enuf,
                                 int hector_state , int goal_state)
{
    std::array<std::string_view,5> unpack_h_state = {"TAKEOFF" , "LAND" , "TURTLE" , "START" , "GOAL"};
    std::array<std::string_view,3> unpack_g_state = {"PREDICTION" , "CHASE" , "GOTO"};

    std::string_view h_state = unpack_h_state.at(hector_state);
    std::string_view g_state = unpack_g_state.at(goal_state);

    if (h_state == "TAKEOFF" || h_state == "LAND")
    {
        ROS_INFO("Serving TAKEOFF/LAND");
        tspline.spline.clear();
        tspline.spline = LinearVert(current_goal , h_pos);
        tspline.curr_spline_id++;
    }


    else if (h_state == "TURTLE")
    {
        /*
            there will be 2 possibilities here: 
            1. Hector Position --> TurtlePosition(GOTO) --> Final Goal (GOTO)
            2. Hector Position --> TurtlePosition(CHASE) --> Final Goal (GOTO)

            Hence, the full trajectory of the turtlebot will be in 2 pieces
            (a1) TurtlePosition_PREDICTION(PosBegin) -- HectorPosition(PosEnd) : Higher Order Spline
            (a2) TurtlePosition_CHASE(PosBegin) -- HectorPosition(PosEnd) : Linear Planar Spline --> [TurtlePosition,.....,HectorPosition)
            (b) FinalGoal_GOTO(PosBegin) -- TurtlePosition(PosEnd) : Higher Order Spline --> [FinalGoal, ......, TurtlePosition)

            Spline Combination: Spline B concatenated with Spline A --> [FinalGoal, ......, TurtlePosition) + [TurtlePosition,.....,HectorPosition)
        */
        std::vector<bot_utils::Pos3D> spline_a;
        std::vector<bot_utils::Pos3D> spline_b;

        //velocities for spline_b

        bot_utils::Pos3D vel_at_turtle;
        bot_utils::Pos2D dir(h_pos.x - next_goal.x , h_pos.y - next_goal.x);
        double heading = atan2(dir.y , dir.x);
        vel_at_turtle.x = average_speed * std::cos(heading);
        vel_at_turtle.y = average_speed * std::sin(heading);
        vel_at_turtle.z = 0; //we dont use this, but for posterity's sake, lets just set this to 0
        bot_utils::Pos3D vel_at_next(0,0,0);
        
        ROS_WARN_COND(g_state == "GOTO" , "Something wrong with Trajectory State Machine!");

        if (g_state == "PREDICTION")
        {
            spline_a = Cubic(current_goal , h_pos, vel_at_turtle , h_vel , average_speed , target_dt , height , close_enuf);
        }

        else //(g_state == "CHASE")
        {
            spline_a = LinearPlanar(current_goal , h_pos , average_speed , target_dt , height , close_enuf);
        }

        spline_b = Cubic(next_goal , current_goal , vel_at_next , vel_at_turtle , average_speed , target_dt , height , close_enuf);

        //merge spline a with spline b
        //spline_a.pop_back(); //pop the last one so we dont have close targets when merging

        for (auto & tgt: spline_a)
        {
            spline_b.push_back(tgt);
        }

        tspline.spline.clear();
        tspline.spline = spline_b;
        tspline.curr_spline_id++;
    }

    else if (h_state == "GOAL")
    {
        /*
            there will be 1 possibilities here: 
            1. Hector Position --> FinalGoal(GOTO) --> Start (GOTO)

            Hence, the full trajectory of the turtlebot will be in 2 pieces
            (a) FinalGoal(PosBegin) -- HectorPosition(PosEnd) : Higher Order Spline --> [FinalGoal,.....HectorPosition]
            (b) StartGoal(PosBegin) -- FinalGoal(PosEnd) : Higher Order Spline --> [StartGoal, ......, FinalGoal)

            Spline Combination: Spline B concatenated with Spline A --> [StartGoal, ......, FinalGoal) + [FinalGoal,.....,HectorPosition)
        */

        bot_utils::Pos3D vel_at_final;
        bot_utils::Pos2D dir(h_pos.x - next_goal.x , h_pos.y - next_goal.x);
        double heading = atan2(dir.y , dir.x);
        vel_at_final.x = average_speed * std::cos(heading);
        vel_at_final.y = average_speed * std::sin(heading);
        vel_at_final.z = 0; //we dont use this, but for posterity's sake, lets just set this to 0
        bot_utils::Pos3D vel_at_next(0,0,0);

        std::vector<bot_utils::Pos3D> spline_a = Cubic(current_goal , h_pos , vel_at_final , h_vel , average_speed , target_dt , height , close_enuf);
        std::vector<bot_utils::Pos3D> spline_b = Cubic(next_goal , current_goal , vel_at_next , vel_at_final , average_speed , target_dt , height , close_enuf);
        
        for (auto & tgt: spline_a)
        {
            spline_b.push_back(tgt);
        }
        
        tspline.spline.clear();
        tspline.spline = spline_b;
        tspline.curr_spline_id++;
    }

    else if (h_state == "START")
    {
        ROS_ERROR("If I am here, something is wrong with the state machine. Check the trigggers!");
    }

}