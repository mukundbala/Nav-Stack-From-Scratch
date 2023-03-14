#include "trajectory_generator.h"


TrajectoryGenerator::TrajectoryGenerator(double target_dt , double average_speed, std::string primary_traj, double cruise_height, double takeoff_height, double land_height, bool verbose):
target_dt_(target_dt),
average_speed_(average_speed),
primary_traj_(primary_traj),
cruise_height_(cruise_height),
takeoff_height_(takeoff_height_),
land_height_(land_height_),
verbose_(verbose)
{
    if (primary_traj_ == "Cubic")
    {
        spline_gen_ = std::bind(&TrajectoryGenerator::Cubic , this , std::placeholders::_1 , std::placeholders::_2 , std::placeholders::_3 , std::placeholders::_4 );
    }

    else if (primary_traj_ == "Quintic")
    {
        spline_gen_ = std::bind(&TrajectoryGenerator::Quintic , this , std::placeholders::_1 , std::placeholders::_2 , std::placeholders::_3 , std::placeholders::_4 );
    }

    else
    {
        primary_traj_ = "Cubic";
        spline_gen_ = std::bind(&TrajectoryGenerator::Quintic , this , std::placeholders::_1 , std::placeholders::_2 , std::placeholders::_3 , std::placeholders::_4 );
    }

    ROS_INFO("[DroneCommander- TrajectoryGenerator]: Trajectory Generator Prepared");
}


std::vector<bot_utils::Pos3D> TrajectoryGenerator::LinearVert(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end)
{
    std::vector<bot_utils::Pos3D> segment_traj = {pos_begin};
    return segment_traj;

    // if (pos_begin.z > pos_end.z)
    // {
    //      std::vector<bot_utils::Pos3D> segment_traj = {pos_begin};
    //     for (int i = 0 ; i < 3 ; ++ i)
    //     {
    //         segment_traj.emplace_back(pos_begin.x ,pos_begin.y , pos_begin.z - 0.5 * (i+1));
    //     }
    //     return segment_traj;
    // }

    // else
    // {
    //     std::vector<bot_utils::Pos3D> segment_traj = {pos_begin};
    //     for (int i = 0 ; i < 3 ; ++ i)
    //     {
    //         segment_traj.emplace_back(pos_begin.x ,pos_begin.y , pos_begin.z + 0.5 * (i+1));
    //     }
    //     return segment_traj;
    // }
}

std::vector<bot_utils::Pos3D> TrajectoryGenerator::LinearPlanar(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end)
{
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double d = sqrt ((Dx * Dx) + (Dy * Dy));

    std::vector<bot_utils::Pos3D> segment_traj = {pos_begin};

    for (double time = target_dt_ ; time < d ; time += target_dt_)
    {
        double x = pos_begin.x + Dx * time / d;
        double y = pos_begin.y + Dy * time / d;
        segment_traj.emplace_back(x , y , cruise_height_);
    }

    return segment_traj;
}

std::vector<bot_utils::Pos3D> TrajectoryGenerator::Cubic(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end , bot_utils::Pos3D &vel_begin , bot_utils::Pos3D &vel_end)
{
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double d =  sqrt(Dx * Dx + Dy * Dy) / average_speed_;

    std::vector<bot_utils::Pos3D> traj = {pos_begin};

    Eigen::MatrixXd M(4,4);
    M << 1.0, 0.0 , 0.0 , 0.0,
         0.0, 1.0 , 0.0 , 0.0,
         1.0, d , d*d , d*d*d,
         0.0, 1.0, 2.0*d, 3*d*d;
    
    Eigen::Vector4d in_x(pos_begin.x , vel_begin.x , pos_end.x , vel_end.x);
    Eigen::Vector4d in_y(pos_begin.y , vel_begin.y , pos_end.y , vel_end.y);

    Eigen::Vector4d ax = M.inverse() * in_x;
    Eigen::Vector4d by = M.inverse() * in_y;

    for (double time = target_dt_ ; time < d ; time += target_dt_)
    {
        double px = ax[0] + ax[1]*time + ax[2]*time*time + ax[3]*time*time*time;
        double py = by[0] + by[1]*time + by[2]*time*time + by[3]*time*time*time;
        traj.emplace_back(px,py,cruise_height_);   
    }

    return traj;
}


std::vector<bot_utils::Pos3D> TrajectoryGenerator::Quintic(bot_utils::Pos3D &pos_begin , bot_utils::Pos3D &pos_end , bot_utils::Pos3D &vel_begin , bot_utils::Pos3D &vel_end)
{
    double Dx = pos_end.x - pos_begin.x;
    double Dy = pos_end.y - pos_begin.y;
    double d =  sqrt(Dx * Dx + Dy * Dy) / average_speed_;

    std::vector<bot_utils::Pos3D> traj = {pos_begin};

    Eigen::MatrixXd M_inv(6,6);

    M_inv << 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0,
             0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0,
             0.0 , 0.0 , 0.5, 0.0 , 0.0 , 0.0,
             -10.0/(d*d*d) , -6.0/(d*d) , -3.0/(2.0*d) , 10.0/(d*d*d) , -4.0/(d*d), 1.0/(2.0*d),
             15.0/(d*d*d*d) , 8.0/(d*d*d) , 3.0/(2.0*d*d) , -15.0/(d*d*d*d) , 7.0/(d*d*d) , -1.0/(d*d),
             -6.0/(d*d*d*d*d) , -3/(d*d*d*d) , -1.0/(2*d*d*d) , 6.0/(d*d*d*d*d) , -3.0/(d*d*d*d) , 1.0/(2*d*d*d);
    
    Eigen::VectorXd in_x(6);
    in_x << pos_begin.x , vel_begin.x , 0 , pos_end.x , vel_end.x , 0;

    Eigen::VectorXd in_y(6);
    in_y << pos_begin.y , vel_begin.y , 0 , pos_end.y , vel_end.y , 0;

    Eigen::VectorXd ax(6);
    Eigen::VectorXd by(6);

    ax = M_inv * in_x;
    by = M_inv * in_y;

    for (double time = target_dt_ ; time < d ; time += target_dt_)
    {
        double px = ax[0] + ax[1]*time + ax[2]*time*time + ax[3]*time*time*time + ax[4]*time*time*time*time + ax[5]*time*time*time*time*time;
        double py = by[0] + by[1]*time + by[2]*time*time + by[3]*time*time*time + by[4]*time*time*time*time + by[5]*time*time*time*time*time;
        traj.emplace_back(px,py,cruise_height_);   
    }
    return traj;

}   

void TrajectoryGenerator::trajectory_handler(bot_utils::Pos3D current_goal , 
                        bot_utils::Pos3D next_goal , 
                        bot_utils::Pos3D h_pos , 
                        bot_utils::Pos3D h_vel ,
                        bot_utils::SplineData3D &hspline,
                        mission_states::HectorState h_state,
                        mission_states::GoalState g_state)
{
    if (h_state == mission_states::HectorState::TAKEOFF || h_state == mission_states::HectorState::LAND)
    {
        ROS_INFO("[DroneCommander]: Serving TAKEOFF/LAND");
        hspline.spline.clear();
        hspline.spline = LinearVert(current_goal,h_pos);
        hspline.curr_spline_id++;
    }

    else if (h_state == mission_states::HectorState::TURTLE)
    {
        ROS_INFO("[DroneCommander]: SERVING TO TURTLE");
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
        vel_at_turtle.x = average_speed_ * std::cos(heading);
        vel_at_turtle.y = average_speed_ * std::sin(heading);
        vel_at_turtle.z = 0; //we dont use this, but for posterity's sake, lets just set this to 0
        bot_utils::Pos3D vel_at_next(0,0,0);
        
        ROS_WARN_COND(g_state == mission_states::GoalState::GOTO , "Something wrong with Trajectory State Machine!");

        if (g_state == mission_states::GoalState::PREDICTION)
        {
            ROS_INFO("SPLINE FOR TURTLE PREDICTION");
            spline_a = spline_gen_(current_goal , h_pos, vel_at_turtle , h_vel);
        }

        else //(g_state == "CHASE")
        {
            spline_a = LinearPlanar(current_goal , h_pos);
        }

        spline_b = spline_gen_(next_goal , current_goal , vel_at_next , vel_at_turtle);

        //merge spline a with spline b
        //spline_a.pop_back(); //pop the last one so we dont have close targets when merging

        for (auto & tgt: spline_a)
        {
            spline_b.push_back(tgt);
        }

        ROS_INFO("SPLINE SUCCESSFULLY GENERATED");
        hspline.spline.clear();
        hspline.spline = spline_b;
        hspline.curr_spline_id++;
    }

    else if (h_state == mission_states::HectorState::GOAL) //consider just extending the trajectory
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
        vel_at_final.x = average_speed_ * std::cos(heading);
        vel_at_final.y = average_speed_ * std::sin(heading);
        vel_at_final.z = 0; //we dont use this, but for posterity's sake, lets just set this to 0
        bot_utils::Pos3D vel_at_next(0,0,0);

        std::vector<bot_utils::Pos3D> spline_a = spline_gen_(current_goal , h_pos , vel_at_final , h_vel);
        std::vector<bot_utils::Pos3D> spline_b = spline_gen_(next_goal , current_goal , vel_at_next , vel_at_final);
        
        for (auto & tgt: spline_a)
        {
            spline_b.push_back(tgt);
        }
        
        hspline.spline.clear();
        hspline.spline = spline_b;
        hspline.curr_spline_id++;
    }

    else if (h_state == mission_states::HectorState::START)
    {
        ROS_ERROR("If I am here, something is wrong with the state machine. Check the trigggers!");
    }

    else if (h_state == mission_states::HectorState::FOLLOW || h_state == mission_states::HectorState::HOME)
    {
        std::vector<bot_utils::Pos3D> spline_a;

        if (g_state == mission_states::GoalState::GOTO)
        {
            bot_utils::Pos2D dir(h_pos.x - current_goal.x , h_pos.y - current_goal.x);
            bot_utils::Pos3D vel;
            double heading = atan2(dir.y , dir.x);
            vel.x = average_speed_ * std::cos(heading);
            vel.y = average_speed_ * std::sin(heading);
            vel.z = 0;
            spline_a = spline_gen_(current_goal , h_pos , vel , h_vel);
            hspline.spline.clear();
            hspline.spline = spline_a;
            hspline.curr_spline_id++;
        }

        else if (g_state == mission_states::GoalState::CHASE)
        {
            spline_a = LinearPlanar(current_goal , h_pos);
            hspline.spline.clear();
            hspline.spline = spline_a;
            hspline.curr_spline_id++;
        }
    }
}