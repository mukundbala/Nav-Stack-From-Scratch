#include "drone_commander.h"


DroneCommander::DroneCommander(ros::NodeHandle &nh)
{
    nh_ = nh;
    bool comm_params = loadCommanderParams();
    bool traj_params = loadTrajParams();
    bool cont_params = loadControllerParams();


    ROS_WARN_COND(comm_params + traj_params + cont_params < 3, "[DroneCommander]: Warning. Params not loaded properly");

    h_state_ = mission_states::HectorState::TAKEOFF;
    g_state_ = mission_states::GoalState::GOTO;

    hector_position_.setCoords(NaN,NaN,NaN); //set hector pos and heading to NaN
    hector_heading_ = NaN;
    hector_lin_vel_.setCoords(NaN,NaN,NaN); //set velocities to NaN
    hector_ang_vel_ = NaN;
    hector_spline_.curr_spline_id = -1;
    hector_vel_mag_ = 0.35;


    turtle_position_.setCoords(NaN,NaN); //set turtle pos, heading to Nan. Set spline to -1. 
    turtle_heading_ = NaN;
    turtle_spline_.curr_spline_id = -1;
    ts_id_ = -1;

    //initialise triggers
    gen_traj_passthrough_ = true; //this trigger set to true to handle takeoff
    gen_traj_turtle_ = false;
    kill_switch_ = false;

    //initialise target
    current_target_.setCoords(NaN,NaN,NaN);
    h_id_ = -1;

    // At this point, hector_initial_pos has already been set under loadCommanderParams

    hector_takeoff_goal_.setCoords(hector_initial_pos_.x , hector_initial_pos_.y , takeoff_height_);
    hector_land_goal_.setCoords(hector_initial_pos_.x , hector_initial_pos_.y , land_height_);
    hector_start_goal_.setCoords(NaN,NaN,NaN);
    hector_end_goal_.setCoords(NaN,NaN,NaN);
    hector_home_goal_.setCoords(NaN,NaN,NaN);
    hector_pred_goal_.setCoords(NaN,NaN);
    pred_id_ = -1;

    current_goal_= hector_takeoff_goal_;
    next_goal_.setCoords(NaN,NaN,NaN);

    if (co_op_) //cooperation with turtlebot mode
    {
        if (nh_.hasParam("/turtle/goals"))
        {
            XmlRpc::XmlRpcValue goal_loader;
            nh_.getParam("/turtle/goals",goal_loader);

            if (goal_loader.getType() == XmlRpc::XmlRpcValue::TypeArray)
            {
                int total_goals = goal_loader.size();
                hector_end_goal_.x = goal_loader[total_goals - 1][0];
                hector_end_goal_.y = goal_loader[total_goals - 1][1];
                hector_end_goal_.z = cruise_height_;
            }
            hector_start_goal_.setCoords(hector_initial_pos_.x , hector_initial_pos_.y , cruise_height_);
        }
        else
        {
            kill_switch_ = true;
        }
    }
    else
    {
        
        XmlRpc::XmlRpcValue goal_loader;
        nh_.getParam("solo_goals" , goal_loader);

        if (goal_loader.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            int total_goals = goal_loader.size();
            for (int i = 0; i < total_goals ; ++i)
            {
                XmlRpc::XmlRpcValue ob = goal_loader[i];
                bot_utils::Pos3D goal(ob[0] , ob[1] , ob[2]);
                goal.print();
                solo_goals_.push_back(goal);
            }
            hector_home_goal_.setCoords(hector_initial_pos_.x , hector_initial_pos_.y , cruise_height_);
            goal_full_msg_.goal_id = -2; //if its takeoff, id is -3
        }
    }
    //initialise velocities to 0
    vel_x_ = 0;
    vel_y_ = 0;
    vel_z_ = 0;
    
    //setup messages
    traj_msg_.header.frame_id = "world";
    target_msg_.header.frame_id = "world";
    goal_msg_.header.frame_id = "world";
    vel_msg_.linear.x = 0;
    vel_msg_.linear.y = 0;
    vel_msg_.linear.z = 0;
    vel_msg_.angular.z = 0;

    //setup publishers
    traj_pub_ = nh_.advertise<nav_msgs::Path>("trajectory", 1, true);
    target_pub_ = nh_.advertise<geometry_msgs::PointStamped>("target", 1, true);
    rotate_pub_ = nh_.advertise<std_msgs::Bool>("rotate", 1, true);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel" ,1 , true);
    motor_switch_client_ = nh_.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");

    //setup subscribers
    sub_h_pose_ = nh_.subscribe("pose", 1, &DroneCommander::callbackHPose,this);
    sub_h_vel_ = nh_.subscribe("velocity", 1, &DroneCommander::callbackHVel,this);
    sub_t_pose_ = nh_.subscribe("/turtle/pose", 1, &DroneCommander::callbackTPose,this);
    sub_t_spline_ = nh_.subscribe("/turtle/spline" , 1 , &DroneCommander::callbackTSpline,this);

    if (co_op_)
    {
        ROS_INFO("[DroneCommander]: Drone Commander Prepared! In Co-Op Mode with Turtle!");
    }
    
    else
    {
        //Note to self. Shut down the general goal subscriber from mission planner!
        sub_t_pose_.shutdown();
        sub_t_spline_.shutdown();
        vel_mag_pub_ = nh_.advertise<std_msgs::Float64>("vel_mag" , 1 , true);
        goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("goal" , 1 , true);
        goal_full_pub_ = nh_.advertise<hmsgs::Goal>("goal_full" , 1 , true);
        error_pub_ = nh_.advertise<std_msgs::Float64>("error" , 1 , true);
        error_vec_pub_ = nh_.advertise<geometry_msgs::Point>("error_vec" , 1 , true);
        ROS_INFO("[DroneCommander]: Drone Commander Prepared! In Solo-Flight mode!");
    }
}

void DroneCommander::callbackHPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    auto &p = msg->pose.pose.position;
    hector_position_.setCoords(p.x,p.y,p.z);

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    hector_heading_ = atan2(siny_cosp, cosy_cosp);
}

void DroneCommander::callbackHVel(const geometry_msgs::Twist::ConstPtr &msg)
{
    hector_lin_vel_.setCoords(msg->linear.x , msg->linear.y, msg->linear.z);
    hector_ang_vel_ = msg->angular.z;

    if (h_state_ != mission_states::HectorState::TAKEOFF)
    {
        hector_vel_mag_ = sqrt(msg->linear.x * msg->linear.x + msg->linear.y * msg->linear.y); //compute planar magnitude only when its not taking off
        vel_mag_msg_.data = hector_vel_mag_; //attach value to the message
    }
    
}

void DroneCommander::callbackTPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    turtle_position_.setCoords(p.x,p.y);

    auto &q = msg->pose.orientation;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    turtle_heading_ = atan2(siny_cosp, cosy_cosp);
}

void DroneCommander::callbackTSpline(const tmsgs::TurtleSplineConstPtr &spline_msg)
{
    int received_id = spline_msg->spline_id;
    if (received_id != turtle_spline_.curr_spline_id)
    {
        turtle_spline_.curr_spline_id = received_id;
        turtle_spline_.avg_speed = spline_msg->average_speed;
        turtle_spline_.target_dt = spline_msg -> target_dt;
        turtle_spline_.spline.clear();

        auto &p = spline_msg->spline.poses;

        for (auto &m : p)
        {
            turtle_spline_.spline.emplace_back(m.pose.position.x , m.pose.position.y);
        }

        if (verbose_)
        {
            ROS_INFO("[DroneCommander]: New Turtle Spline Received!");
        }
        gen_traj_turtle_ = true;
    }
}

std::pair<bot_utils::Pos2D,int> DroneCommander::predict_turtle_pos()
{
    bot_utils::Pos2D h_planar_pos(hector_position_.x , hector_position_.y);
    std::pair<bot_utils::Pos2D,int> res;
    int tbot_id = ts_id_;
    assert(tbot_id != -1);
    /*
    The idea here is that targets are generated at regular interval dt. So, from one target to another, it takes dt.
    Using this idea, we can estimate how long the turtlebot will take to go from current target to target n.
    The best target n is the one that the drone can reach as quickly as the turtlebot. There can be a few cases here

    1. The turtlebot can reach the last target in the spline faster than the drone can. In this case, we just return the last target in the spline and leave the state machine to figure it out
    2. We find a point n at which the drone can reach about as fast as the turtle bot. Return that point
    */
   double tbot_time = 0;
   double tru_avg_speed = 0.35;
   for (int i = tbot_id ; i < turtle_spline_.spline.size() -1 ; ++i)
   {
        tbot_time += turtle_spline_.target_dt; //turtle will reach i to i+1 within target_dt

        double hector_dist = bot_utils::dist_euc(hector_position_.x , hector_position_.y , turtle_spline_.spline.at(i+1).x , turtle_spline_.spline.at(i+1).y); //the distance hector needs to fly to the i+1 th index

        double hector_time = hector_dist / hector_vel_mag_;

        if (tbot_time - hector_time >= 1e-6)
        {
            res = {turtle_spline_.spline.at(i+1) , i+1};
            if (verbose_)
            {
                ROS_INFO("[DroneCommander]: Found a prediction!");
                res.first.print();
                ROS_INFO_STREAM("[DroneCommander]: ID: "<<res.second);
            }
            return res;
        }
        //we continue searching otherwise
   }
   
   //if we came here without returning anything, this means we couldnt find a good enough time
   res = {turtle_spline_.spline.at(turtle_spline_.spline.size()-1) , turtle_spline_.spline.size()- 1};
   if (verbose_)
   {
        ROS_INFO("[DroneCommander]: Prediction returning last target");
        res.first.print();
   }
   return res; //return the last value in the spline
}

void DroneCommander::run()
{
    ros::Rate spinrate(rate_);
    nh_.setParam("run" , true);

    TrajectoryGenerator trajectory_generator(target_dt_ , average_speed_ , traj_name_ , cruise_height_ , takeoff_height_ , land_height_ , verbose_traj_);
    VelocityController velocity_controller(controller_params_, verbose_);

    ROS_INFO("[DroneCommander]: Waiting for Topics!");
    if (co_op_)
    {
        while (ros::ok() && nh_.param("run", true) && (std::isnan(hector_position_.x) || std::isnan(turtle_position_.x) || 
                                                    std::isnan(hector_lin_vel_.x) || turtle_spline_.curr_spline_id == -1))
        {
            ros::spinOnce();
        }
    }

    else
    {
        while (ros::ok() && nh_.param("run", true) && (std::isnan(hector_position_.x) || std::isnan(hector_lin_vel_.x)))
        {
            ros::spinOnce();
        }
    }

                                     
    ROS_INFO("[DroneCommander]: Starting Drone Commander!");
    ROS_INFO("[DroneCommander]: Starting Motors");
    
    bool allOk = armMotor();

    if (!allOk)
    {
        nh_.setParam("run" , false);
        ROS_INFO("[DroneCommander]: Avionics shut down!");
    }

    double now = ros::Time::now().toSec();
    velocity_controller.prepareController(now);
    
    ROS_INFO("[DroneCommander]: Taking Off!");
    
    while (ros::ok() && nh_.param("run" , true))
    {
        ros::spinOnce();
        double time_now = ros::Time::now().toSec(); 
        bool vel_cont_status = velocity_controller.updateDt(time_now);
        if (!vel_cont_status)
        {
            continue;
        }

        if (co_op_)
        {
            ts_id_ = turtle_spline_.find_pos_id(turtle_position_); //prepare for the current loop
        }
        
    
        if (h_state_ == mission_states::HectorState::TAKEOFF)
        {
            rotate_msg_.data = false;
            gen_traj_turtle_ = false; //set this to false to counter callback demands due to new TSpline arriving
            if (bot_utils::dist_euc(hector_position_.x , hector_position_.y , current_goal_.x , current_goal_.y) < thresh_cruise_planar_ && std::abs(hector_position_.z - takeoff_height_) < 0.05)
            {
                if (verbose_)
                {
                    ROS_INFO_COND(co_op_ , "[DroneCommander]: TRANSITION FROM TAKEOFF TO TURTLE");
                    ROS_INFO_COND(!co_op_, "[DroneCommander]: TRANSITION FROM TAKEOFF TO FOLLOW");
                }

                if (co_op_)
                {
                    h_state_ = mission_states::HectorState::TURTLE;
                    g_state_ = mission_states::GoalState::PREDICTION;
                    std::tie(hector_pred_goal_,pred_id_) = predict_turtle_pos();
                    current_goal_.setCoords(hector_pred_goal_.x , hector_pred_goal_.y , cruise_height_); //current_goal: turtlebot's predicted position
                    next_goal_.setCoords(hector_end_goal_.x , hector_end_goal_.y , cruise_height_); //next_goal: final waypoint of turtlebot
                    gen_traj_turtle_ = true;
                    gen_traj_passthrough_ = false;
                }

                else
                {
                    h_state_ = mission_states::HectorState::FOLLOW;
                    g_state_ = mission_states::GoalState::GOTO;
                    solo_goal_id_ = 0;
                    current_goal_ = solo_goals_.at(solo_goal_id_);
                    goal_full_msg_.goal_id = solo_goal_id_;
                    gen_traj_turtle_ = false;
                    gen_traj_passthrough_ = true;
                }

            }
        }

        else if (h_state_ == mission_states::HectorState::TURTLE)
        {
            rotate_msg_.data = true;
            gen_traj_passthrough_ = false;

            bool hector_reached_turtle_planar = bot_utils::dist_euc(hector_position_.x , hector_position_.y , turtle_position_.x , turtle_position_.y) < thresh_cruise_planar_;
            bool hector_at_target_height = std::abs(hector_position_.z-cruise_height_) < thresh_cruise_height_;
            
            if (hector_reached_turtle_planar && hector_at_target_height) //base check if we are close enough to the turtle
            {
                //transition state
                ROS_INFO("[DroneCommander]:TRANSITION FROM TURTLE TO GOAL");
                h_state_ = mission_states::HectorState::GOAL;
                g_state_ = mission_states::GoalState::GOTO;
                current_goal_ = hector_end_goal_; //current_goal: final waypoint of turtlebot
                next_goal_ = hector_start_goal_;  //next_goal: start position of hector
                gen_traj_turtle_ = false;
                gen_traj_passthrough_ = true;
            }
            else //if we are nowhere close to the turtle yet
            {
                if (gen_traj_turtle_)
                {
                    //a case where we recived a new trajectory
                    std::tie(hector_pred_goal_,pred_id_) = predict_turtle_pos();
                    current_goal_.setCoords(hector_pred_goal_.x , hector_pred_goal_.y , cruise_height_); //current_goal: Predictied position of turtle
                    next_goal_ = hector_end_goal_; //next_goal: End point of hector
                    g_state_ = mission_states::GoalState::PREDICTION;
                }

                else //!generate_trajectory_turtle
                {
                    //we have received no new trajectories.
                    if (g_state_ == mission_states::GoalState::PREDICTION)
                    {
                        bool turtle_reached_pred = ts_id_ >= pred_id_;
                        bool hector_reached_pred = bot_utils::dist_euc(hector_position_.x , hector_position_.y , hector_pred_goal_.x , hector_pred_goal_.y) < thresh_cruise_planar_;

                        if (turtle_reached_pred && !hector_reached_pred)
                        {
                            //if the turtle reached the predicted position BEFORE the hector, we run the prediction again
                            std::tie(hector_pred_goal_,pred_id_) = predict_turtle_pos();
                            current_goal_.setCoords(hector_pred_goal_.x , hector_pred_goal_.y , cruise_height_); //current_goal: Predicted position of turtle
                            next_goal_ = hector_end_goal_; //next_goal: Final waypoint of hector
                            g_state_ = mission_states::GoalState::PREDICTION;
                            gen_traj_turtle_ = true;
                        }

                        else if (hector_reached_pred & !turtle_reached_pred)
                        {
                            //if the hector reached the predicted position BEFORE the turtle, we just switch to chasing the turtle
                            hector_pred_goal_ = turtle_position_;
                            pred_id_ = -2; //set this to signal that we are chasing the turtlebot
                            current_goal_.setCoords(turtle_position_.x , turtle_position_.y , cruise_height_); //current_goal: Actual position of turtle
                            next_goal_ = hector_end_goal_; //next_goal: Final waypoint of turtle
                            g_state_ = mission_states::GoalState::CHASE;
                            gen_traj_turtle_ = true;
                        }

                        else if (!hector_reached_pred && !turtle_reached_pred)
                        {
                            //if neither has reached the prediction, we can run the prediction again
                            int initial_pred = pred_id_;
                            int new_pred_id;
                            bot_utils::Pos2D new_pred_goal;
                            std::tie(new_pred_goal,new_pred_id) = predict_turtle_pos();
                            if (new_pred_id != initial_pred)
                            {
                                hector_pred_goal_.setCoords(new_pred_goal.x,new_pred_goal.y);
                                current_goal_.setCoords(hector_pred_goal_.x , hector_pred_goal_.y , cruise_height_);
                                next_goal_ = hector_end_goal_;
                                g_state_ = mission_states::GoalState::PREDICTION;
                                gen_traj_turtle_ = true;
                            }
                            // ROS_WARN_STREAM("INITIAL_PRED: " << initial_pred);
                            // ROS_WARN_STREAM("NEW_PRED: " << new_pred_id);
                        }
                    }
                    
                    else if (g_state_ == mission_states::GoalState::CHASE)
                    {
                        if (bot_utils::dist_euc(hector_position_.x , hector_position_.y , turtle_position_.x , turtle_position_.y) < thresh_cruise_planar_ && std::abs(hector_position_.z-cruise_height_) < thresh_cruise_height_)
                        {
                            ROS_INFO("[DroneCommander]: TRANSITION FROM TURTLE TO GOAL");
                            h_state_ = mission_states::HectorState::GOAL;
                            g_state_ = mission_states::GoalState::GOTO;
                            current_goal_ = hector_end_goal_;
                            next_goal_ = hector_takeoff_goal_;
                            gen_traj_turtle_ = false;
                            gen_traj_passthrough_ = true;
                        }
                        else
                        {
                            hector_pred_goal_ = turtle_position_;
                            pred_id_ = -2;
                            current_goal_.setCoords(turtle_position_.x , turtle_position_.y , cruise_height_);
                            next_goal_ = hector_end_goal_;
                            g_state_ = mission_states::GoalState::CHASE;
                            gen_traj_turtle_ = true;
                        }
                    }
                }
            }
        }

        else if (h_state_ == mission_states::HectorState::GOAL)
        {
            rotate_msg_.data = true;
            gen_traj_passthrough_ = false;
            gen_traj_turtle_ = false;
            bool hector_reached_end_goal = bot_utils::dist_euc(hector_position_.x , hector_position_.y , hector_end_goal_.x , hector_end_goal_.y) < thresh_cruise_planar_;
            bool hector_reached_end_height = std::abs(hector_position_.z - cruise_height_) < thresh_cruise_height_;
            
            if (hector_reached_end_goal && hector_reached_end_height)
            {
                //state transition to START
                ROS_INFO("[DroneCommander]: TRANSITION FROM GOAL TO START");
                h_state_ = mission_states::HectorState::START;
                g_state_ = mission_states::GoalState::GOTO;
                current_goal_ = hector_start_goal_;
                next_goal_.setCoords(turtle_position_.x , turtle_position_.y , cruise_height_);
                // gen_traj_turtle_ = false;
                // gen_traj_passthrough_ =true;
            }
        }

        else if (h_state_ == mission_states::HectorState::START)
        {   
            rotate_msg_.data = true;
            gen_traj_turtle_ = false;
            gen_traj_passthrough_ = false;
            bool hector_reached_start_planar = bot_utils::dist_euc(hector_position_.x , hector_position_.y , hector_start_goal_.x , hector_start_goal_.y) < thresh_cruise_planar_;
            bool hector_reached_start_height = std::abs(hector_position_.z - cruise_height_) < thresh_cruise_height_;

            if (!nh_.param("/turtle/trigger_nodes", false))
            { 
                if (hector_reached_start_planar && hector_reached_start_height)
                {
                    //transition to landing
                    ROS_INFO("[DroneCommander]: TRANSITION FROM START TO LAND");
                    h_state_ = mission_states::HectorState::LAND;
                    g_state_ = mission_states::GoalState::CHASE;
                    current_goal_ = hector_land_goal_;
                    next_goal_.setCoords(NaN,NaN,NaN);
                    gen_traj_turtle_ = false;
                    gen_traj_passthrough_ = true;
                }
            }

            else
            {
                if (hector_reached_start_planar && hector_reached_start_height)
                {
                    //state transition
                    ROS_INFO("[DroneCommander]: TRANSITION FROM START TO TURTLE");
                    h_state_ = mission_states::HectorState::TURTLE;
                    g_state_ = mission_states::GoalState::PREDICTION;
                    std::tie(hector_pred_goal_,pred_id_) = predict_turtle_pos();
                    current_goal_.setCoords(hector_pred_goal_.x , hector_pred_goal_.y , cruise_height_);
                    next_goal_ = hector_end_goal_;
                    gen_traj_turtle_ = true;
                    gen_traj_passthrough_ = false;
                }
            }
        }

        else if (h_state_ == mission_states::HectorState::LAND)
        {
            rotate_msg_.data = false;
            gen_traj_turtle_ = false;
            gen_traj_passthrough_ = false;
            bool hector_reached_land_planar = bot_utils::dist_euc(hector_position_.x , hector_position_.y , hector_land_goal_.x , hector_land_goal_.y) < thresh_cruise_planar_;
            bool hector_reached_land_height = std::abs(hector_position_.z - land_height_) < thresh_land_height_;

            if (hector_reached_land_planar && hector_reached_land_height)
            {
                ROS_INFO("[DroneCommander]: Landed Safely.");
                kill_switch_ = true;
                ROS_INFO("[DroneCommander]: Shutting down Avionics");
            }
        }

        ///APPLICABLE TO ONLY SOLO FLIGHTS!
        else if (h_state_ == mission_states::HectorState::FOLLOW)
        {
            rotate_msg_.data = true;
            gen_traj_turtle_ = false;
            gen_traj_passthrough_ = false;
            bool hector_reached_passthrough_planar = bot_utils::dist_euc(hector_position_.x , hector_position_.y , current_goal_.x , current_goal_.y) < thresh_cruise_planar_ ;
            bool hector_reached_passthrough_height = std::abs(hector_position_.z - cruise_height_) < thresh_cruise_height_;
            
            if (hector_reached_passthrough_planar && hector_reached_passthrough_height)
            {
                if (solo_goal_id_ == solo_goals_.size() - 1)
                {
                    //this means that we have reached out last goal
                    ROS_INFO("[DroneCommander]: TRANSITION FROM FOLLOW TO HOME");
                    h_state_ = mission_states::HectorState::HOME;
                    g_state_ = mission_states::GoalState::GOTO;
                    current_goal_ = hector_home_goal_;
                    goal_full_msg_.goal_id = solo_goals_.size();
                    //next_goal_.setCoords(NaN,NaN,NaN);
                    gen_traj_passthrough_ = true;
                }
                else
                {
                    solo_goal_id_ ++;
                    goal_full_msg_.goal_id = solo_goal_id_;
                    current_goal_ = solo_goals_.at(solo_goal_id_);
                    h_state_ = mission_states::HectorState::FOLLOW;
                    g_state_ = mission_states::GoalState::GOTO;
                    //next_goal_.setCoords(NaN,NaN,NaN);
                    gen_traj_passthrough_ = true;
                }
            }
        }

        else if (h_state_ == mission_states::HectorState::HOME)
        {
            bool hector_reached_home_planar = bot_utils::dist_euc(hector_position_.x , hector_position_.y , hector_home_goal_.x , hector_home_goal_.y) < thresh_cruise_planar_;
            bool hector_reached_home_height = std::abs(hector_position_.z - cruise_height_) < thresh_cruise_height_;

            if (hector_reached_home_planar && hector_reached_home_height)
            {
                ROS_INFO("[DroneCommander]:TRANSITION FROM HOME TO LAND");
                h_state_ = mission_states::HectorState::LAND;
                g_state_ = mission_states::GoalState::CHASE;
                current_goal_ = hector_land_goal_;
                goal_full_msg_.goal_id = -1; //-2 is takeoff, -1 is land
                //next_goal_.setCoords(NaN,NaN,NaN);
                gen_traj_turtle_ = false;
                gen_traj_passthrough_ = true;
            }
        }


        if (kill_switch_)
        {
            break;
        }
        
        
        if (gen_traj_passthrough_ || gen_traj_turtle_)
        {
            trajectory_generator.trajectory_handler(current_goal_ , next_goal_ ,hector_position_ , hector_lin_vel_ , hector_spline_ , h_state_ , g_state_);
            h_id_ = hector_spline_.spline.size() - 1;

            if (hector_spline_.spline.size() > look_ahead_)
            {
                h_id_ -= look_ahead_;
            }
            gen_traj_passthrough_ = false;
            gen_traj_turtle_ = false;
        }

        current_target_ = hector_spline_.spline.at(h_id_);

        bool hector_reached_target_planer = bot_utils::dist_euc(hector_position_.x , hector_position_.y, current_target_.x , current_target_.y) < thresh_cruise_planar_;
        bool hector_reached_target_height = false;

        if (h_state_ == mission_states::HectorState::TAKEOFF)
        {
            hector_reached_target_height = std::abs(hector_position_.z - current_target_.z) < thresh_takeoff_height_;
            ROS_INFO("HERE1");
        }
        else if (h_state_ == mission_states::HectorState::LAND)
        {
            hector_reached_target_height = std::abs(hector_position_.z - current_target_.z) < thresh_land_height_;
        }

        else
        {
            hector_reached_target_height = std::abs(hector_position_.z - current_target_.z) < thresh_cruise_height_;
        }

        if (hector_reached_target_planer && hector_reached_target_height)
        {
            if (h_id_ == 0)
            {
                current_target_ = hector_spline_.spline.at(0);
            }
            else
            {
                h_id_--;
                current_target_ = hector_spline_.spline.at(h_id_);
            }
        }
        
        ROS_ERROR_COND(std::isnan(current_target_.x) || std::isnan(current_target_.y),"[DroneCommander]: Warning! Current target is NAN");
        
        std::array<double,4> curr_vels = velocity_controller.generate_velocities(hector_position_ , hector_heading_ , current_target_ , rotate_msg_.data);
        
        
        writeTrajMsg();
        writeTargetMsg();

        if (enable_controller_)
        {
            writeVelocityMsg(curr_vels);
            vel_pub_.publish(vel_msg_);
        }

        traj_pub_.publish(traj_msg_);
        target_pub_.publish(target_msg_);   
        rotate_pub_.publish(rotate_msg_);

        if (!co_op_)
        {
            double error = bot_utils::dist_euc(hector_position_.x , hector_position_.y , current_goal_.x , current_goal_.y);
            error_msg_.data = error;
            goal_msg_.point.x = current_goal_.x;
            goal_msg_.point.y = current_goal_.y;
            goal_msg_.point.z = current_goal_.z;
            goal_full_msg_.goal_position.point = goal_msg_.point;

            error_vec_msg.x = abs(current_goal_.x - hector_position_.x); //errors are in absolute terms
            error_vec_msg.y = abs(current_goal_.y - hector_position_.y);
            error_vec_msg.z = abs(current_goal_.z - hector_position_.z);

            vel_mag_pub_.publish(vel_mag_msg_);
            goal_pub_.publish(goal_msg_);
            goal_full_pub_.publish(goal_full_msg_);
            error_pub_.publish(error_msg_);
            error_vec_pub_.publish(error_vec_msg);
        }
        
        
        if (verbose_)
        {
            
            ROS_INFO_STREAM("[DroneCommander]: H STATE: " << mission_states::unpack_h_state(h_state_));
            ROS_INFO_STREAM("[DroneCommander]: G_STATE: " << mission_states::unpack_g_state(g_state_));
            ROS_INFO_STREAM("[DroneCommander]: CURRENT GOAL: (" << current_goal_.x << "," << current_goal_.y << "," << current_goal_.z << ")");
            ROS_INFO_STREAM("[DroneCommander]: NEXT GOAL: (" << next_goal_.x << "," << next_goal_.y << "," << next_goal_.z << ")");
            ROS_INFO_STREAM("[DroneCommander]: CURRENT TARGET: " << current_target_.x << "," << current_target_.y << "," << current_target_.z << ")");
            ROS_WARN_STREAM("[DroneCommander]: LINEAR VELOCITY: " << hector_lin_vel_.x << "," << hector_lin_vel_.y << "," << hector_lin_vel_.z << ")");
            ROS_WARN_STREAM("[DroneCommander]: PLANAR X-Y VELOCITY MAGNITUDE" << hector_vel_mag_);
            ROS_WARN_STREAM("[DroneCommander]: VERTICAL Z VELOCITY MAGNITUDE" << std::abs(hector_lin_vel_.z));
            ROS_INFO("#######################################################");
        }

        spinrate.sleep();
    }

    ROS_INFO("[DroneCommander]: Disabling Motors!");
    bool disableok = disableMotor();

    if (!disableok)
    {
        ROS_INFO("[DroneCommander]: Crashing avionics! Watch out!");
    }
    nh_.setParam("run", false);
    return;
};



bool DroneCommander::armMotor()
{
    int attempts = 1;
    int max_attempts = 5;
    bool status = false;

    while (!status && attempts <= max_attempts)
    {
        motor_switch_srv_.request.enable = true;

        if (motor_switch_client_.call(motor_switch_srv_))
        {
            ROS_INFO_STREAM("[DroneCommander]: Attempt " << attempts << "/" << max_attempts << ": Motors Armed!");
            status = true;
        }

        else
        {
            ROS_WARN_STREAM("[DroneCommander]: Attempt " << attempts << "/" << max_attempts << ": Unable to Arm Motor! Trying again");
            attempts ++;
        }
    }

    if (status)
    {
        ROS_INFO("[DroneCommander]: Ready to Fly!");
    }

    else
    {
        ROS_INFO("[DroneCommander]: Shutting down avionics!");
    }

    return status;
}

bool DroneCommander::disableMotor()
{
    int attempts = 1;
    int max_attempts = 5;
    bool status = false;

    while (!status && attempts <= max_attempts)
    {
        motor_switch_srv_.request.enable = false;

        if (motor_switch_client_.call(motor_switch_srv_))
        {
            ROS_INFO_STREAM("[DroneCommander]: Attempt " << attempts << "/" << max_attempts << ": Motors Disabled!");
            status = true;
        }

        else
        {
            ROS_WARN_STREAM("[DroneCommander]: Attempt " << attempts << "/" << max_attempts << ": Unable to Arm Motor! Trying again");
            attempts++;
        }
    }

    if (status)
    {
        ROS_INFO("[DroneCommander]: Motors disabled succesfully!");
    }
    
    else
    {
        ROS_WARN("[DroneCommander]: Danger! Unable to disable motors!");
    }

    return status;
}

void DroneCommander::sigintHandler()
{
    ROS_INFO("[DroneCommander]: SIGINT RECEIVED!");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = false;
    motor_switch_client_.call(en_mtrs_srv); 
}

void DroneCommander::writeTrajMsg()
{
    traj_msg_.poses.clear();
    for (auto &p : hector_spline_.spline)
    {
        //p.print();
        geometry_msgs::PoseStamped x;
        x.pose.position.x = p.x;
        x.pose.position.y = p.y;
        x.pose.position.z = p.z;

        traj_msg_.poses.push_back(x);
    }
}

void DroneCommander::writeTargetMsg()
{
    target_msg_.point.x = current_target_.x;
    target_msg_.point.y = current_target_.y; 
    target_msg_.point.z = current_target_.z;
}

void DroneCommander::writeVelocityMsg(std::array<double,4> &vels)
{
    vel_msg_.linear.x = vels.at(0);
    vel_msg_.linear.y = vels.at(1);
    vel_msg_.linear.z = vels.at(2);
    vel_msg_.angular.z = vels.at(3);
}

bool DroneCommander::loadCommanderParams()
{
    bool status = true;

    if (!nh_.param("verbose_commander", verbose_, false))
    {
        ROS_WARN("[DroneCommander]: Param verbose_commander not found, set to false");
        status = false;
    }

    if (!nh_.param("commander_rate", rate_, 30.0))
    {
        ROS_WARN("[DroneCommander]: Param commander_rate not found, set to 30.0");
        status = false;
    }

    if (!nh_.param("cruise_height", cruise_height_, 2.0))
    {
        ROS_WARN("[DroneCommander]: Param cruise_height not found, set to 2.0");
        status = false;
    }

    if (!nh_.param("takeoff_height", takeoff_height_, 2.0))
    {
        ROS_WARN("[DroneCommander]: Param takeoff_height not found, set to 2.0");
        status = false;
    }

    if (!nh_.param("land_height", land_height_, 0.18))
    {
        ROS_WARN("[DroneCommander]: Param land_height not found, set to 0.18");
        status = false;
    }

    if (!nh_.param("thresh_cruise_height", thresh_cruise_height_, 0.2))
    {
        ROS_WARN("[DroneCommander]: Param thresh_cruise_height not found, set to 0.2");
        status = false;
    }

    if (!nh_.param("thresh_takeoff_height", thresh_takeoff_height_, 0.1))
    {
        ROS_WARN("[DroneCommander]: Param thresh_takeoff_height not found, set to 0.1");
        status = false;
    }

    if (!nh_.param("thresh_land_height", thresh_land_height_, 0.05))
    {
        ROS_WARN("[DroneCommander]: Param thresh_cruise_height not found, set to 0.05");
        status = false;
    }

    if (!nh_.param("thresh_cruise_planar", thresh_cruise_planar_, 0.2))
    {
        ROS_WARN("[DroneCommander]: Param thresh_cruise_planar not found, set to 0.2");
        status = false;
    }

    if (!nh_.param("look_ahead", look_ahead_, 10))
    {
        ROS_WARN("[DroneCommander]: Param look_ahead not found, set to 10");
        status = false;
    }

    if (!nh_.param("co_op" , co_op_, false))
    {
        ROS_WARN("[DroneCommander]: Param co_op not found, set to false");
        status = false;
    }

    if (!nh_.param("initial_x", hector_initial_pos_.x, 0.0))
    {
        ROS_WARN("[DroneCommander]:: Param initial_x not found, set initial_x to 0.0");
        status = false;
    }
        
    if (!nh_.param("initial_y", hector_initial_pos_.y, 0.0))
    {
        ROS_WARN("[DroneCommander]: Param initial_y not found, set initial_y to 0.0");
        status = false;
    }
        
    if (!nh_.param("initial_z", hector_initial_pos_.z, 0.178))
    {
        ROS_WARN("[DroneCommander]: Param initial_z not found, set initial_z to 0.178");
        status = false;
    }
        
    return status;
}

bool DroneCommander::loadTrajParams()
{
    bool status = true;
    if (!nh_.param("target_dt", target_dt_, 0.030))
    {
        ROS_WARN("[DroneCommander]: Param target_dt not found, set to 0.030");
        status = false;
    }

    if (!nh_.param("average_speed", average_speed_, 2.0))
    {
        ROS_WARN("[DroneCommander]: Param average_speed not found, set to 2.0");
        status = false;
    }

    if (!nh_.param<std::string>("primary_traj", traj_name_, "Cubic"))
    {
        ROS_WARN("[DroneCommander]: Param primary_traj not found, set to Cubic");
        status = false;
    }

    if (!nh_.param("verbose_trajectory", verbose_traj_, false))
    {
        ROS_WARN("[DroneCommander]: Param verbose_trajectory not found, set to false");
        status = false;
    }

    return status;
}

bool DroneCommander::loadControllerParams()
{
    bool status = true;
    if (!nh_.param("enable_controller", enable_controller_, true))
    {
        ROS_WARN("[DroneCommander]: Param enable_move not found, set to true");
        status = false;
    }
        
    if (!nh_.param("verbose_controller", verbose_controller_, false))
    {
        ROS_WARN("[DroneCommander]: Param verbose_move not found, set to false");
        status = false;
    }
        
    if (!nh_.param("Kp_lin", controller_params_.KP_LIN, 1.0))
    {
        ROS_WARN("[DroneCommander]: Param Kp_lin not found, set to 1.0");
        status = false;
    }
        
    if (!nh_.param("Ki_lin", controller_params_.KI_LIN, 0.0))
    {
        ROS_WARN("[DroneCommander]: Param Ki_lin not found, set to 0");
        status = false;
    }
        
    if (!nh_.param("Kd_lin", controller_params_.KD_LIN, 0.0))
    {
        ROS_WARN("[DroneCommander]: Param Kd_lin not found, set to 0");
        status = false;
    }
        
    if (!nh_.param("Kp_z", controller_params_.KP_Z, 1.0))
    {   
        ROS_WARN("[DroneCommander]: Param Kp_z not found, set to 1.0");
        status = false;
    }
        
    if (!nh_.param("Ki_z", controller_params_.KI_Z, 0.0))
    {
        ROS_WARN("[DroneCommander]: Param Ki_lin not found, set to 0");
        status = false;
    }
        
    if (!nh_.param("Kd_z", controller_params_.KD_Z, 0.0))
    {
        ROS_WARN("[DroneCommander]: Param Kd_lin not found, set to 0");
        status = false;
    }
        
    
    if (!nh_.param("yaw_rate", controller_params_.YAW_RATE, 0.5))
    {
        ROS_WARN("[DroneCommander]: Param yaw_rate not found, set to 0.5");
        status = false;
    }
        
    
    if (!nh_.param("max_lin_vel", controller_params_.MAX_LIN_VEL, 2.0))
    {
        ROS_WARN("[DroneCommander]: Param max_lin_vel not found, set to 2");
        status = false;
    }
        
    
    if (!nh_.param("max_z_vel", controller_params_.MAX_Z_VEL, 0.5))
    {
        ROS_WARN("[DroneCommander]: Param max_z_vel not found, set to 0.5");
        status = false;
    }
    return status;
}