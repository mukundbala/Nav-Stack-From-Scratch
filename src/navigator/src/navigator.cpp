#include "navigator.h"


Navigator::Navigator(ros::NodeHandle& nh)
{
    nh_ = nh;

    //loading parameters from param server
    bool param_status = loadParams();
    ROS_WARN_COND(!param_status , "[Navigator]: Parameters have not been loaded correctly!");
    
    //initalize map variables
    mapdata.origin_ = mapdata.pos_min_;
    mapdata.map_size_.i = std::round((mapdata.pos_max_.x - mapdata.pos_min_.x) / mapdata.cell_size_);
    mapdata.map_size_.j = std::round((mapdata.pos_max_.y - mapdata.pos_min_.y) / mapdata.cell_size_);
    mapdata.total_cells_ = mapdata.map_size_.i * mapdata.map_size_.j;

    //initialise robot variables
    robot_position_.setCoords(-500,-500); //-500 is a very unlikely value
    robot_index_.setIdx(-500,-500); //-500 is a very unlikely value
    backup_robot_position_ = robot_position_;
    robot_status_ = true; //robot is okay
    backup_mode_ = false;

    //set up grid arrays
    mapdata.grid_inflation_.resize(mapdata.total_cells_);
    mapdata.grid_logodds_.resize(mapdata.total_cells_);

    //setup goals
    current_goal_.setCoords(-1000 , -1000); //again some very unlikely value
    goal_status_ = true;
    current_goal_id_ = -1;

    //setup triggers
    trigger_plan = false;
    trigger_replan = false;

    //Prepare path messages
    path_.clear();
    path_msg_.poses.clear();
    path_comm_msg_.path.poses.clear();
    path_id_ = 0;
    path_msg_.header.frame_id = "world";

    //setup subscribers
    pose_sub_ = nh_.subscribe("pose" , 1 , &Navigator::poseCallback , this);
    inflation_sub_ = nh_.subscribe("grid/mapinfo/inflation" , 1 , &Navigator::inflationCallback , this);
    lo_sub_ = nh_.subscribe("grid/mapinfo/logodds" , 1 , &Navigator::logoddsCallback , this);
    goal_sub_ = nh_.subscribe("goal" , 1 , &Navigator::goalCallback , this);
    replan_sub_ = nh_.subscribe("trigger_replan", 1 , &Navigator::replanCallback,this);

    //setup service clients
    update_goal_client_ = nh_.serviceClient<tmsgs::UpdateTurtleGoal>("update_t_goal");

    //Planner service clients. Made into a persisitent connection
    main_planner_client_ = nh_.serviceClient<tmsgs::PlanMainPath>("plan_main_path",true);
    fallback_planner_client_ = nh_.serviceClient<tmsgs::FindFallbackPosition>("find_fallback_position",true);
    
    //setup publisher
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1 , true);
    path_comm_pub_ = nh_.advertise<tmsgs::TurtlePath>("path_comm" , 1 , true);

    ROS_INFO("[Navigator]: Navigator prepared!");
}

//Pose Callback
void Navigator::poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg)
{
    this->robot_pose_ = *pose_msg;
    this->robot_position_.x = robot_pose_.pose.position.x;
    this->robot_position_.y = robot_pose_.pose.position.y;
    robot_index_ = pos2idx(robot_position_);

}

//Map inflation callback
void Navigator::inflationCallback(const std_msgs::Int32MultiArrayConstPtr &inflation)
{
    mapdata.grid_inflation_ = inflation->data;
}

//Log Odds occupancy map callback
void Navigator::logoddsCallback(const std_msgs::Int32MultiArrayConstPtr &lo)
{
    mapdata.grid_logodds_ = lo->data;
}

//Goal callback from Mission Planner
void Navigator::goalCallback(const tmsgs::GoalConstPtr &goal)
{
    //check if the arriving goal is the same as our current goal
    int received_goal_id = goal->goal_id;

    if (received_goal_id > current_goal_id_)
    {
        this->current_goal_.setCoords(goal->goal_position.x , goal->goal_position.y);
        this->current_goal_id_ = received_goal_id;


        this->path_.clear();
        this->path_msg_.poses.clear();
        trigger_plan = true;
        if (verbose_){ROS_INFO_STREAM("[Navigator]: New goal received: (" << current_goal_.x << "," << current_goal_.y << ")");};
    }
    // else
    // {
    //     trigger_plan = false; //if we dont get a new goal, we always set trigger plan to false
    // }
}

//Replan callback that gets replan triggers from the Commander
void Navigator::replanCallback(const std_msgs::BoolConstPtr &trigger)
{
    this -> trigger_replan = trigger->data;
    if (trigger_replan)
    {
        if (verbose_){ROS_WARN("[Navigator]: Trigger replan from Commander's Request");};
    }
}


//Main Logic
void Navigator::run()
{
    ros::Rate spinrate(rate_);

    ROS_INFO("[Navigator]: Waiting for topics");

    while(ros::ok() && nh_.param("trigger_nodes" , true) && (robot_position_.x == -500 
                                                            || mapdata.grid_inflation_.empty() 
                                                            || mapdata.grid_logodds_.empty() 
                                                            || current_goal_.x == -1000)
                                                            || !trigger_plan) //safeguard, make sure everythign that needs to load has loaded
    {
        spinrate.sleep();
        ros::spinOnce();
    }
    ROS_INFO("[Navigator]: All topics received!");

    
    ROS_INFO("[Navigator]: Starting Navigator!");
    while(ros::ok() && nh_.param("trigger_nodes" , true))
    {
         //process a round of callbacks
        ros::spinOnce();

        /*
         * Test robot and goal positions
         * False: On occupied/inflated/oob cell
         * True: On Free Cell
         * Informally, we refer to the statuses as ok(true) or bad(false)
        */
        robot_status_ = testPos(robot_position_); 
        goal_status_ = testPos(current_goal_);

        /*
         * This condition will be entered IFF due to 2 reasons
         * 1. trigger_plan is true
         * 2. trigger_replan is true, set by commander because the path cuts an oob/inflated/occupied cell
        */
        if (trigger_plan || trigger_replan)
        {
            //local scope variables
            bool tp = trigger_plan; 
            bool trp = trigger_replan;

            //Case 1: Robot and Goal are both okay
            if (robot_status_ && goal_status_)
            {
                ROS_INFO_COND(verbose_,"[Navigator]: Main planner planning a path!");
                path_.clear();
                path_ = request_path(robot_position_,current_goal_); //just plan a path. This will generate a new path
                writeToPathMsg();
            }

            //Case 2: Robot okay, goal bad
            else if (robot_status_ && !goal_status_)
            {
                ROS_WARN_COND(verbose_,"[Navigator]: Bad goal received. Finding a new goal!");
                bot_utils::Pos2D updated_goal = request_backup_position(current_goal_);
                
                tmsgs::Goal update_goal_msg;
                update_goal_msg.action = 2;
                update_goal_msg.goal_position.x = updated_goal.x;
                update_goal_msg.goal_position.y = updated_goal.y;
                tmsgs::UpdateTurtleGoal update_goal_srv;
                update_goal_srv.request.to_update = update_goal_msg;

                ROS_WARN_COND(verbose_,"[Navigator]: New Goal Found. Updating Mission Planner!");
                if (update_goal_client_.call(update_goal_srv))
                {
                    ROS_INFO_COND(verbose_,"[Navigator]: Mission Planner updated!");
                }
                else
                {
                    ROS_INFO_COND(verbose_,"[Navigator]: Failed to update mission planner!");
                }
                current_goal_ = updated_goal;

                path_.clear();
                path_ = request_path(robot_position_,current_goal_);
                writeToPathMsg();
            }

            //Case 3: Robot bad, goal okay
            else if (!robot_status_ && goal_status_)
            {
                ROS_WARN_COND(verbose_,"[Navigator]: Robot on Non-Free Cell");
                ROS_WARN_COND(verbose_,"[Navigator]: Finding better robot position!");
                ROS_INFO_COND(verbose_,"ROBOT POSITION: ");
                if (verbose_){robot_position_.print();}
                
                backup_robot_position_ = request_backup_position(robot_position_);
                
                ROS_INFO_COND(verbose_,"BACKUP POSITION: ");
                if (verbose_){backup_robot_position_.print();}
                ROS_WARN_COND(verbose_,"[Navigator]: Using backup robot position!");
                
                backup_mode_ = true;
                path_.clear();
                path_ = request_path(robot_position_,current_goal_);
                writeToPathMsg();
            }
            //Final Case 4: Robot and goal bad
            else if (!robot_status_ && !goal_status_)
            {
                ROS_WARN_COND(verbose_,"[Navigator]: Goal is bad and robot on non-free. Finding backups for both!");
                ROS_WARN_COND(verbose_,"[Navigator]: Finding better robot AND goal positions!");
                
                backup_robot_position_ = request_backup_position(robot_position_);
                bot_utils::Pos2D updated_goal = request_backup_position(current_goal_);
                
                tmsgs::Goal update_goal_msg;
                update_goal_msg.action = 2;
                update_goal_msg.goal_position.x = updated_goal.x;
                update_goal_msg.goal_position.y = updated_goal.y;
                tmsgs::UpdateTurtleGoal update_goal_srv;
                update_goal_srv.request.to_update = update_goal_msg;

                ROS_WARN_COND(verbose_,"[Navigator]: New Goal Found. Updating Mission Planner!");

                if (update_goal_client_.call(update_goal_srv))
                {
                    ROS_INFO_COND(verbose_,"[Navigator]: Mission Planner updated!");
                }

                else
                {
                    ROS_INFO_COND(verbose_,"[Navigator]: Failed to update mission planner!");
                }

                current_goal_ = updated_goal;
                path_.clear();
                path_ = request_path(backup_robot_position_,current_goal_);
                writeToPathMsg();
            }

            //Once all the cases have been checked to generate a path, we check if we have received a valid path
            if (path_.empty())
            {
                if (verbose_){ROS_WARN("[Navigator]: No path found!");};

                if (tp && !trp)
                {
                    trigger_plan = true;
                }
                else if (!tp && trp)
                {
                    trigger_replan = true;
                }
                else if (tp && trp)
                {
                    trigger_plan = true;
                    trigger_replan = true;
                }
            }

            else
            {
                path_pub_.publish(path_msg_);
                path_comm_pub_.publish(path_comm_msg_);

                if (tp && !trp)
                {
                    trigger_plan = false;
                }
                else if (!tp && trp)
                {
                    trigger_replan = false;
                }
                else if (tp && trp)
                {
                    trigger_plan = false;
                    trigger_replan = false;
                }
            }
        }

        else
        {
            if (path_.empty())
            {
                trigger_plan = true;
                if (verbose_){ROS_WARN("[Navigator]: No path found!");};
            }
            else
            {
                
                if (!goal_status_)
                {
                    trigger_plan = true;
                }

                else if (!robot_status_)
                {
                    if (backup_mode_)
                    {
                        path_pub_.publish(path_msg_);
                        path_comm_pub_.publish(path_comm_msg_);
                    }
                    else
                    {
                        trigger_plan = true;
                    }
                    
                }

                else
                {
                    path_pub_.publish(path_msg_);
                    path_comm_pub_.publish(path_comm_msg_);
                    backup_mode_ = false;
                }

            }
        }
        
        spinrate.sleep();
    }
}

//Client requests
void Navigator::request_goal_update(bot_utils::Pos2D& goal_to_update)
{
    tmsgs::Goal update_goal_msg;
    update_goal_msg.action = 2;
    update_goal_msg.goal_position.x = goal_to_update.x;
    update_goal_msg.goal_position.y = goal_to_update.y;
    tmsgs::UpdateTurtleGoal update_goal_srv;
    update_goal_srv.request.to_update = update_goal_msg;

    ROS_WARN_COND(verbose_,"[Navigator]: New Goal Found. Updating Mission Planner!");

    if (update_goal_client_.call(update_goal_srv))
    {
        ROS_INFO_COND(verbose_,"[Navigator]: Mission Planner updated!");
    }

    else
    {
        ROS_INFO_COND(verbose_,"[Navigator]: Failed to update mission planner!");
    }
}

std::vector<bot_utils::Pos2D> Navigator::request_path(bot_utils::Pos2D &start_pos, bot_utils::Pos2D &goal_pos)
{
    geometry_msgs::Point start;
    geometry_msgs::Point goal;
    start.x = start_pos.x;
    start.y = start_pos.y;
    goal.x = goal_pos.x;
    goal.y = goal_pos.y;

    tmsgs::PlanMainPath path_request;
    path_request.request.start_pos = start;
    path_request.request.goal_pos = goal;
    
    std::vector<bot_utils::Pos2D> new_path;

    if (main_planner_client_.call(path_request))
    {
        ROS_WARN("[Navigator]: Plan from Global Planner received!");
        for (auto& pt : path_request.response.path.poses)
        {
            bot_utils::Pos2D path_point;
            path_point.x = pt.pose.position.x;
            path_point.y = pt.pose.position.y;
            new_path.push_back(path_point);
        }
    }

    else
    {
        ROS_WARN("[Navigator]: Main Plan failed");
    }

    return new_path;
}

bot_utils::Pos2D Navigator::request_backup_position(bot_utils::Pos2D &bad_pos)
{
    geometry_msgs::Point bad_point;
    bad_point.x = bad_pos.x;
    bad_point.y = bad_pos.y;
    
    tmsgs::FindFallbackPosition fallback_request;
    fallback_request.request.bad_position = bad_point;

    bot_utils::Pos2D fallback_point;

    if (fallback_planner_client_.call(fallback_request))
    {
        ROS_WARN("[Navigator]: Fallback point from Global Planner received!");

        fallback_point.x = fallback_request.response.fallback_position.x;
        fallback_point.y = fallback_request.response.fallback_position.y;
    }

    else
    {
        ROS_WARN("[Navigator]: Fallback point failed!");
        fallback_point.x = bad_pos.x;
        fallback_point.y = bad_pos.y;
    }

    return fallback_point;
}


/*
Map Utilities
*/

int Navigator::flatten(bot_utils::Index &idx)
{
    return idx.i * mapdata.map_size_.j + idx.j;
}

bool Navigator::oob(bot_utils::Index &idx)
{
    return (idx.i < 0 || idx.i >= mapdata.map_size_.i || idx.j < 0 && idx.j >= mapdata.map_size_.j);
}

bot_utils::Index Navigator::pos2idx(bot_utils::Pos2D &pos)
{
    int i = round((pos.x - mapdata.origin_.x) / mapdata.cell_size_);
    int j = round((pos.y - mapdata.origin_.y) / mapdata.cell_size_);
    return bot_utils::Index(i,j);
}

bot_utils::Pos2D Navigator::idx2pos(bot_utils::Index &idx)
{
    double x = idx.i * mapdata.cell_size_ + mapdata.origin_.x;
    double y = idx.j * mapdata.cell_size_ + mapdata.origin_.y;
    return bot_utils::Pos2D(x,y);
}

bool Navigator::testPos(bot_utils::Pos2D pos)
{
    bot_utils::Index idx = pos2idx(pos);
    int key = flatten(idx);

    if (oob(idx))
    {
        return false; //test fail
    }

    if (mapdata.grid_inflation_.at(key) > 0)
    {
        return false; //in map, but on inflated
    }
    else if (mapdata.grid_logodds_.at(key) > mapdata.lo_thresh_)
    {
        return false; //in map, not inflated but log odds occupied
    }
    else
    {
        return true; //in map, not inflated not lo occupied
    }
}

//Convert internal path store to nav_msgs/path
void Navigator::writeToPathMsg()
{
    path_msg_.poses.clear();
    path_comm_msg_.path.poses.clear();
    for (auto &p : path_)
    {
        geometry_msgs::PoseStamped pt;
        pt.pose.position.x = p.x;
        pt.pose.position.y = p.y;
        path_msg_.poses.push_back(pt);
        path_comm_msg_.path.poses.push_back(pt);
    }
    path_comm_msg_.id = ++(this->path_id_); //everytime we write to message, we update the id
    if (verbose_){ROS_INFO_STREAM("[Navigator]: Current Path Id: " << path_id_);};
}

//loading parameters
bool Navigator::loadParams()
{
    bool status = true;
    if (!nh_.param("cell_size" , this->mapdata.cell_size_ , 0.05))
    {
        ROS_WARN("[Navigator]: Param cell_size not found, defaulting to 0.05");
        status = false;
    }

    if (!nh_.param("log_odds_thresh" , this->mapdata.lo_thresh_ , 10))
    {
        ROS_WARN("[Navigator]: Param log_odds_thresh not found, defaulting to 10");
        status = false;
    }

    if (!nh_.param("log_odds_cap" , this->mapdata.lo_cap_ , 20))
    {
        ROS_WARN("[Navigator]: Param log_odds_cap not found, defaulting to 20");
        status = false;
    }

    if (!nh_.param("nv_rate" , this->rate_ , 25.0))
    {
        ROS_WARN("[Navigator]: Param log_odds_cap not found, defaulting to 20");
        status = false;
    }

    if (!nh_.param("min_x" , this->mapdata.pos_min_.x , -10.0))
    {
        ROS_WARN("[Navigator]: Param min_x not found, defaulting to -10");
        status = false;
    }

    if (!nh_.param("min_y" , this->mapdata.pos_min_.y , -10.0))
    {
        ROS_WARN("[Navigator]: Param min_y not found, defaulting to -10");
        status = false;
    }

    if (!nh_.param("max_x" , this->mapdata.pos_max_.x , 10.0))
    {
        ROS_WARN("[Navigator]: Param max_x not found, defaulting to 10");
        status = false;
    }

    if (!nh_.param("max_y" , this->mapdata.pos_max_.y , 10.0))
    {
        ROS_WARN("[Navigator]: Param max_y not found, defaulting to 10");
        status = false;
    }

    if (!nh_.param("verbose_navigator" , this->verbose_ , false))
    {
        ROS_WARN("[Navigator]: Param verbose_planner not found, defaulting to false");
        status = false;
    }



    return status;
}