#include "commander.h"

Commander::Commander(ros::NodeHandle &nh)
{
    nh_ = nh;
    bool map_loader = loadMapParams();
    bool pid_loader = loadPIDParams();
    bool traj_loader = loadTrajParams();
    bool comm_loader = loadCommanderParams();
    
    ROS_WARN_COND(!map_loader , "[Commander]: Map parameters have not been loaded correctly!");
    ROS_WARN_COND(!pid_loader , "[Commander]: PID parameters have not been loaded correctly!");
    ROS_WARN_COND(!traj_loader , "[Commander]: Map parameters have not been loaded correctly!");
    ROS_WARN_COND(!comm_loader , "[Commander]: Map parameters have not been loaded correctly!");
    ROS_INFO_COND(map_loader + pid_loader + traj_loader + comm_loader == 4 , "[Commander]: All parameters loaded successfully!");

    //initialise robot variables
    robot_position_.setCoords(-500,-500); //-500 is a very unlikely value
    robot_heading_ = -500;//-500 is an unlikely value

    //setup map variables
    mapdata.origin_ = mapdata.pos_min_;
    mapdata.map_size_.i = std::round((mapdata.pos_max_.x - mapdata.pos_min_.x) / mapdata.cell_size_);
    mapdata.map_size_.j = std::round((mapdata.pos_max_.y - mapdata.pos_min_.y) / mapdata.cell_size_);
    mapdata.total_cells_ = mapdata.map_size_.i * mapdata.map_size_.j;

    //set up grid arrays
    mapdata.grid_inflation_.resize(mapdata.total_cells_);
    mapdata.grid_logodds_.resize(mapdata.total_cells_);
    
    //setup triggers
    trigger_replan_ = false;
    generate_trajectory_ = false;

    //prepare the local planner
    local_planner.updateParams(target_dt_ , average_speed_ , traj_type_);

    //prepare messgaes
    traj_msg_.header.frame_id = "world";
    target_msg_.header.frame_id = "world";

    //setup subscribers
    pose_sub_ = nh_.subscribe("pose" , 1 , &Commander::poseCallback , this);
    inflation_sub_ = nh_.subscribe("grid/mapinfo/inflation" , 1 , &Commander::inflationCallback , this);
    lo_sub_ = nh_.subscribe("grid/mapinfo/logodds" , 1 , &Commander::logoddsCallback , this);
    path_sub_ = nh_.subscribe("path" , 1 , &Commander::pathCallback , this);

    //setup publishers
    target_pub_ = nh_.advertise<geometry_msgs::PointStamped>("target" , 1 , true);
    traj_pub_ = nh_.advertise<nav_msgs::Path>("trajectory" , 1 , true);
    replan_pub_ = nh_.advertise<std_msgs::Bool>("trigger_replan" , 1 ,true);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel" , 1 ,true);

    ROS_INFO("[Commander]: Commander prepared!");
}

void Commander::poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg)
{
    this->robot_pose_ = *pose_msg;
    this->robot_position_.x = robot_pose_.pose.position.x;
    this->robot_position_.y = robot_pose_.pose.position.y;
    this->robot_heading_ = this->robot_heading_ = bot_utils::headingFromQuat(robot_pose_);
}

void Commander::inflationCallback(const std_msgs::Int32MultiArrayConstPtr &inflation)
{
    mapdata.grid_inflation_ = inflation->data;
}

void Commander::logoddsCallback(const std_msgs::Int32MultiArrayConstPtr &lo)
{
    mapdata.grid_logodds_ = lo->data;
}

void Commander::pathCallback(const nav_msgs::PathConstPtr &path)
{
    
    bool check_if_same = true; //returns true if our current path and received path are the same

    int msg_size = path->poses.size();
    int curr_size = path_.size();

    if (msg_size != curr_size) //if the 2 messages have different sizes, they are definitely diff paths
    {
        check_if_same = false;
    }

    else //there is a good chance that 2 paths with same sizes are different. Need to check for this
    {   
        for (int i = 0 ; i < msg_size ; ++i)
        {
            bot_utils::Pos2D msg_pos(path->poses.at(i).pose.position.x , path->poses.at(i).pose.position.y);

            if (path_.at(i) != msg_pos)
            {
                check_if_same = false;
                break; //We just need to show that atleast one element in the corr. index is diff for a diff path
            }
        }
    }

    if (!check_if_same) //if not the same, a new path has been received, EITHER from a new goal OR a replan request from Commander
    {
        ROS_INFO("[Commander]: New Path Received!");
        path_.clear();
        for (auto &pos : path->poses)
        {
            path_.emplace_back(pos.pose.position.x , pos.pose.position.y);
        }
        
        if (current_traj_state_ == TrajState::GOOD)
        {
            ROS_INFO("[Commander]: Path Updated!");
            trigger_replan_ = false;
            generate_trajectory_ = true;
        }
        else if (current_traj_state_ == TrajState::BAD) //this is the case where replan request was sent.
        {
            ROS_INFO("[Commander]: Request for new path fulfilled. Path Updated!");
            trigger_replan_ = false; //reset the replan. Only the main loop has the power to turn it on
            generate_trajectory_ = true;
        }
    }
}

void Commander::run()
{
    ros::Rate spinrate(rate_);

    while (ros::ok() && nh_.param("trigger_nodes" , true) && (robot_position_.x == -500 
                                                              || mapdata.grid_inflation_.empty()
                                                              || mapdata.grid_logodds_.empty()
                                                              || path_.empty()))
    {
        spinrate.sleep();
        ros::spinOnce();
        ROS_INFO("[Commander]: Waiting for topics");
    }

    
    trajectory_ = local_planner.generateTrajectory(path_); //generate initial trajectory
    current_target_ = trajectory_.front(); //extract the first point to go to

    Controller pid(pid_params_);
    double t = ros::Time::now().toSec();
    pid.prepareController(robot_position_ , robot_heading_ , current_target_,t);

    while (ros::ok() && nh_.param("trigger_nodes",true))
    {
        ros::spinOnce();//process a round of callbacks
        double t_loop = ros::Time::now().toSec();
        bool time_ok = pid.updateDT(t_loop);
        if (!time_ok)
        {
            continue;
        }

        if (generate_trajectory_)
        {
            trajectory_.clear();
            trajectory_ = local_planner.generateTrajectory(path_);
            current_target_ = trajectory_.front();
            generate_trajectory_ = false;
            ROS_INFO("[Commander]: New Trajectory Generated");
            writeTrajMsg();
        }
        current_traj_state_ = checkTrajectory() ? TrajState::GOOD : TrajState::BAD;

        if (current_traj_state_ == TrajState::GOOD)
        {
            double lin_vel = 0;
            double ang_vel = 0;

            bool target_reached = checkDist();
            if (target_reached)
            {
                if (trajectory_.size() <=1)
                {
                    trigger_replan_ = 1;
                    std::tie(lin_vel,ang_vel) = pid.generateCmdSignal(robot_position_ , robot_heading_ , current_target_);
                }
                else
                {
                    trajectory_.pop_front();
                    current_target_ = trajectory_.front();
                    std::tie(lin_vel,ang_vel) = pid.generateCmdSignal(robot_position_ , robot_heading_ , current_target_);
                }
            }

            else
            {
                std::tie(lin_vel,ang_vel) = pid.generateCmdSignal(robot_position_ , robot_heading_ , current_target_);
            }
            target_msg_.point.x = current_target_.x;
            target_msg_.point.y = current_target_.y;
            cmd_vel_msg_.linear.x = lin_vel;
            cmd_vel_msg_.angular.z = ang_vel;
            replan_msg_.data = trigger_replan_;

            traj_pub_.publish(traj_msg_);
            target_pub_.publish(target_msg_);
            cmd_vel_pub_.publish(cmd_vel_msg_);
            replan_pub_.publish(replan_msg_);
        }
        else
        {
            double lin_vel = 0;
            double ang_vel = 0;
            bool target_reached = checkDist();

            if (target_reached)
            {
                if (trajectory_.size() > 1)
                {
                    trajectory_.pop_front();
                    current_target_ = trajectory_.front();
                    std::tie(lin_vel , ang_vel) = pid.generateCmdSignal(robot_position_ , robot_heading_ , current_target_);
                }
            }
            else
            {
                std::tie(lin_vel , ang_vel) = pid.generateCmdSignal(robot_position_ , robot_heading_ , current_target_);
            }
            trigger_replan_ = 1;

            target_msg_.point.x = current_target_.x;
            target_msg_.point.y = current_target_.y;
            cmd_vel_msg_.linear.x = lin_vel;
            cmd_vel_msg_.angular.z = ang_vel;
            replan_msg_.data = trigger_replan_;

            traj_pub_.publish(traj_msg_);
            target_pub_.publish(target_msg_);
            cmd_vel_pub_.publish(cmd_vel_msg_);
            replan_pub_.publish(replan_msg_);
        }

        spinrate.sleep();
    }
}

bool Commander::checkDist()
{
    double dist = bot_utils::dist_euc(robot_position_ , current_target_);
    if (dist < close_enough_)
    {
        return true;
    }
    return false;
}

bool Commander::checkTrajectory() //returns false if bad, true if okay
{
    if (trajectory_.size() == 0)
    {
        return false; //trajectory empty
    }

    else if (trajectory_.size() == 1)
    {
        bot_utils::Index to_check = pos2idx(trajectory_.at(0));
        bool status = checkCell(to_check);
        return status;
    }
    else
    {
        
        for (auto &tgt : trajectory_) //we only need to show that 1 point fails for the whole trajectory to fail
        {
            bot_utils::Index to_check = pos2idx(tgt);
            if (!checkCell(to_check))
            {
                return false;
            }
        }
        return true;
    }
}

bool Commander::checkCell(bot_utils::Index &idx)
{
    if (oob(idx))
    {
        return false;
    }

    int k = flatten(idx);

    if (mapdata.grid_inflation_.at(k) > 0)
    {
        return false; //in map, inflated
    }

    else if (mapdata.grid_logodds_.at(k) > mapdata.lo_thresh_)
    {
        return false; //not inflated, lo occuped
    }

    else
    {
        return true;
    }
}

bot_utils::Index Commander::pos2idx(bot_utils::Pos2D &pos)
{
    int i = round((pos.x - mapdata.origin_.x) / mapdata.cell_size_);
    int j = round((pos.y - mapdata.origin_.y) / mapdata.cell_size_);
    return bot_utils::Index(i,j);
}


bool Commander::oob(bot_utils::Index &idx)
{
    return (idx.i <=0 || idx.i >= mapdata.map_size_.i || idx.j < 0 && idx.j >= mapdata.map_size_.j);
}

int Commander::flatten(bot_utils::Index &idx)
{
    int flat = idx.i * mapdata.map_size_.j + idx.j;
    return flat;
}

void Commander::writeTrajMsg()
{
    traj_msg_.poses.clear();
    for (auto& tgt : trajectory_)
    {
        geometry_msgs::PoseStamped tgt_pse;
        tgt_pse.pose.position.x = tgt.x;
        tgt_pse.pose.position.y = tgt.y;
        traj_msg_.poses.push_back(tgt_pse);
    }

}

//Loading parameters
bool Commander::loadMapParams()
{
    bool status = true;
    if (!nh_.param("cell_size" , this->mapdata.cell_size_ , 0.05))
    {
        ROS_WARN("[Commander]: Param cell_size not found, defaulting to 0.05");
        status = false;
    }

    if (!nh_.param("log_odds_thresh" , this->mapdata.lo_thresh_ , 10))
    {
        ROS_WARN("[Commander]: Param log_odds_thresh not found, defaulting to 10");
        status = false;
    }

    if (!nh_.param("log_odds_cap" , this->mapdata.lo_cap_ , 20))
    {
        ROS_WARN("[Commander]: Param log_odds_cap not found, defaulting to 20");
        status = false;
    }

    if (!nh_.param("min_x" , this->mapdata.pos_min_.x , -10.0))
    {
        ROS_WARN("[Commander]: Param min_x not found, defaulting to -10");
        status = false;
    }

    if (!nh_.param("min_y" , this->mapdata.pos_min_.y , -10.0))
    {
        ROS_WARN("[Commander]: Param min_y not found, defaulting to -10");
        status = false;
    }

    if (!nh_.param("max_x" , this->mapdata.pos_max_.x , 10.0))
    {
        ROS_WARN("[Commander]: Param max_x not found, defaulting to 10");
        status = false;
    }

    if (!nh_.param("max_y" , this->mapdata.pos_max_.y , 10.0))
    {
        ROS_WARN("[Commander]: Param max_y not found, defaulting to 10");
        status = false;
    }
    return status;
}

bool Commander::loadPIDParams()
{
    bool status = true;
    if (!nh_.param("Kp_lin" , this->pid_params_.KP_LIN , 2.4))
    {
        ROS_WARN("[Commander]: Param Kp_lin not found, defaulting to 0.05");
        status = false;
    }

    if (!nh_.param("Ki_lin" , this->pid_params_.KI_LIN , 0.0))
    {
        ROS_WARN("[Commander]: Param Ki_lin not found, defaulting to 0");
        status = false;
    }

    if (!nh_.param("Kd_lin" , this->pid_params_.KD_LIN , 0.2))
    {
        ROS_WARN("[Commander]: Param Kd_Lin not found, defaulting to 0.2");
        status = false;
    }

    if (!nh_.param("max_lin_vel" , this->pid_params_.max_lin_vel , 0.18))
    {
        ROS_WARN("[Commander]: Param max_lin_vel not found, defaulting to 0.18");
        status = false;
    }

    if (!nh_.param("max_lin_acc" , this->pid_params_.max_lin_acc , 1.0))
    {
        ROS_WARN("[Commander]: Param max_lin_acc not found, defaulting to 1.0");
        status = false;
    }

    if (!nh_.param("Kp_ang" , this->pid_params_.KP_ANG , 1.3))
    {
        ROS_WARN("[Commander]: Param Kp_ang not found, defaulting to 1.3");
        status = false;
    }

    if (!nh_.param("Ki_ang" , this->pid_params_.KI_ANG , 0.0))
    {
        ROS_WARN("[Commander]: Param Ki_ang not found, defaulting to 0");
        status = false;
    }

    if (!nh_.param("Kd_ang" , this->pid_params_.KD_ANG , 0.3))
    {
        ROS_WARN("[Commander]: Param Kd_ang not found, defaulting to 0.3");
        status = false;
    }

    if (!nh_.param("max_ang_vel" , this->pid_params_.max_ang_vel , 2.80))
    {
        ROS_WARN("[Commander]: Param max_ang_vel not found, defaulting to 2.80");
        status = false;
    }

    if (!nh_.param("max_ang_acc" , this->pid_params_.max_ang_acc , 4.0))
    {
        ROS_WARN("[Commander]: Param max_ang_acc not found, defaulting to 4.0");
        status = false;
    }

    if (!nh_.param("damping_limit" , this->pid_params_.damping_limit , 15.0))
    {
        ROS_WARN("[Commander]: Param damping_limit not found, defaulting to 15.0");
        status = false;
    }

    if (!nh_.param("reverse_limit" , this->pid_params_.reverse_limit , 90.0))
    {
        ROS_WARN("[Commander]: Param reverse_limit not found, defaulting to 90");
        status = false;
    }

    return status;
}

bool Commander::loadTrajParams()
{
    bool status = true;
    if (!nh_.param("target_dt" , target_dt_ , 0.04))
    {
        ROS_WARN("[Commander]: Param target_dt not found, defaulting to 0.04");
        status = false;
    }

    if (!nh_.param("average_speed" , average_speed_ , 0.16))
    {
        ROS_WARN("[Commander]: Param average_speed not found, defaulting to 0.16");
        status = false;
    }

    if (!nh_.param<std::string>("traj_type" , traj_type_ , "cubic"))
    {
        ROS_WARN("[Commander]: Param traj_type not found, defaulting to cubic");
        status = false;
    }

    return status;
}

bool Commander::loadCommanderParams()
{
    bool status = true;
    if (!nh_.param("com_rate" , rate_ , 25.0))
    {
        ROS_WARN("[Commander]: Param com_rate not found, defaulting to 0.04");
        status = false;
    }

    if (!nh_.param("enable_commander" , enable_commander_, true))
    {
        ROS_WARN("[Commander]: Param average_speed not found, defaulting to true");
        status = false;
    }

    if (!nh_.param("close_enough" , close_enough_ , 0.05))
    {
        ROS_WARN("[Commander]: Param close_enough not found, defaulting to 0.05");
        status = false;
    }

    return status;
}

