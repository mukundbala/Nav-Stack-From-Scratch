#include "commander.h"

Commander::Commander(ros::NodeHandle &nh)
{
    nh_ = nh;
    bool map_loader = loadMapParams();
    bool pid_loader = loadPIDParams();
    bool traj_loader = loadTrajParams();
    bool comm_loader = loadCommanderParams();
    
    ROS_WARN_COND(!map_loader , "[Commander]:  Map parameters have not been loaded correctly!");
    ROS_WARN_COND(!pid_loader , "[Commander]:  PID parameters have not been loaded correctly!");
    ROS_WARN_COND(!traj_loader , "[Commander]: Map parameters have not been loaded correctly!");
    ROS_WARN_COND(!comm_loader , "[Commander]: Map parameters have not been loaded correctly!");
    ROS_INFO_COND(map_loader + pid_loader + traj_loader + comm_loader == 4 , "[Commander]: All parameters loaded successfully!");

    //initialise robot variables
    robot_position_.setCoords(-500,-500); //-500 is a very unlikely value
    robot_heading_ = -500;//-500 is an unlikely value
    robot_speed_ = 0;

    //Braking
    brake_ = 0;
    
    //setup map variables
    mapdata.origin_ = mapdata.pos_min_;
    mapdata.map_size_.i = std::round((mapdata.pos_max_.x - mapdata.pos_min_.x) / mapdata.cell_size_);
    mapdata.map_size_.j = std::round((mapdata.pos_max_.y - mapdata.pos_min_.y) / mapdata.cell_size_);
    mapdata.total_cells_ = mapdata.map_size_.i * mapdata.map_size_.j;

    //set up grid arrays
    mapdata.grid_inflation_.resize(mapdata.total_cells_);
    mapdata.grid_logodds_.resize(mapdata.total_cells_);
    
    //setup triggers
    generate_trajectory_ = false;
    trigger_replan_ = false;

    //setup path
    curr_path_id = -1;

    //spline stuff
    spline_id_ = 0;
    spline_msg_.spline_id = -1;
    spline_msg_.average_speed = average_speed_;
    spline_msg_.target_dt = target_dt_;
    
    //prepare velocities
    cmd_lin_vel_ = 0;
    cmd_ang_vel_ = 0;

    //prepare messgaes
    traj_msg_.header.frame_id = "world";
    target_msg_.header.frame_id = "world";

    //setup subscribers
    pose_sub_ = nh_.subscribe("pose" , 1 , &Commander::poseCallback , this);
    inflation_sub_ = nh_.subscribe("grid/mapinfo/inflation" , 1 , &Commander::inflationCallback , this);
    lo_sub_ = nh_.subscribe("grid/mapinfo/logodds" , 1 , &Commander::logoddsCallback , this);
    path_sub_ = nh_.subscribe("path_comm" , 1 , &Commander::pathCallback , this);
    speed_sub_ = nh_.subscribe("motion_filter_vel" , 1 , &Commander::motionFilterCallback , this);
    
    //setup publishers
    target_pub_ = nh_.advertise<geometry_msgs::PointStamped>("target" , 1 , true);
    traj_pub_ = nh_.advertise<nav_msgs::Path>("trajectory" , 1 , true);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel" , 1 ,true);
    replan_pub_ = nh_.advertise<std_msgs::Bool>("trigger_replan" , 1 , true);
    spline_pub_ = nh_.advertise<tmsgs::TurtleSpline>("spline" , 1 , true);

    //braking server
    brake_server_ = nh_.advertiseService("brake_tbot", &Commander::brakeServiceCallback,this);
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

void Commander::pathCallback(const tmsgs::TurtlePath &path)
{
    int new_path_id = path.id;
    if (new_path_id != curr_path_id)
    {
        path_.clear();
        
        for (auto &p : path.path.poses)
        {
            path_.emplace_back(p.pose.position.x , p.pose.position.y);
        }
        
        curr_path_id = new_path_id;
        generate_trajectory_ = true;
        trigger_replan_ = false;
        trajectory_.clear();
        if (verbose_){ROS_INFO("[Commander]: New Path Received!");};
    }
}

void Commander::motionFilterCallback(const std_msgs::Float64ConstPtr &spd)
{
    robot_speed_ = spd->data;
}

bool Commander::brakeServiceCallback(tmsgs::Brake::Request &req,tmsgs::Brake::Response &res)
{
    brake_ = req.brake_mode;
    res.response = true;
    return true;
}

void Commander::run()
{
    ros::Rate spinrate(rate_);

    LocalPlanner local_planner(target_dt_ , average_speed_ , traj_type_);
    Controller pid(pid_params_);
    
    ROS_INFO("[Commander]: Waiting for topics");

    while (ros::ok() && nh_.param("trigger_nodes" , true) && (robot_position_.x == -500 
                                                              || mapdata.grid_inflation_.empty()
                                                              || mapdata.grid_logodds_.empty()
                                                              || curr_path_id == -1))
    {
        spinrate.sleep();
        ros::spinOnce();
        
    }

    current_target_ = robot_position_;

    ROS_INFO("[Commander]: Starting Commander!");

    double t_now = ros::Time::now().toSec();
    pid.prepareController(robot_position_ , robot_heading_ , current_target_ , t_now);

    while (ros::ok() && nh_.param("trigger_nodes",true))
    {
        ros::spinOnce();//process a round of callbacks
        double t_now = ros::Time::now().toSec();
        bool time_status = pid.updateDT(t_now);
        if (!time_status)
        {
            spinrate.sleep();
            continue;
        }
        //ROS_INFO_STREAM("[Commander]: DT: " << pid.getDt());
        if (generate_trajectory_)
        {
            if (traj_type_ == "Linear")
            {
                trajectory_ = local_planner.generate_trajectory(path_);
            }

            else
            {
                trajectory_ = local_planner.generate_trajectory(path_ , robot_speed_ , robot_heading_);
            }

            t_id = trajectory_.size() - 1;
            if (t_id > 15)
            {
                t_id -= 15;  //choose a further trajectory
            }

            spline_msg_.spline.poses.clear();

            for (int i = t_id ; i>=0 ; i--) //t_id is the current target of the robot
            {
                geometry_msgs::PoseStamped p;
                p.pose.position.x = trajectory_.at(i).x;
                p.pose.position.y = trajectory_.at(i).y;
                spline_msg_.spline.poses.push_back(p);
            }
            spline_msg_.spline_id = spline_id_; //we have pushed back all the trajectories from the turtle's current target all the way to the final target
            spline_id_ ++;
        }

        auto [safe , bad_idx] = checkTrajectorySafety();
        current_target_ = trajectory_.at(t_id);
        
        if (!safe)
        {
            if (bad_idx == -2)
            {
                if (verbose_){ROS_WARN("[Commander]: Trajectory is empty!");};
                current_target_ = robot_position_; //just write the robot position
                cmd_lin_vel_ = 0;
                cmd_ang_vel_ = 0;
                trigger_replan_ = true;
            }

            else if (bad_idx == -1)
            {
                if (verbose_){ROS_WARN("[Commander]: Only target in Trajectory is bad!");};
                current_target_ = robot_position_; //just write the robot position
                //publish 0 velocity
                cmd_lin_vel_ = 0;
                cmd_ang_vel_ = 0;
                trigger_replan_ = true;
            }

            else
            {
                if (t_id - bad_idx < danger_close_)
                {

                    if (bot_utils::dist_euc(robot_position_,current_target_) < close_enough_)
                    {
                        if (t_id == 0)
                        {
                            current_target_ = trajectory_.at(t_id);
                            
                        }
                        else
                        {
                            t_id --;
                            current_target_ = trajectory_.at(t_id);
                        }
                    }
                    std::tie(cmd_lin_vel_,cmd_ang_vel_) = pid.generate_cmdvel(robot_position_ , robot_heading_ , current_target_);
                    trigger_replan_ = true;
                }

                else
                {
                    if (bot_utils::dist_euc(robot_position_ , current_target_) < close_enough_)
                    {
                        if (t_id == 0)
                        {
                            //this means we are in the last target
                            //publish 0 velocity
                            current_target_ = trajectory_.at(t_id);
                            trigger_replan_ = true;
                        }
                        else
                        {
                            t_id --;
                            current_target_ = trajectory_.at(t_id);
                        }
                    }
                    //generate velocity using PID
                    std::tie(cmd_lin_vel_,cmd_ang_vel_) = pid.generate_cmdvel(robot_position_ , robot_heading_ , current_target_);
                }
            }
        }
        else //safe trajectory
        {
            if (bot_utils::dist_euc(robot_position_ , current_target_) < close_enough_)
            {
                if (t_id == 0)
                {
                    current_target_ = trajectory_.at(t_id); 
                }
                else
                {
                    t_id--;
                    current_target_ = trajectory_.at(t_id);
                }
                trigger_replan_ = false;
            }

            //generate velocity using PID
            std::tie(cmd_lin_vel_,cmd_ang_vel_) = pid.generate_cmdvel(robot_position_ , robot_heading_ , current_target_);
        }
        if (generate_trajectory_)
        {
            writeTrajMsg();
            generate_trajectory_ = false;
        }
        writeTargetMsg();
        writeVelocityMsg();
        writeReplanMsg();
        traj_pub_.publish(traj_msg_);
        target_pub_.publish(target_msg_);
        cmd_vel_pub_.publish(cmd_vel_msg_);
        replan_pub_.publish(replan_msg_);

        if (spline_msg_.spline_id != -1)
        {
            spline_pub_.publish(spline_msg_);
        }
        spinrate.sleep();
    }
    cmd_vel_msg_.angular.z = 0;
    cmd_vel_msg_.linear.x = 0;
    cmd_vel_pub_.publish(cmd_vel_msg_);
    
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

std::pair<bool,int> Commander::checkTrajectorySafety() //returns false if bad, true if okay
{
    std::pair<bool,int> result; //pair returns bool:True(safe) , False(unsafe). Int is the unsafe index, else -1

    if (trajectory_.size() == 0)
    {
        result = {false , -2};
        return result; //unsafe trajectory
    }

    else if (trajectory_.size() == 1)
    {
        bot_utils::Index to_check = pos2idx(trajectory_.at(0));
        bool status = checkCell(to_check);
        if (status)
        {
            result = {true , -1};
            return result;
        }
        else
        {
            result = {false , -1};
            return result;
        }
    }
    else
    {
        for (int i = t_id ; i>=0 ; --i)
        {
            bot_utils::Pos2D cell_to_check = trajectory_.at(i);
            bot_utils::Index idx_to_check = pos2idx(cell_to_check);

            if (!checkCell(idx_to_check))
            {
                result = {false , i}; //ith index is first occuring the bad index
                return result;
            }
        }
    }

    //nothing wrong with the traj
    result = {true , -1};
    return result;
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

void Commander::writeTargetMsg()
{
    target_msg_ .point.x = current_target_.x;
    target_msg_.point.y = current_target_.y;
}

void Commander::writeVelocityMsg()
{
    //braking coefficient
    int braking_coefficient = !brake_;
    cmd_vel_msg_.linear.x = cmd_lin_vel_ * braking_coefficient;
    cmd_vel_msg_.angular.z = cmd_ang_vel_ * braking_coefficient;
}

void Commander::writeReplanMsg()
{
    replan_msg_.data = this->trigger_replan_;
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

    if (!nh_.param<std::string>("damping_function" , this->pid_params_.damping_function , "PieceWise"))
    {
        ROS_WARN("[Commander]: Param damping_function not found, defaulting to PieceWise");
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
        ROS_WARN("[Commander]: Param enable_commander not found, defaulting to true");
        status = false;
    }

    if (!nh_.param("close_enough" , close_enough_ , 0.05))
    {
        ROS_WARN("[Commander]: Param close_enough not found, defaulting to 0.05");
        status = false;
    }

    if (!nh_.param("danger_close" , danger_close_ , 10))
    {
        ROS_WARN("[Commander]: Param danger_close not found, defaulting to 10");
        status = false;
    }

    if (!nh_.param("verbose_commander" , verbose_ , false))
    {
        ROS_WARN("[Commander]: Param verbose_commander not found, defaulting to false");
        status = false;
    }
    return status;
}

