#include "mission_planner.h"

MissionPlanner::MissionPlanner(ros::NodeHandle &nh)
{
    nh_ = nh;
    bool status = loadParams();
    ROS_WARN_COND(!status , "[MissionPlanner]: Params not loaded properly!");
    
    //Load all the waypoints
    preset_waypoint_status_ = loadPresetWaypoints();

    //Initialise all states
    robot_position_.x = -500;
    robot_position_.y = -500;

    brake_state_ = 0;

    goal_id_ = 0;

    //Setup Publishers
    goal_pub_ = nh_.advertise<tmsgs::Goal>("goal",1);

    goal_visualization_pub_ = nh_.advertise<geometry_msgs::PointStamped>("goal_viz",1);

    //Setup Subcribers
    pose_sub_ = nh_.subscribe("pose" , 1 , &MissionPlanner::poseCallback , this);

    single_goal_sub_ = nh_.subscribe("/single_goal",1,&MissionPlanner::singleGoalCallback,this);
    
    //Enable server to update goals triggered by Navigator
    update_goal_server_ = nh_.advertiseService("update_t_goal",&MissionPlanner::updateGoalService,this);

    //Setup brake client
    brake_client_ = nh_.serviceClient<tmsgs::Brake>("brake_tbot");
    
    /*
    If no goals are loaded, we must wait for RVIZ goal. However, this
    is unlikely to happen because XMLRPC will throw an exception that
    causes roslaunch to fail immediately. So goals (atleast 1) must be
    given
    */
    if (goals_.size() == 0)
    {
        ROS_WARN("[MissionPlanner]: No Preset Waypoint Provided. Please use RVIZ to send goals!");
    }

    else
    {
        ROS_INFO("###WAYPOINTS###");
        int cnt = 0;
        for (auto &gl : goals_)
        {
            ROS_INFO_STREAM("Waypoint " << cnt << ": (" << gl.x << "," << gl.y<<")");
            cnt++;
        }
    }

    ROS_INFO("###WAYPOINTS###");

    ROS_INFO_STREAM("[Mission Planner]: Mission Planner Prepared!");
}

void MissionPlanner::poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg)
{
    robot_pose_ = *pose_msg;
    robot_position_.setCoords(robot_pose_.pose.position.x , robot_pose_.pose.position.y);
    
    /*
    Place the robot's current position to the goals list.
    This will happen exactly once, and this dummy goal will be consumed immediately by
    the mission planner before waiting for the next dynamically set goal.
    */
    if (!preset_waypoint_status_)
    {
        //set to true because of the dummy position
        preset_waypoint_status_ = true;
        goals_.push_back(robot_position_);
    }
}

void MissionPlanner::singleGoalCallback(const geometry_msgs::PoseStampedConstPtr &single_goal_msg)
{
    /*
    Based on rviz clicks (nav_goal click tool), update clicked point in 2D cartesian space
    as a goal
    */

    bot_utils::Pos2D single_goal;
    single_goal.x = single_goal_msg->pose.position.x;
    single_goal.y = single_goal_msg->pose.position.y;
    goals_.push_back(single_goal);
    
    //Release the brakes if they were enabled, because we are ready to plan now
    if (brake_state_)
    {
        brake_state_ = 0;
        tmsgs::Brake brake_srv;
        brake_srv.request.brake_mode = brake_state_;
        while (!brake_client_.call(brake_srv))
        {
            ROS_INFO("[MissionPlanner]: Releasing Brakes!");
        }
    }

    ROS_INFO("[MissionPlanner]: Single goal received!");
}

bool MissionPlanner::updateGoalService(tmsgs::UpdateTurtleGoal::Request &req , tmsgs::UpdateTurtleGoal::Response &res)
{
    /*
    Action 1: Refers to the action of adding a goal infront of the current goal
    Action 2: Replace the top goal, which is deemed invalid by Navigator, with a new goal
              sent by the Navigator
    */
    bot_utils::Pos2D updated_goal(req.to_update.goal_position.x , req.to_update.goal_position.y);
    int action = req.to_update.action;
    if (action == 1)
    {
        ROS_INFO("[MissionPlanner]: Goal added to goals!");
        goals_.push_front(updated_goal);
        goal_id_++;
    }

    else if (action == 2)
    {
        ROS_INFO("[MissionPlanner]: Goal replaced!");
        if(goals_.empty())
        {
            goals_.push_back(updated_goal);
        }
        else
        {
            goals_.at(0) = updated_goal;
        }
        
    }
    res.response = true;
    return true;
}

void MissionPlanner::run()
{
    ros::Rate spinrate(rate_);
    ROS_INFO("[Mission Planner]: Waiting for topics");
    
    //Wait until all the topics have started
    while (ros::ok() && nh_.param("trigger_nodes" , true) && goals_.empty() && robot_position_.x == -500);
    {
        ros::spinOnce();
        spinrate.sleep();
    }

    ROS_INFO("[Mission Planner]: Starting Mission Planner");

    //Main Loop
    while(ros::ok() && nh_.param("trigger_nodes" , true))
    {
        ros::spinOnce();

        //If the goals are empty, we need to wait for a new goal to be clicked on rviz
        //So pump the brakes and wait!
        if (goals_.empty())
        {
            if (!brake_state_)
            {
                brake_state_ = 1;
                tmsgs::Brake brake_srv;
                brake_srv.request.brake_mode = brake_state_;
                while (!brake_client_.call(brake_srv))
                {
                    ROS_INFO("[MissionPlanner]: Braking!");
                }
            }
            continue;
            
        }

        //Check if the euclidean distance to the goal is less than goal_radius
        double dist_to_goal = bot_utils::dist_euc(robot_position_,goals_.front());
        if (dist_to_goal < goal_radius_)
        {
            //If it is, pop the goal to get the next goal
            ROS_INFO_STREAM("[Mission Planner]: Goal " << "Reached!");
            goals_.pop_front();
            goal_id_ ++;

            if (goals_.empty())
            {
                continue;
            }
        }

        //Custom Goal message to send to Navigator
        tmsgs::Goal goal;
        goal.goal_position.x = goals_.front().x;
        goal.goal_position.y = goals_.front().y;
        goal.goal_id = goal_id_;

        //Goal message for RVIZ Visualization
        geometry_msgs::PointStamped current_goal_viz_msg;
        current_goal_viz_msg.header.frame_id = "world";
        current_goal_viz_msg.point.x = goal.goal_position.x;
        current_goal_viz_msg.point.y = goal.goal_position.y;
        
        //Publish goal
        goal_pub_.publish(goal);
        goal_visualization_pub_.publish(current_goal_viz_msg);
        
        spinrate.sleep();
    }
    ROS_INFO("[Mission Planner]: Shutting down all nodes!");
    nh_.setParam("trigger_nodes" , false);
}


bool MissionPlanner::loadParams()
{
    /*
    Loading param routine, common usage in this work
    */
    bool status = true;
    if (!nh_.param("mp_rate" , this->rate_ , 25.0))
    {
        ROS_WARN("[MissionPlanner]: Param cell_size not found, defaulting to 25.0");
        status = false;
    }

    if (!nh_.param("goal_radius" , this->goal_radius_, 0.1))
    {
        ROS_WARN("[MissionPlanner]: Param goal_radius not found, defaulting to 0.1");
        status = false;
    }

    return status;
}

bool MissionPlanner::loadPresetWaypoints()
{
    /*
    Load the preset waypoints, return false if it fails.
    */
    if (!nh_.hasParam("goals"))
    {
        ROS_WARN("[MissionPlanner]: Preset Waypoints not found!");
        return false;
    }

    try
    {
        nh_.getParam("goals",goal_loader_);
        if (goal_loader_.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for (int i = 0 ; i < goal_loader_.size() ; ++i)
            {
                XmlRpc::XmlRpcValue ob = goal_loader_[i];

                bot_utils::Pos2D my_goal;
                my_goal.x = ob[0];
                my_goal.y = ob[1];
                goals_.push_back(my_goal);
            }
        }

        if (goals_.empty())
        {
            ROS_WARN("[MissionPlanner]: PresetWaypoints empty!");
            return false;
        }
        return true;
    }

    catch (const std::exception &e)
    {
        std::cout<< e.what();
        return false;
    }
}