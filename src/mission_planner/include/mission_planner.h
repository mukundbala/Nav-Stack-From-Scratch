#ifndef TBOT__MISSION_PLANNER_H
#define TBOT__MISSION_PLANNER_H

#include "ros/ros.h"
#include "bot_utils/bot_utils.h"
#include "tmsgs/Goal.h"
#include "tmsgs/UpdateTurtleGoal.h"
#include "tmsgs/Brake.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "xmlrpcpp/XmlRpc.h"
#include <vector>
#include <deque>

/*
* @info
  The MissionPlanner class handles the following
  * Loading preset waypoints from param server
  * Capturing dynamic goals set on RVIZ
  * Tracking the state of each goal
  * Sending the current goal to the Navigator
  * Updating internal store of goals in case an invalid goal was given.
    Note that this update is triggered in Navigator.
  * Invalid goals refer to those that are on occupied/inflated cells.
*/
class MissionPlanner
{
private:

    //Deque to store goals for indexing and adding to front-back in O(1)
    std::deque<bot_utils::Pos2D> goals_;

    //Goal radius to declare that goal is reached
    double goal_radius_;

    //ID of goal, incremented when new goals arrive
    int goal_id_;

    //The success of loading preset waypoints
    bool preset_waypoint_status_;

    //To trigger the robot to brake while waiting for a new goal to arrive
    bool brake_state_;
    
    //robot state
    geometry_msgs::PoseStamped robot_pose_;
    bot_utils::Pos2D robot_position_;

    /*
    //Publishers
    * goal_pub_ : Publishes the goal to Navigator
    * goal_visualization_pub: Publishes goal to rviz
    */
    ros::Publisher goal_pub_;
    ros::Publisher goal_visualization_pub_;

    /*
    //Subscribers
    * pose_sub_: Subscribes to robot pose
    * single_goal_sub_: Subcribes to a clicked nav_goal on rviz
    */
    ros::Subscriber pose_sub_;
    ros::Subscriber single_goal_sub_;

    /*
    //Servers
    * update_goal_server: Update internal goal store on request from Navigator 
      in case of an invalid goal
      Invalid in this case refers to a goal that is placed on an occupied/inflated cell
    */
    ros::ServiceServer update_goal_server_;

    /*
    //Clients
     * brake_client: Triggers turtlebot to brake to a stop while waiting for a new goal
    */
    ros::ServiceClient brake_client_;

    //ros nodehandle
    ros::NodeHandle nh_;

    //param server goal loader
    XmlRpc::XmlRpcValue goal_loader_;

    //rate of node
    double rate_;

public:

    //Constructor to setup resources and load goals
    MissionPlanner(ros::NodeHandle &nh);

    //Callback to listen to robot's pose
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg);

    //Callback to listen to new (dynamic) goals from rviz from click
    void singleGoalCallback(const geometry_msgs::PoseStampedConstPtr &single_goal_msg);

    //Service Callback that updates internal store of goals triggered by Navigator
    bool updateGoalService(tmsgs::UpdateTurtleGoal::Request &req, tmsgs::UpdateTurtleGoal::Response &res);
    
    //Main Logic
    void run();

    //Loading params from param server
    bool loadParams();

    //Loading preset waypoints from param server
    bool loadPresetWaypoints();
};

#endif //TBOT__MISSION_PLANNER_H