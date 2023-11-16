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
class MissionPlanner
{
private:

    std::deque<bot_utils::Pos2D> goals_;
    double goal_radius_;
    int goal_id_;

    bool preset_waypoint_status_;
    bool brake_state_;

    geometry_msgs::PoseStamped robot_pose_;
    bot_utils::Pos2D robot_position_;

    ros::Publisher goal_pub_;
    ros::Publisher goal_visualization_pub_;

    ros::Subscriber pose_sub_;
    ros::Subscriber single_goal_sub_;

    ros::ServiceServer update_goal_server_;

    ros::ServiceClient brake_client_;

    ros::NodeHandle nh_;
    XmlRpc::XmlRpcValue goal_loader_;

    double rate_;

public:
    MissionPlanner(ros::NodeHandle &nh);
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg);
    void singleGoalCallback(const geometry_msgs::PoseStampedConstPtr &single_goal_msg);

    bool updateGoalService(tmsgs::UpdateTurtleGoal::Request &req, tmsgs::UpdateTurtleGoal::Response &res);
    void run();

    bool loadParams();
    bool loadPresetWaypoints();
};

#endif //TBOT__MISSION_PLANNER_H