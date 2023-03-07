#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <errno.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "xmlrpcpp/XmlRpc.h"
#include <opencv2/core/core.hpp>
#include "common.hpp"
#define NaN std::numeric_limits<double>::quiet_NaN()

enum HectorState
{
    TAKEOFF,
    LAND,
    TURTLE,
    START,
    GOAL
};
std::string to_string(HectorState state)
{
    switch (state)
    {
    case TAKEOFF:
        return "TAKEOFF";
    case LAND:
        return "LAND";
    case TURTLE:
        return "TURTLE";
    case START:
        return "START";
    case GOAL:
        return "GOAL";
    default:
        return "??";
    }
}

bool verbose;
double initial_x, initial_y, initial_z;
double x = NaN, y = NaN, z = NaN, a = NaN;
void cbHPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    auto &p = msg->pose.pose.position;
    x = p.x;
    y = p.y;
    z = p.z;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    a = atan2(siny_cosp, cosy_cosp);
}
double turtle_x = NaN, turtle_y = NaN;
void cbTPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    turtle_x = p.x;
    turtle_y = p.y;
}
double vx = NaN, vy = NaN, vz = NaN, va = NaN;
void cbHVel(const geometry_msgs::Twist::ConstPtr &msg)
{
    vx = msg->linear.x;
    vy = msg->linear.y;
    vz = msg->linear.z;
    va = msg->angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_main");
    ros::NodeHandle nh;

    // Make sure motion and move can run (fail safe)
    nh.setParam("run", true); // turns off other nodes

    double main_iter_rate;
    if (!nh.param("main_iter_rate", main_iter_rate, 25.0))
        ROS_WARN(" HMAIN : Param main_iter_rate not found, set to 25");
    if (!nh.param("initial_x", initial_x, 0.0))
        ROS_WARN(" HMAIN : Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", initial_y, 0.0))
        ROS_WARN(" HMAIN : Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", initial_z, 0.178))
        ROS_WARN(" HMAIN : Param initial_z not found, set initial_z to 0.178");
    double height;
    if (!nh.param("height", height, 2.0))
        ROS_WARN(" HMAIN : Param initial_z not found, set to 5");
    double look_ahead;
    if (!nh.param("look_ahead", look_ahead, 1.0))
        ROS_WARN(" HMAIN : Param look_ahead not found, set to 1");
    double close_enough;
    if (!nh.param("close_enough", close_enough, 0.1))
        ROS_WARN(" HMAIN : Param close_enough not found, set to 0.1");
    double average_speed;
    if (!nh.param("average_speed", average_speed, 2.0))
        ROS_WARN(" HMAIN : Param average_speed not found, set to 2.0");
    if (!nh.param("verbose_main", verbose, true))
        ROS_WARN(" HMAIN : Param verbose_main not found, set to false");
    // get the final goal position of turtle
    std::string goal_str;
    double goal_x = NaN, goal_y = NaN;

    if (nh.hasParam("/turtle/goals"))
    {
        XmlRpc::XmlRpcValue goal_loader;
        nh.getParam("/turtle/goals",goal_loader);

        if (goal_loader.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            int total_goals = goal_loader.size();
            goal_x = goal_loader[total_goals - 1][0];
            goal_y = goal_loader[total_goals - 1][1];
        }
    }

    else
    {
        ROS_ERROR("PARAM NOT FOUND!");
        ros::shutdown();
        return 1;
    }
    
    //     ROS_WARN(" HMAIN : Param goal not found, set to %s", goal_str.c_str());

    // --------- Subscribers ----------
    ros::Subscriber sub_hpose = nh.subscribe("pose", 1, &cbHPose);
    ros::Subscriber sub_tpose = nh.subscribe("/turtle/pose", 1, &cbTPose);
    ros::Subscriber sub_hvel = nh.subscribe("velocity", 1, &cbHVel);

    // --------- Publishers ----------
    ros::Publisher pub_target = nh.advertise<geometry_msgs::PointStamped>("target", 1, true);
    geometry_msgs::PointStamped msg_target;
    msg_target.header.frame_id = "world";
    ros::Publisher pub_rotate = nh.advertise<std_msgs::Bool>("rotate", 1, true);
    std_msgs::Bool msg_rotate;
    ros::Publisher pub_traj = nh.advertise<nav_msgs::Path>("trajectory", 1, true);
    nav_msgs::Path msg_traj;
    msg_traj.header.frame_id = "world";

    // --------- Wait for Topics ----------
    while (ros::ok() && nh.param("run", true) && (std::isnan(x) || std::isnan(turtle_x) || std::isnan(vx))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce();                                                                                    // update the topics

    // --------- Main loop ----------
    ROS_INFO(" HMAIN : ===== BEGIN =====");
    HectorState state = TAKEOFF;
    ros::Rate rate(main_iter_rate);
    while (ros::ok() && nh.param("run", true))
    {
        // get topics
        ros::spinOnce();

        /* // remove this block comment (1/2)

        //// IMPLEMENT ////
        
        if (state == TAKEOFF)
        {   // Initial State
            // Disable Rotate
            msg_rotate.data = false;
            pub_rotate.publish(msg_rotate)

            // Enable Rotate when the height is reached
        }
        else if (state == TURTLE)
        {   // flying to turtlebot
            
        }
        else if (state == START)
        {   // flying to hector's starting position
            if (!nh.param("/turtle/run", false))
            { // use this if else case to track when the turtle reaches the final goal
                state = LAND;
            }
        }
        else if (state == GOAL)
        {   // flying to goal
            
        }
        else if (state == LAND)
        {   // reached hector's starting position, and trying to land. Can disable rotation.
            
        }

        */ // remove this block comment (2/2)

        if (verbose)
            ROS_INFO_STREAM(" HMAIN : " << to_string(state));

        rate.sleep();
    }

    nh.setParam("run", false); // turns off other nodes
    ROS_INFO(" HMAIN : ===== END =====");
    return 0;
}
