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
#include "bot_utils/bot_utils.h"
#include "tmsgs/TurtleSpline.h"
#include "hector_trajectory.hpp"

#define NaN std::numeric_limits<double>::quiet_NaN()

enum class HectorState : unsigned short
{
    TAKEOFF,
    LAND,
    TURTLE,
    START,
    GOAL
};
            
enum class GoalState : unsigned short
{
    PREDICTION,
    CHASE,
    GOTO
};

std::string_view to_string(HectorState state)
{
    std::array<std::string_view,5> states = {"TAKEOFF" , "LAND" , "TURTLE" , "START" , "GOAL"};
    return states.at(static_cast<unsigned short> (state));
}

std::string_view to_string(GoalState state)
{
    std::array<std::string_view,3> states = {"PREDICTION" , "CHASE" , "GOTO"};
    return states.at(static_cast<unsigned short> (state));
}

bool verbose;
bool kill_switch = false;

//states
HectorState state;
GoalState g_state;

//store for hector's position and orientation about the z axis
bot_utils::Pos3D hector_position(NaN,NaN,NaN);
double hector_heading = NaN;

//store for hector's velocity
bot_utils::Pos3D hector_linear_velocity(NaN,NaN,NaN);
double hector_angular_velocity = NaN;

//store for turtlebot position
bot_utils::Pos2D tbot_position(NaN,NaN);
double tbot_heading = NaN;

//generate trajectory triggers.
//We need 2 to avoid generate trajectories for passthroughs if turtle gets a new traj
bool generate_trajectory_turtle = false;
bool generate_trajectory_passthrough = false;

//Hector current target
bot_utils::Pos3D current_target;

//Hector current goal
bot_utils::Pos3D current_goal(NaN,NaN,NaN); //the store for the current goal we need
bot_utils::Pos3D next_goal(NaN,NaN,NaN);
bot_utils::Pos3D final_goal(NaN,NaN,NaN); //the turtlebots goal
bot_utils::Pos3D hector_initial(NaN,NaN,NaN); //hector's initial goal
bot_utils::Pos3D hector_takeoff_goal(NaN,NaN,NaN); //hector's takeoff goal
bot_utils::Pos3D hector_land_goal(NaN,NaN,NaN); //hector's land goal
bot_utils::Pos2D turtle_predicted_goal(NaN,NaN);
int predicted_id = -1;

//Hector Spline
SplineData3D hector_spline_data;
int h_id = -1;

//store of turtle spline information
SplineData2D turtle_spline_data;


void cbHPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    auto &p = msg->pose.pose.position;
    hector_position.setCoords(p.x,p.y,p.z);

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    hector_heading = atan2(siny_cosp, cosy_cosp);
}

void cbTPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    tbot_position.setCoords(p.x,p.y);

    auto &q = msg->pose.orientation;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    tbot_heading = atan2(siny_cosp, cosy_cosp);
}

void cbHVel(const geometry_msgs::Twist::ConstPtr &msg)
{
    hector_linear_velocity.setCoords(msg->linear.x , msg->linear.y, msg->linear.z);
    hector_angular_velocity = msg->angular.z;
}

void cbTSpline(const tmsgs::TurtleSplineConstPtr &spline_msg)
{
    int received_id = spline_msg->spline_id;
    if (received_id != turtle_spline_data.curr_spline_id)
    {
        turtle_spline_data.curr_spline_id = received_id;
        turtle_spline_data.avg_speed = spline_msg->average_speed;
        turtle_spline_data.target_dt = spline_msg -> target_dt;
        turtle_spline_data.spline.clear();

        auto &p = spline_msg->spline.poses;

        for (auto &m : p)
        {
            turtle_spline_data.spline.emplace_back(m.pose.position.x , m.pose.position.y);
        }

        if (verbose)
        {
            ROS_INFO("[Tmain]: New Turtle Spline Received!");
        }
        generate_trajectory_turtle = true;
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_main");
    ros::NodeHandle nh;

    // Make sure motion and move can run (fail safe)
    nh.setParam("run", true); // turns off other nodes

    double main_iter_rate = 25.0;
    double height = 2.0;
    double look_ahead = 1.0;
    double close_enough = 0.1;
    double average_speed = 2.0;
    double goal_x = NaN;
    double goal_y = NaN;

    if (!nh.param("main_iter_rate", main_iter_rate, 25.0))
        ROS_WARN(" HMAIN : Param main_iter_rate not found, set to 25");
    if (!nh.param("initial_x", hector_initial.x, 0.0))
        ROS_WARN(" HMAIN : Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", hector_initial.y, 0.0))
        ROS_WARN(" HMAIN : Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", hector_initial.z, 0.178))
        ROS_WARN(" HMAIN : Param initial_z not found, set initial_z to 0.178");
    if (!nh.param("height", height, 2.0))
        ROS_WARN(" HMAIN : Param initial_z not found, set to 5");
    if (!nh.param("look_ahead", look_ahead, 1.0))
        ROS_WARN(" HMAIN : Param look_ahead not found, set to 1");
    if (!nh.param("close_enough", close_enough, 0.1))
        ROS_WARN(" HMAIN : Param close_enough not found, set to 0.1");
    if (!nh.param("average_speed", average_speed, 2.0))
        ROS_WARN(" HMAIN : Param average_speed not found, set to 2.0");
    if (!nh.param("verbose_main", verbose, true))
        ROS_WARN(" HMAIN : Param verbose_main not found, set to false");

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
    hector_land_goal.setCoords(hector_initial.x , hector_initial.y , 0.18);
    hector_takeoff_goal.setCoords(hector_initial.x , hector_initial.y , height);
    final_goal.setCoords(goal_x , goal_y , height);
    double target_dt = 0.033;
    hector_spline_data.avg_speed = average_speed;
    hector_spline_data.target_dt = target_dt;
    hector_spline_data.curr_spline_id = -1;
    //Subscribers
    ros::Subscriber sub_hpose = nh.subscribe("pose", 1, &cbHPose);
    ros::Subscriber sub_tpose = nh.subscribe("/turtle/pose", 1, &cbTPose);
    ros::Subscriber sub_hvel = nh.subscribe("velocity", 1, &cbHVel);
    ros::Subscriber sub_tspline = nh.subscribe("/turtle/spline" , 1 , &cbTSpline);

    //Messages to Publish
    geometry_msgs::PointStamped msg_target;
    std_msgs::Bool msg_rotate;
    nav_msgs::Path msg_traj;
    msg_traj.header.frame_id = "world";
    msg_target.header.frame_id = "world";

    //Publishers
    ros::Publisher pub_target = nh.advertise<geometry_msgs::PointStamped>("target", 1, true);
    ros::Publisher pub_rotate = nh.advertise<std_msgs::Bool>("rotate", 1, true);
    ros::Publisher pub_traj = nh.advertise<nav_msgs::Path>("trajectory", 1, true);

    // --------- Wait for Topics ----------
    while (ros::ok() && nh.param("run", true) && (std::isnan(hector_position.x) || std::isnan(tbot_position.x) || 
                                                  std::isnan(hector_linear_velocity.x) || turtle_spline_data.curr_spline_id == -1)) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce();                                                                                    // update the topics

    // --------- Main loop ----------
    ROS_INFO(" HMAIN : ===== BEGIN =====");
    ros::Rate rate(main_iter_rate);

    HectorState state = HectorState::TAKEOFF;
    GoalState g_state = GoalState::CHASE;
    current_goal = hector_takeoff_goal;
    generate_trajectory_passthrough = true;
    next_goal.setCoords(NaN,NaN,NaN);
    //                                                          If turtle ends: LAND
    //                                                                            ^
    //TAKEOFF --> TURTLE --> FINAL GOAL --> START --> TURTLE --> FINAL GOAL --> START ...
    while (ros::ok() && nh.param("run", true))
    {
        
        ros::spinOnce();

        int current_turtle_id = turtle_spline_data.find_pos_id(tbot_position);

        if (state == HectorState::TAKEOFF)
        {   
            msg_rotate.data = false;

            generate_trajectory_turtle = false;

            if (dist_euc(hector_position.x , hector_position.y , current_target.x , current_target.y) && std::abs(hector_position.z - height) < 0.05)
            {
                //transition state
                state = HectorState::TURTLE;
                g_state = GoalState::PREDICTION;
                ROS_INFO("TRANSITION FROM TAKEOFF TO TURTLE");
                std::tie(turtle_predicted_goal,predicted_id) = get_best_goal(turtle_spline_data , hector_position , tbot_position , average_speed); //predict a goal
                current_goal.setCoords(turtle_predicted_goal.x , turtle_predicted_goal.y , height);
                next_goal = final_goal;
                generate_trajectory_turtle = true;
                generate_trajectory_passthrough = false;
            }
        }
        else if (state == HectorState::TURTLE)
        {   
            msg_rotate.data = true;
            generate_trajectory_passthrough = false;

            if (dist_euc(hector_position.x , hector_position.y , tbot_position.x , tbot_position.y) < close_enough) //base check if we are close enough to the turtle
            {
                //transition state
                ROS_INFO("TRANSITION FROM TURTLE TO GOAL");
                state = HectorState::GOAL;
                g_state = GoalState::GOTO;
                current_goal = final_goal;
                next_goal = hector_takeoff_goal;
                generate_trajectory_turtle = false;
                generate_trajectory_passthrough = true;
            }
            else //if we are nowhere close to the turtle yet
            {
                if (generate_trajectory_turtle)
                {
                    //a case where we recived a new trajectory
                    std::tie(turtle_predicted_goal,predicted_id) = get_best_goal(turtle_spline_data , hector_position , tbot_position , average_speed);
                    current_goal.setCoords(turtle_predicted_goal.x , turtle_predicted_goal.y , height);
                    next_goal = final_goal;
                    g_state = GoalState::PREDICTION;
                }

                else //!generate_trajectory_turtle
                {
                    //we have received no new trajectories.
                    if (g_state == GoalState::PREDICTION)
                    {
                        bool turtle_reached_pred = current_turtle_id >= predicted_id;
                        bool hector_reached_pred = bot_utils::dist_euc(hector_position.x , hector_position.y , turtle_predicted_goal.x , turtle_predicted_goal.y) < 0.1;

                        if (turtle_reached_pred && !hector_reached_pred)
                        {
                            std::tie(turtle_predicted_goal,predicted_id) = get_best_goal(turtle_spline_data , hector_position , tbot_position , average_speed);
                            current_goal.setCoords(turtle_predicted_goal.x , turtle_predicted_goal.y , height);
                            next_goal = final_goal;
                            g_state = GoalState::PREDICTION;
                            generate_trajectory_turtle = true;
                        }

                        else if (hector_reached_pred & !turtle_reached_pred)
                        {
                            turtle_predicted_goal = tbot_position;
                            predicted_id = -2; //set this to signal that we are chasing the turtlebot
                            current_goal.setCoords(tbot_position.x , tbot_position.y , height);
                            next_goal = final_goal;
                            g_state = GoalState::CHASE;
                            generate_trajectory_turtle = true;
                        }
                    }
                    
                    else if (g_state == GoalState::CHASE)
                    {
                        if (dist_euc(hector_position.x , hector_position.y , tbot_position.x , tbot_position.y) < close_enough)
                        {
                            ROS_INFO("TRANSITION FROM TURTLE TO GOAL");
                            state = HectorState::GOAL;
                            g_state = GoalState::GOTO;
                            current_goal = final_goal;
                            next_goal = hector_takeoff_goal;
                            generate_trajectory_turtle = false;
                            generate_trajectory_passthrough = true;
                        }
                        else
                        {
                            turtle_predicted_goal = tbot_position;
                            predicted_id = -2;
                            current_goal.setCoords(tbot_position.x , tbot_position.y , height);
                            next_goal = final_goal;
                            g_state = GoalState::CHASE;
                            generate_trajectory_turtle = true;
                        }
                    }
                }
            }
        }

        else if (state == HectorState::GOAL)
        {   
            msg_rotate.data = true;
            generate_trajectory_turtle = false;
            generate_trajectory_passthrough = false;

            if (dist_euc(hector_position.x , hector_position.y , final_goal.x , final_goal.y) < close_enough && std::abs(hector_position.z - height) < 0.1)
            {
                //state transition to START
                ROS_INFO("TRANSITION FROM GOAL TO START");
                state = HectorState::START;
                g_state = GoalState::GOTO;
                current_goal = hector_takeoff_goal;
                next_goal.setCoords(NaN,NaN,NaN);
            }
        }

        else if (state == HectorState::START)
        {   
            msg_rotate.data = true;
            generate_trajectory_turtle = false;
            generate_trajectory_passthrough = false;

            if (!nh.param("/turtle/trigger_nodes", false))
            { 
                if (dist_euc(hector_position.x , hector_position.y , hector_takeoff_goal.x , hector_takeoff_goal.y) < close_enough && std::abs(hector_position.z - height) < 0.1)
                {
                    //transition to landing
                    ROS_INFO("TRANSITION FROM START TO LAND");
                    state = HectorState::LAND;
                    g_state = GoalState::CHASE;
                    current_goal = hector_land_goal;
                    next_goal.setCoords(NaN,NaN,NaN);
                    generate_trajectory_turtle = false;
                    generate_trajectory_passthrough = true;
                }
            }

            else
            {
                if (dist_euc(hector_position.x , hector_position.y , hector_takeoff_goal.x , hector_takeoff_goal.y) < close_enough && std::abs(hector_position.z - height) < 0.1)
                {
                    //state transition
                    ROS_INFO("TRANSITION FROM START TO TURTLE");
                    state = HectorState::TURTLE;
                    g_state = GoalState::PREDICTION;
                    std::tie(turtle_predicted_goal,predicted_id) = get_best_goal(turtle_spline_data , hector_position , tbot_position , average_speed);
                    current_goal.setCoords(turtle_predicted_goal.x , turtle_predicted_goal.y , height);
                    generate_trajectory_turtle = true;
                    generate_trajectory_passthrough = false;
                }
            }
        }

        else if (state == HectorState::LAND)
        {   
            msg_rotate.data = false;
            generate_trajectory_turtle = false;
            generate_trajectory_passthrough = false;
            if (dist_euc(hector_position.x , hector_position.y , hector_land_goal.x , hector_land_goal.y) < 0.05 && std::abs(hector_position.z - hector_land_goal.z) < 0.1)
            {
                ROS_INFO("LANDED SAFELY");
                kill_switch = true;
                ROS_INFO("LANDED SAFELY");
            }
        }

        if (kill_switch)
        {
            break;
        }

        if (generate_trajectory_passthrough && generate_trajectory_turtle)
        {
            ROS_ERROR("SOMETHING WRONG WITH STATE MACHINE");
        }

        if (generate_trajectory_turtle || generate_trajectory_passthrough)
        {
            int hector_state = static_cast<unsigned int>(state);
            int goal_state = static_cast<unsigned int>(g_state);

            
            TrajectoryGenerationHandler(current_goal , 
                                        next_goal ,  
                                        hector_position ,
                                        hector_linear_velocity ,
                                        hector_spline_data, 
                                        average_speed , 
                                        target_dt , 
                                        height, 
                                        close_enough , 
                                        hector_state , goal_state);
            
            msg_traj.poses.clear();
            for (auto &p : hector_spline_data.spline)
            {
                //p.print();
                geometry_msgs::PoseStamped x;
                x.pose.position.x = p.x;
                x.pose.position.y = p.y;
                x.pose.position.z = p.z;

                msg_traj.poses.push_back(x);
            }

            h_id = hector_spline_data.spline.size() - 1; //set to the last index because we generate traj from the back!

            if (hector_spline_data.spline.size() > look_ahead)
            {
                h_id -= look_ahead;
            }
            generate_trajectory_turtle = false;
            generate_trajectory_passthrough = false;
        }

        current_target = hector_spline_data.spline.at(h_id);

        msg_target.point.x = current_goal.x;
        msg_target.point.y = current_goal.y; 
        msg_target.point.z = current_goal.z;
        pub_target.publish(msg_target);
        pub_traj.publish(msg_traj);
        ROS_INFO_STREAM("HECTOR STATE: " << to_string(state));
        ROS_INFO_STREAM("GOAL STATE: " << to_string(g_state));
        // if (verbose)
        //     ROS_INFO_STREAM(" HMAIN : " << to_string(state));

        rate.sleep();
    }

    nh.setParam("run", false); // turns off other nodes
    ROS_INFO(" HMAIN : ===== END =====");
    return 0;
}
