#ifndef HBOT__DRONE_COMMANDER_H
#define HBOT__DRONE_COMMANDER_H

#include "ros/ros.h"

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include "xmlrpcpp/XmlRpc.h"
#include "bot_utils/bot_utils.h"
#include "bot_utils/spline_data.h"
#include "tmsgs/TurtleSpline.h"
#include "hmsgs/Goal.h"

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <errno.h>
#include <signal.h>

#include "trajectory_generator.h"
#include "velocity_controller.h"
#include "velocity_controller_params.h"
#include "mission_states.h"

#define NaN std::numeric_limits<double>::quiet_NaN()


class DroneCommander
{
private:

    //state machine states
    mission_states::HectorState h_state_;
    mission_states::GoalState g_state_;
    
    //solo flight goals
    std::vector<bot_utils::Pos3D> solo_goals_;
    int solo_goal_id_;
    //#############Hector Data########################

    //hector robot position and heading in WORLD FRAME
    bot_utils::Pos3D hector_position_;
    double hector_heading_;

    //hector robot linear and angular velocity in ROBOT FRAME
    bot_utils::Pos3D hector_lin_vel_;
    double hector_ang_vel_;

    //hector spline
    bot_utils::SplineData3D hector_spline_;
    
    //hector goals in WORLD FRAME
    bot_utils::Pos3D current_goal_; //store for current goal
    bot_utils::Pos3D next_goal_; //store for next goal

    bot_utils::Pos3D hector_initial_pos_; //store for initial hector position
    bot_utils::Pos3D hector_takeoff_goal_;
    bot_utils::Pos3D hector_land_goal_; //store for hector's land goals
    bot_utils::Pos3D hector_start_goal_; //store for start goal. Also the takeoff goal
    bot_utils::Pos3D hector_end_goal_;
    bot_utils::Pos3D hector_home_goal_;
    bot_utils::Pos2D hector_pred_goal_; //store for hector's prediction of Tbot position
    double pred_id_; //store for the hector's prediction of the TRAJECTORY ID the turtlebot is on
    
    //current target store of the hector
    bot_utils::Pos3D current_target_;

    //curent id of hector target
    int h_id_;
    //#############Hector Data########################

    //#############Turtle Data########################
    bot_utils::Pos2D turtle_position_;
    double turtle_heading_;

    //store for turtle spline data
    bot_utils::SplineData2D turtle_spline_;
    int ts_id_; //target id where the turlebot is on
    //#############Turtle Data########################

    //commander params
    double cruise_height_;
    double takeoff_height_;
    double land_height_;

    double thresh_cruise_height_;
    double thresh_takeoff_height_;
    double thresh_land_height_;
    double thresh_cruise_planar_;

    int look_ahead_;
    double head_start_;

    //trajectory generator params
    double target_dt_;
    double average_speed_;
    std::string traj_name_;
    bool verbose_traj_;

    //velocity controller params
    ControllerParams controller_params_;
    bool enable_controller_;
    bool verbose_controller_;

    //triggers
    bool gen_traj_turtle_;
    bool gen_traj_passthrough_;
    bool kill_switch_;

    //velocities for the drone in WORLD FRAME
    double vel_x_;
    double vel_y_;
    double vel_z_;

    //Publishers
    ros::Publisher traj_pub_;
    ros::Publisher target_pub_;
    ros::Publisher rotate_pub_;
    ros::Publisher vel_pub_;
    ros::Publisher vel_mag_pub_;    
    ros::Publisher goal_pub_;
    ros::Publisher goal_full_pub_;
    ros::Publisher error_pub_;
    ros::Publisher error_vec_pub_;

    //Subscribers
    ros::Subscriber sub_h_pose_;
    ros::Subscriber sub_h_vel_;
    ros::Subscriber sub_t_pose_;
    ros::Subscriber sub_t_spline_;
    //note to self: Write a subcriber for general goals in solo mode

    //messages to publish
    nav_msgs::Path traj_msg_;
    geometry_msgs::PointStamped target_msg_;
    std_msgs::Bool rotate_msg_;
    geometry_msgs::Twist vel_msg_;
    std_msgs::Float64 vel_mag_msg_;
    geometry_msgs::PointStamped goal_msg_;
    hmsgs::Goal goal_full_msg_;
    std_msgs::Float64 error_msg_;
    geometry_msgs::Point error_vec_msg;

    //setup service call to trigger motor
    ros::ServiceClient motor_switch_client_;

    //set up service request
    hector_uav_msgs::EnableMotors motor_switch_srv_;

    //ros nodehandle
    ros::NodeHandle nh_;

    //ros rate
    double rate_;

    //verbosity
    bool verbose_;

    //solo flight or chasing turtle
    bool co_op_;

public:
    
    //constructor
    DroneCommander(ros::NodeHandle &nh);

    //callbacks
    void callbackHPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void callbackHVel(const geometry_msgs::Twist::ConstPtr &msg);
    void callbackTPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void callbackTSpline(const tmsgs::TurtleSplineConstPtr &spline_msg);
    
    std::pair<bot_utils::Pos2D,int> predict_turtle_pos();
    
    void run();
    
    void writeTrajMsg();
    void writeTargetMsg();
    void writeVelocityMsg(std::array<double,4> &vels);
    
    bool armMotor();
    bool disableMotor();
    void sigintHandler();

    //load params
    bool loadCommanderParams();
    bool loadTrajParams();
    bool loadControllerParams();
    
};

#endif //HBOT__DRONE_COMMANDER_H