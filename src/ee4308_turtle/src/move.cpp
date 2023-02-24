#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <functional>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include "common.hpp"

double INF = 5000;
const double EPS = 1e-6; //for more precise double comparisons
bool target_changed = false;
Position target_position(INF,INF); //initialized as INF
Position robot_position(0, 0);
double robot_angle = 10; // set to 10, because ang_rbt is between -pi and pi, and integer for correct comparison while waiting for motion to load
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target_position.x = msg->point.x;
    target_position.y = msg->point.y;
    //ROS_INFO_STREAM("TMOVE: Target Coordinates Received: (" << target_position.x << "," << target_position.y<<")");
}

void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    robot_position.x = p.x;
    robot_position.y = p.y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    robot_angle = atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    // Get ROS Parameters
    bool enable_move;
    bool verbose;
    
    double Kp_lin;
    double Ki_lin;
    double Kd_lin;
    double max_lin_vel;
    double max_lin_acc;
    
    double Kp_ang;
    double Ki_ang;
    double Kd_ang;
    double max_ang_vel;
    double max_ang_acc;
    
    double move_iter_rate;

    bool tune_mode;
    bool tune_lin;
    bool tune_ang;

    std::string function_name = "pw";
    std::function<double(double)> damping_function;

    if (!nh.param("enable_move", enable_move, true))
    {
        ROS_WARN(" TMOVE : Param enable_move not found, set to true");
    }
        
    if (!nh.param("verbose_move", verbose, false))
    {
        ROS_WARN(" TMOVE : Param verbose_move not found, set to false");
    }

    if (!nh.param("Kp_lin", Kp_lin, 1.0))
    {
        ROS_WARN(" TMOVE : Param Kp_lin not found, set to 1.0");
    }
        
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
    {
        ROS_WARN(" TMOVE : Param Ki_lin not found, set to 0");
    }
        
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
    {
        ROS_WARN(" TMOVE : Param Kd_lin not found, set to 0");
    }
        
    if (!nh.param("max_lin_vel", max_lin_vel, 0.22))
    {
        ROS_WARN(" TMOVE : Param max_lin_vel not found, set to 0.22");
    }
        
    if (!nh.param("max_lin_acc", max_lin_acc, 1.0))
    {
        ROS_WARN(" TMOVE : Param max_lin_acc not found, set to 1");
    }
        
    if (!nh.param("Kp_ang", Kp_ang, 1.0))
    {
        ROS_WARN(" TMOVE : Param Kp_ang not found, set to 1.0");
    }
        
    if (!nh.param("Ki_ang", Ki_ang, 0.0))
    {
        ROS_WARN(" TMOVE : Param Ki_ang not found, set to 0");
    }
        
    if (!nh.param("Kd_ang", Kd_ang, 0.0))
    {
        ROS_WARN(" TMOVE : Param Kd_ang not found, set to 0");
    }
        
    if (!nh.param("max_ang_vel", max_ang_vel, 2.84))
    {
        ROS_WARN(" TMOVE : Param max_ang_vel not found, set to 2.84");
    }
        
    if (!nh.param("max_ang_acc", max_ang_acc, 4.0))
    {
        ROS_WARN(" TMOVE : Param max_ang_acc not found, set to 4");
    }
        
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
    {
        ROS_WARN(" TMOVE : Param move_iter_rate not found, set to 25");
    }

    if (!nh.param("tune_mode" , tune_mode , false))
    {
        ROS_WARN(" TMOVE: Param tune_mode not found, set to false");
    }

    if (!nh.param("tune_lin" , tune_lin , false))
    {
        ROS_WARN("TMOVE: Param tune_lin not found, set to false");
    }

    if (!nh.param("tune_ang" , tune_ang , false))
    {
        ROS_WARN(" TMOVE: Param tune_ang not found, set to false");
    }

    if (!nh.param<std::string>("func_type" , function_name , "pw"))
    {
        ROS_WARN("TMOVE: Function type name not found. Set to piecewise");
    }

    if (function_name == "pw")
    {
        damping_function = dampingPieceWise;
    }
    else if (function_name == "cos")
    {
        damping_function = dampingCos;
    }
    else if (function_name == "exp")
    {
        damping_function = dampingExp;
    }
    else if (function_name == "quadratic")
    {
        damping_function = dampingQuadratic;
    }

    // Subscribers
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // Publishers
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    // prepare published messages
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // Setup rate
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic

    if (!tune_mode) //just a safeguard in case we forget (see turtle.yaml)
    {
        tune_lin = false;
        tune_ang = false; 
    }
    else
    {
        if ((!tune_lin && !tune_ang) || (tune_ang && tune_lin))
        {
            ROS_INFO("Both tune_ang and tune_lin cannot have the same boolean value. Setting tune_mode to false!");
            tune_mode = false;
        }
    }
    // wait for other nodes to load
    ROS_INFO(" TMOVE : Waiting for topics");
    // we keep pinging the topic until we have received robot pos and target pos. 
    //We wait for robot angle and target_position to be published.
    while (ros::ok() && nh.param("run", true) && (robot_angle == 10 || (target_position.x == INF && target_position.y == INF))) 
    {
        rate.sleep();
        ros::spinOnce(); //update the topics
    }

    // Setup variables
    double cmd_lin_vel = 0;
    double cmd_ang_vel = 0;
    double dt = 0; //good practice to initalize variables and not leave them uninitialized
    double prev_time = ros::Time::now().toSec();
    double prev_linear_error = 0; //prevent derivative kick
    double prev_angular_error = 0; 
    prev_linear_error = dist_euc(robot_position , target_position);
    prev_angular_error = limit_angle(atan2(target_position.y - robot_position.y , target_position.x - robot_position.x) - robot_angle);
    if (fabs(prev_angular_error) > M_PI/2)
    {
        prev_linear_error *= -1; //going to reverse to the target
        prev_angular_error -= (sign(prev_angular_error) * M_PI);
    }
    
    double cumulative_linear_error = 0.0;
    double cumulative_angular_error = 0.0;

    if (tune_mode)
    {
        if (tune_lin && tune_ang)
        {
            ROS_WARN("[Tmove]: Params incorrectly set, turning off tune mode");
            tune_mode = false;
        }
        else if (tune_lin && tune_ang)
        {
            ROS_WARN("[Tmove]: Params incorrectly set, turning off tune mode");
            tune_mode = false;
        }
    }

    if (enable_move)
    {
        while (ros::ok() && nh.param("run", true))
        {
            // update all topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough.robot_angle == 10 || target_position.x == INF
            {
                continue;
            }
            prev_time += dt;

            //coupling --> velocity damping from coupling --> constrain linear vel --> constrain angular vel
            //linear errors
            double curr_linear_error = dist_euc(robot_position , target_position);
            cumulative_linear_error += (curr_linear_error * dt);

            //linear gains
            double p_cmd_lin = Kp_lin * curr_linear_error;
            double i_cmd_lin = Ki_lin * cumulative_linear_error;
            double d_cmd_lin = Kd_lin * (curr_linear_error - prev_linear_error) / dt;
            double raw_cmd_lin_vel = p_cmd_lin + i_cmd_lin + d_cmd_lin; //the raw signal based on euc error

            //angular errors
            double curr_angular_error = limit_angle(atan2(target_position.y - robot_position.y , target_position.x - robot_position.x) - robot_angle); // [-pi,pi)

            //we want to check if the errors are greater than |pi/2| aka 90 degrees on either side
            if (fabs(curr_angular_error) > M_PI/2)
            {
                raw_cmd_lin_vel *= -1; //going to reverse to the target
                curr_angular_error -= (sign(curr_angular_error) * M_PI); //now, curr_angular error is constrained between (-pi/2 , pi/2)
            } //at this point, we have the final curr_angular_error!

            ROS_WARN_COND(fabs(curr_angular_error) > M_PI/2 , "Current angular error has not be constrained properly!");
            cumulative_angular_error += (curr_angular_error * dt); //we can add in the cumulative angular error now
            
            double coupled_cmd_vel = raw_cmd_lin_vel * damping_function(curr_angular_error); //apply the coefficient to raw cmd_vel to couple it to ang error

            double p_cmd_ang = Kp_ang * curr_angular_error;
            double i_cmd_ang = Ki_ang * cumulative_angular_error;
            double d_cmd_ang = Kd_ang * (curr_angular_error - prev_angular_error) / dt ;
            double coupled_cmd_ang = p_cmd_ang + i_cmd_ang + d_cmd_ang;

            double curr_lin_acc = (coupled_cmd_vel - cmd_lin_vel) / dt;
            double sat_lin_acc = fabs(curr_lin_acc) > max_lin_acc ? sign(curr_lin_acc) * max_lin_acc : curr_lin_acc;
            double linvel_from_sat_lin_acc = cmd_lin_vel + (sat_lin_acc * dt);
            double sat_lin_vel = fabs(linvel_from_sat_lin_acc) > max_lin_vel ? sign(linvel_from_sat_lin_acc) * max_lin_vel : linvel_from_sat_lin_acc;

            double curr_ang_acc = (coupled_cmd_ang - cmd_ang_vel) / dt;
            double sat_ang_acc = fabs(curr_ang_acc) > max_ang_acc ? sign(curr_ang_acc) * max_ang_acc : curr_ang_acc;
            double angvel_from_sat_ang_acc = cmd_ang_vel + (sat_ang_acc * dt);
            double sat_ang_vel = fabs(angvel_from_sat_ang_acc) > max_ang_vel ? sign(angvel_from_sat_ang_acc) * max_ang_vel : angvel_from_sat_ang_acc;

            if (tune_mode)
            {
                if (tune_lin)
                {
                    sat_ang_vel = 0;
                }
                else if (tune_ang)
                {
                    sat_lin_vel = 0;
                }
            }
            
            cmd_lin_vel = sat_lin_vel;
            cmd_ang_vel = sat_ang_vel;
            prev_linear_error = curr_linear_error;
            prev_angular_error = curr_angular_error;

            // publish speeds
            msg_cmd.linear.x = cmd_lin_vel;
            msg_cmd.angular.z = cmd_ang_vel;
            pub_cmd.publish(msg_cmd);

            // verbose
            if (verbose)
            {
                //ROS_INFO(" TMOVE :  FV(%6.3f) AV(%6.3f)", cmd_lin_vel, cmd_ang_vel);
                //ROS_INFO("#####################################################################");
            }

            // wait for rate
            rate.sleep();
        }
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    ROS_INFO(" TMOVE : ===== END =====");
    return 0;
}