#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>


std::string tf_prefix;

void broadcast_turtle(const geometry_msgs::PoseStamped& msg)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = tf_prefix + "/base_link";
    transformStamped.transform.translation.x = msg.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.position.z;
    
    auto &q = msg.pose.orientation;
    transformStamped.transform.rotation.x = q.x;
    transformStamped.transform.rotation.y = q.y;
    transformStamped.transform.rotation.z = q.z;
    transformStamped.transform.rotation.w = q.w;
    
    br.sendTransform(transformStamped);
}
void cbPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    broadcast_turtle(*msg);
}
nav_msgs::Odometry msg_odom;
void cbOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    msg_odom = *msg;
}

int main(int argc, char **argv)
{   
    // the purpose of this node is to ad-hoc transform the base_link frame to the estimated position in rviz, so that the turtle moves around in rviz.
    // side steps odom and base_footprint transformations. This is possible because base_link transformation is static and only published once at the start, so parent of base link is permanently replaced with this.
    // possible also bcos no tf between odom and world
    // odom --> base_footprint --> base_link --> base_scan; base_link and base_footprint have same origin and orientation. base_scan is relative placed according to base_link

    ros::init(argc, argv, "turtle_tf2");
    ros::NodeHandle nh;
    
    geometry_msgs::PoseStamped msg;
    if (!nh.param<double>("initial_x", msg.pose.position.x, 0.0))
        ROS_WARN(" TMOVE : Param initial_x not found, set to 0");
    if (!nh.param<double>("initial_y", msg.pose.position.y, 0.0))
        ROS_WARN(" TMOVE : Param initial_y not found, set to 0");
    if (!nh.param<std::string>("tf_prefix", tf_prefix, "turtle"))
        ROS_WARN(" TMOVE : Param tf_prefix not found, set to 'turtle'");

    // send a preliminary message (may not be latched)
    msg.pose.orientation.w = 1;
    broadcast_turtle(msg);

    ros::Subscriber sub_odom = nh.subscribe("odom", 1, &cbOdom);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // wait for /turtle/odom to broadcast
    while (msg_odom.header.seq == 0)
    {
        ros::spinOnce();
    }
    sub_odom.shutdown();
    broadcast_turtle(msg); // try broadcasting again

    ros::spin();
    return 0;
}
