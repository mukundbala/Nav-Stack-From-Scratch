#include <ros/ros.h>
#include "grid.hpp"
#include "planner.hpp"
#include "trajectory.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <errno.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

std::vector<float> ranges;
void cbScan(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    ranges = msg->ranges; // creates a copy
}
Position pos_rbt(0, 0);
double ang_rbt = 10; // set to 10, because ang_rbt is between -pi and pi, and integer for correct comparison while waiting for motion to load
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    pos_rbt.x = p.x;
    pos_rbt.y = p.y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    ang_rbt = atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_main");
    ros::NodeHandle nh;

    // Make sure motion and move can run (fail safe)
    nh.setParam("run", true); // turns off other nodes

    // Get ROS parameters
    bool verbose;
    if (!nh.param("verbose_main", verbose, true))
        ROS_WARN(" TMAIN : Param verbose_main not found, set to true");
    if (!nh.param("initial_x", pos_rbt.x, 0.))
        ROS_WARN(" TMAIN : Param initial_x not found, set to 0");
    if (!nh.param("initial_y", pos_rbt.y, 0.))
        ROS_WARN(" TMAIN : Param initial_x not found, set to 0");
    std::vector<Position> goals;
    std::string goal_str;
    if (nh.param("goals", goal_str, std::to_string(pos_rbt.x) + "," + std::to_string(pos_rbt.y)))
    {
        char goal_str_tok[goal_str.length() + 1];
        strcpy(goal_str_tok, goal_str.c_str()); // to tokenise --> convert to c string (char*) first
        char *tok = strtok(goal_str_tok, " ,");
        try
        {
            while (tok != nullptr)
            {
                goals.emplace_back(
                    strtod(tok, nullptr),
                    strtod(strtok(nullptr, " ,"), nullptr));
                ROS_INFO("GOALs %f, %f", goals.back().x, goals.back().y);
                tok = strtok(nullptr, " ,");
            }
        }
        catch (...)
        {
            ROS_ERROR(" TMAIN : Invalid Goals: %s", goal_str.c_str());
            ros::shutdown();
            return 1;
        }
    }
    else
        ROS_WARN(" TMAIN : Param goal not found, set to %s", goal_str.c_str());
    Position pos_min;
    if (!nh.param("min_x", pos_min.x, -10.))
        ROS_WARN(" TMAIN : Param min_x not found, set to -10");
    if (!nh.param("min_y", pos_min.y, -10.))
        ROS_WARN(" TMAIN : Param min_y not found, set to -10");
    Position pos_max;
    if (!nh.param("max_x", pos_max.x, 10.))
        ROS_WARN(" TMAIN : Param max_x not found, set to 10");
    if (!nh.param("max_y", pos_max.y, 10.))
        ROS_WARN(" TMAIN : Param max_y not found, set to 10");
    double close_enough;
    if (!nh.param("close_enough", close_enough, 0.1))
        ROS_WARN(" TMAIN : Param close_enough not found, set to 0.1");
    double target_dt;
    if (!nh.param("target_dt", target_dt, 0.04))
        ROS_WARN(" TMAIN : Param target_dt not found, set to 0.04");
    double average_speed;
    if (!nh.param("average_speed", average_speed, 0.2))
        ROS_WARN(" TMAIN : Param average_speed not found, set to 0.2");
    double cell_size;
    if (!nh.param("cell_size", cell_size, 0.2))
        ROS_WARN(" TMAIN : Param cell_size not found, set to 0.05");
    double inflation_radius;
    if (!nh.param("inflation_radius", inflation_radius, 0.2))
        ROS_WARN(" TMAIN : Param inflation_radius not found, set to 0.3");
    int log_odds_thresh;
    if (!nh.param("log_odds_thresh", log_odds_thresh, 10))
        ROS_WARN(" TMAIN : Param log_odds_thresh not found, set to 10");
    int log_odds_cap;
    if (!nh.param("log_odds_cap", log_odds_cap, 20))
        ROS_WARN(" TMAIN : Param log_odds_cap not found, set to 20");
    double main_iter_rate;
    if (!nh.param("main_iter_rate", main_iter_rate, 25.0))
        ROS_WARN(" TMAIN : Param main_iter_rate not found, set to 25");
    // print out the parameters
    ROS_INFO(" TMAIN : Goals[%s], Grid[%.2f,%.2f to %.2f,%.2f]  CloseEnuf:%f  TgtDt:%f  AvgSpd:%f  CellSize:%f  InfMskRad:%f  LOThrsh:%d  LOCap:%d",
             goal_str.c_str(), pos_min.x, pos_min.y, pos_max.x, pos_max.y, close_enough, target_dt, average_speed, cell_size, inflation_radius, log_odds_thresh, log_odds_cap);

    // subscribers
    ros::Subscriber sub_scan = nh.subscribe("scan", 1, &cbScan);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // Publishers
    ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("path", 1, true);
    ros::Publisher pub_traj = nh.advertise<nav_msgs::Path>("trajectory", 1, true);
    ros::Publisher pub_target = nh.advertise<geometry_msgs::PointStamped>("target", 1, true);
    ros::Publisher pub_grid_lo = nh.advertise<nav_msgs::OccupancyGrid>("grid/log_odds", 1, true);
    ros::Publisher pub_grid_inf = nh.advertise<nav_msgs::OccupancyGrid>("grid/inflation", 1, true);

    // Prepare published messages
    nav_msgs::OccupancyGrid msg_grid_lo, msg_grid_inf; // preparation below
    geometry_msgs::PointStamped msg_target;
    msg_target.header.frame_id = "world"; // for rviz
    nav_msgs::Path msg_path;
    msg_path.header.frame_id = "world"; // for rviz
    nav_msgs::Path msg_traj;
    msg_traj.header.frame_id = "world"; // for rviz

    // Setup the occupancy grid class
    Grid grid(pos_min, pos_max, cell_size, inflation_radius, log_odds_thresh, log_odds_cap);

    // adjust msg_grid for rviz
    msg_grid_lo.data.resize(grid.size.i * grid.size.j);
    msg_grid_lo.header.frame_id = "world";
    msg_grid_lo.info.resolution = cell_size;
    msg_grid_lo.info.width = grid.size.j;
    msg_grid_lo.info.height = grid.size.i;
    msg_grid_lo.info.origin.position.x = -cell_size * 0.5 + pos_min.x;
    msg_grid_lo.info.origin.position.y = -cell_size * 0.5 + pos_min.y;
    // # flip the map bcos it is not displayed properly in rviz by rotating via quaternion
    msg_grid_lo.info.origin.orientation.x = 1 / M_SQRT2;
    msg_grid_lo.info.origin.orientation.y = 1 / M_SQRT2;
    msg_grid_inf = msg_grid_lo; // copy over

    // Setup the planner class
    Planner planner(grid);

    // setup loop rates
    ros::Rate rate(main_iter_rate);

    // Other variables
    bool replan = true;
    std::vector<Position> path, post_process_path, trajectory;
    int g = 0;                    // goal num
    Position pos_goal = goals[g]; // to trigger the reach goal
    int t = 0;                    // target num
    Position pos_target;

    // wait for other nodes to load
    ROS_INFO(" TMAIN : Waiting for topics");
    while (ros::ok() && (ranges.empty() || ang_rbt == 10))
    {
        rate.sleep();
        ros::spinOnce(); // update the topics
    }

    ROS_INFO(" TMAIN : ===== BEGIN =====");

    while (ros::ok())
    {
        // update all topics
        ros::spinOnce();

        // update the occ grid
        grid.update(pos_rbt, ang_rbt, ranges);

        // publish the map
        grid.write_to_msg(msg_grid_lo, msg_grid_inf);
        pub_grid_lo.publish(msg_grid_lo);
        pub_grid_inf.publish(msg_grid_inf);

        if (dist_euc(pos_rbt, pos_goal) < close_enough)
        { // reached the goal, get new goal
            replan = true;
            if (++g >= goals.size())
            {
                if (verbose)
                    ROS_INFO(" TMAIN : Last goal reached");
                break;
            }
            // there are goals remaining
            pos_goal = goals[g];
        }
        else if (!is_safe_trajectory(trajectory, grid))
        { // request a new path if path intersects inaccessible areas, or if there is no path
            replan = true;
        }

        // always try to publish the next target so it does not get stuck waiting for a new path.
        if (!trajectory.empty() && dist_euc(pos_rbt, pos_target) < close_enough)
        {
            if (--t < 0)
                t = 0; // in case the close enough for target triggers. indices cannot be less than 0.

            pos_target = trajectory[t];

            if (verbose)
                ROS_INFO(" TMAIN : Get next target (%f,%f)", pos_target.x, pos_target.y);

            // publish to target topic
            msg_target.point.x = pos_target.x;
            msg_target.point.y = pos_target.y;
            pub_target.publish(msg_target);
        }

        if (replan)
        {
            if (grid.get_cell(pos_rbt) && grid.get_cell(pos_goal))
            {
                if (verbose)
                    ROS_INFO(" TMAIN : Request Path from [%.2f, %.2f] to Goal %d at [%.2f,%.2f]",
                             pos_rbt.x, pos_rbt.y, g, pos_goal.x, pos_goal.y);
                // if the robot and goal are both on accessible cells of the grid
                path = planner.get(pos_rbt, pos_goal); // original path
                if (path.empty())
                { // path cannot be found
                    if (verbose)
                        ROS_WARN(" TMAIN : No path found between robot and goal");
                    // retry
                }
                else
                { // path found
                    // get turning points after processing path

                    // path.size() == 1 means start == goal. There is currently no code to handle this case, which may lead to problems downstream (trajectory generation and post process). Try and handle it.

                    if (verbose)
                        ROS_INFO(" TMAIN : Begin Post Process");
                    post_process_path = post_process(path, grid);

                    if (verbose)
                        ROS_INFO(" TMAIN : Begin trajectory generation over all turning points");
                    // generate trajectory over all turning points
                    // doing the following manner results in the front of trajectory being the goal, and the back being close to the rbt position
                    trajectory.clear();
                    for (int m = 1; m < post_process_path.size(); ++m)
                    {
                        Position &turn_pt_next = post_process_path[m - 1];
                        Position &turn_pt_cur = post_process_path[m];

                        std::vector<Position> traj = generate_trajectory(turn_pt_next, turn_pt_cur, average_speed, target_dt, grid);
                        for (Position &pos_tgt : traj)
                        {
                            trajectory.push_back(pos_tgt);
                        }
                    }

                    if (verbose)
                        ROS_INFO(" TMAIN : Trajectory generation complete");

                    // publish post processed path to path topic
                    msg_path.poses.clear();
                    for (Position &pos : post_process_path)
                    {
                        msg_path.poses.push_back(geometry_msgs::PoseStamped()); // insert a posestamped initialised to all 0
                        msg_path.poses.back().pose.position.x = pos.x;
                        msg_path.poses.back().pose.position.y = pos.y;
                    }
                    pub_path.publish(msg_path);

                    // publish trajectroy to trajectory topic
                    msg_traj.poses.clear();
                    for (Position &pos : trajectory)
                    {
                        msg_traj.poses.push_back(geometry_msgs::PoseStamped()); // insert a posestamped initialised to all 0
                        msg_traj.poses.back().pose.position.x = pos.x;
                        msg_traj.poses.back().pose.position.y = pos.y;
                    }
                    pub_traj.publish(msg_traj);

                    // get new target
                    t = trajectory.size() - 1; // last entry
                    // pick the more distant target so turtlebot does not stop intermitently around very close targets when new path is generated
                    if (t > 15)
                        t -= 15; // this is the average_speed * 15 * target_dt away
                    pos_target = trajectory[t];

                    // publish to target topic
                    msg_target.point.x = pos_target.x;
                    msg_target.point.y = pos_target.y;
                    pub_target.publish(msg_target);

                    replan = false;
                }
            }
            else
            { // robot lies on inaccessible cell, or if goal lies on inaccessible cell
                if (!grid.get_cell(pos_rbt))
                    ROS_WARN(" TMAIN : Robot lies on inaccessible area. No path can be found");
                if (!grid.get_cell(pos_goal))
                    ROS_WARN(" TMAIN : Goal lies on inaccessible area. No path can be found");
            }
        }

        // sleep for rest of iteration
        rate.sleep();
    }

    nh.setParam("run", false); // turns off other nodes
    ROS_INFO(" TMAIN : ===== END =====");
    return 0;
}
