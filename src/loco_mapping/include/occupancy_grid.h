#ifndef TBOT__OCCUPANCY_GRID_H
#define TBOT__OCCUPANCY_GRID_H
#include "bot_utils/bot_utils.h"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <array>
#include <vector>
#include <math.h>
class OccupancyGrid
{
private:
    int log_odds_thresh_; //threshold for occupied vs not occupied
    int log_odds_cap_; //the max value it can take
    double max_scan_range_; //maximum length of scan range
    double inflation_radius_; //inflating cell radius

    //map meta data
    bot_utils::Pos2D origin_; //origin of grid 0,0 in global frame
    bot_utils::Pos2D pos_min_; //minimum position the map can take
    bot_utils::Pos2D pos_max_; //maximum position the map can take
    bot_utils::Index map_size_; //number of grids in length x number of grids in width
    double cell_size_; //cell_size_ x cell_size_
    int total_cells_; //the total number of cells (map_size.i * map_size.j)

    //occupancy logs odd probability handlers
    std::vector<int> grid_log_odds_;
    std::vector<int> grid_inflation_;
    std::vector<bot_utils::Index> inflation_mask_;
    
    //conversion from degrees to rad
    std::array<double,360> DEG2RAD_;

    //laser range data
    std::vector<float> ranges;

    //robot pose
    geometry_msgs::PoseStamped robot_pose_; //the raw pose message we get from motion filter
    bot_utils::Pos2D robot_position_; //position of the robot in x and y coordinates
    double robot_heading_; //the heading of the robot limited to [pi,pi)

    //map details to publish
    nav_msgs::OccupancyGrid map_logsodd_;
    nav_msgs::OccupancyGrid map_inflation_;

    //subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber scan_sub_;
    
    //publishers
    ros::Publisher logsodd_pub_;
    ros::Publisher inflation_pub_;

    //ros rate
    double rate_;

    //node handler
    ros::NodeHandle nh_;

    //verbosity
    bool verbose_;
public:
    /*
    @brief: Class constructor. Takes in nh, loads parameters, prepares variables
    @param: ros::NodeHandle &nh
    @return: void
    */
    OccupancyGrid(ros::NodeHandle &nh);

    /*
    @brief: Main ros loop is here
    @param: void
    @return: void
    */
    void run();
    
    //inflation mask centred at 0,0. See brief
    /*
    @brief
    Our inflation radius is centred at cell 0,0. This implementation supports
    its use as a mask, that can be applied on top of an occupancy grid to get all other
    grids that are some distance away from the grid we want to inflate.

    To be precise, the cell at the edge would be on the circumference of the inflation circle (o)
    However, this would require us to use a floating point M_PI, which leads to rounding errors
    and can be a pain to deal with.
                                                                        ___
    So, we approximate the max search space as a square bounded circle |   | with sides 2*inflation radius
                                                                       |___|
    This would make the furthest away index at a distance squared of r^2/cellsize^2, which is represented by variable
    furthest_away. Again, we avoid usage of square roots to not deal with floating point values
    */  
    void generateMask();

    /*
    @brief: Updates logodds from each scan. Calls updateInflation
    @param: bool occupy , bot_utils::Index &idx
    @return: void
    */
    void updateLogOdds(bool occupy , bot_utils::Index &idx);

    /*
    @brief: Inflates region given by inflation_radius_ around the idx if it is occupied. Updates grid_inflation_
    @param: bool inflate , bot_utils::Index &idx
    @return: void
    */
    void updateInflation(bool inflate , bot_utils::Index &idx);

    /*
    @brief: Loads parameters from the parameter server
    @param: void
    @return: void
    */
    bool load_params();

    /*
    @brief: Gets pose message from turtlebot and populates robot_heading_, robot_pose_ and robot_position_
    @param: const geometry_msgs::PoseStampedConstPtr &pose_msg
    @return: void
    */
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg);

    /*
    @brief: Gets pose message from turtlebot and populates ranges_
    @param: const sensor_msgs::LaserScan::ConstPtr &msg
    @return: void
    */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    //grid helpers

    /*
    @brief: Check if a given index is out of bounds
    @param: bot_utils::Index&
    @return: bool
    */
    bool oob(bot_utils::Index &idx);

    /*
    @brief: Flattens a 2D grid coordinate into a single integer to ensure space complexity O(m)
    @param: bot_utils::Index&
    @return: int key
    */
    int flatten(bot_utils::Index &idx);
    
    /*
    @brief: Converts real number world position space to integer grid space coordinates
    @param: bot_utils::Pos2D&
    @return: bot_utils::Pos2D&
    */
    bot_utils::Index pos2idx(bot_utils::Pos2D &pos);

    /*
    @brief: Converts  integer grid space to real number world position space 
    @param: bot_utils::Pos2D&
    @return: bot_utils::Pos2D&
    */
    bot_utils::Pos2D idx2pos(bot_utils::Index &idx);
};




#endif //TBOT__OCCUPANCY_GRID_H