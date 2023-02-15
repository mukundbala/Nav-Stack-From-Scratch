#include "occupancy_grid.h"

OccupancyGrid::OccupancyGrid(ros::NodeHandle& nh)
{
    nh_ = nh;
    bool param_status = load_params(); //load params first
    ROS_WARN_COND(!param_status , "[OccupancyGrid]: Parameters have not been loaded correctly!");

    origin_ = pos_min_; //Map's origin set at pos_min
    map_size_.i = std::round((pos_max_.x - pos_min_.x) / cell_size_);
    map_size_.j = std::round((pos_max_.y - pos_min_.y) / cell_size_);
    total_cells_ = map_size_.i * map_size_.j;

    grid_log_odds_.assign(total_cells_ , 0); //fill with zeros.
    grid_inflation_.assign(total_cells_, 0); //fill with zeros

    for (int i = 0; i < 360 ; ++i)
    {
        DEG2RAD_[i] = M_PI * i /180;
    }

    //prepare maps
    map_logodds_.data.resize(total_cells_);
    map_logodds_.header.frame_id = "world";
    map_logodds_.info.resolution = cell_size_;
    map_logodds_.info.width = map_size_.j;
    map_logodds_.info.height = map_size_.i;
    map_logodds_.info.origin.position.x = -cell_size_ * 0.5 + pos_min_.x; //we centre our map
    map_logodds_.info.origin.position.y = -cell_size_ * 0.5 + pos_min_.y; //we centre our map
    map_logodds_.info.origin.orientation.x = 1 / M_SQRT2;
    map_logodds_.info.origin.orientation.y = 1 / M_SQRT2;
    map_inflation_ = map_logodds_;

    //prepare msg
    mapinfo_inflation_.data.reserve(total_cells_);
    mapinfo_logodds_.data.reserve(total_cells_);
    //setup space for ranges
    ranges.reserve(360);

    //initalize values
    robot_heading_ = -10; //some unlikely value;
    robot_position_.setCoords(-50,-50); //some unlikely values

    //subscriber
    pose_sub_ = nh_.subscribe("pose" , 1 , &OccupancyGrid::poseCallback , this);
    scan_sub_ = nh_.subscribe("scan" ,1 , &OccupancyGrid::scanCallback , this);

    //advertise topics to publish 
    logsodd_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid/log_odds" , 1 , true);
    inflation_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid/inflation" , 1 , true);

    mapinfo_inflation_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("grid/mapinfo/inflation" , 1 , true);
    mapinfo_logodds_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("grid/mapinfo/logodds" , 1 , true);
    generateMask();

    ROS_INFO("[OccupancyGrid]: Occupancy map prepared!");
}

void OccupancyGrid::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    this->robot_pose_ = *msg;
    this->robot_position_.x = robot_pose_.pose.position.x;
    this->robot_position_.y = robot_pose_.pose.position.y;
    this->robot_heading_ = bot_utils::headingFromQuat(robot_pose_); //done
    ROS_WARN_COND(abs(robot_heading_) > M_PI , "Heading not constrained!");
}

void OccupancyGrid::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    this->ranges = msg->ranges;
}

void OccupancyGrid::generateMask()
{
    inflation_mask_.clear();
    double cells_in_radius = round(inflation_radius_ / cell_size_);
    double furthest_away = inflation_radius_ * inflation_radius_ / cell_size_ / cell_size_;
    for (double i = -cells_in_radius ; i<= cells_in_radius ; ++i)
    {
        for (double j = -cells_in_radius ; j<= cells_in_radius ; ++j)
        {
            double dist = i*i + j*j;
            if (dist <= furthest_away)
            {
                bot_utils::Index pt(i,j);
                inflation_mask_.emplace_back(pt);
            }
        }
    }
    ROS_INFO_STREAM("[OccupancyGrid]: Mask of size " << inflation_mask_.size()-1 << " created"); //we minus one because we exclude the origin grid
}

void OccupancyGrid::run()
{
    ros::Rate spinrate(rate_);

    ROS_INFO("[OccupancyGrid]: Waiting for topics");
    while(ros::ok && (ranges.empty() || robot_heading_ == -10) && nh_.param("trigger_nodes" , true)) //tick until we receive all the information && nh_.param("trigger_occ" , true)
    {
        spinrate.sleep();
        ros::spinOnce();
        ROS_INFO("[OccupancyGrid]: Waiting for topics");
    }
    ROS_INFO("[OccupancyGrid]: Starting Occupancy Grid!");

    while (ros::ok() && nh_.param("trigger_nodes" , true))// && nh_.param("trigger_occ" , true)
    {
        ros::spinOnce();
        bot_utils::Index robot_idx = pos2idx(robot_position_); //geting the index where the robot is at

        for (int i = 0 ; i < 360 ; ++i) //iterating through the scan angles..
        {
            double scan_angle_rad = DEG2RAD_[i] + robot_heading_;
            double range = ranges[i];
            bool obstacle_free = range > max_scan_range_;
            range = obstacle_free ? max_scan_range_ : range;

            bot_utils::Pos2D ray_edge_position( //inverse sensor model
                                    robot_position_.x + range * cos(scan_angle_rad),
                                    robot_position_.y + range * sin(scan_angle_rad));
            
            bot_utils::Index ray_edge_grid = pos2idx(ray_edge_position); //convert the position of the edge cell to index

            //generate an array consisting of all the grids the array touches
            std::vector<bot_utils::Index> ray_array = bot_utils::bresenham_los(robot_idx , ray_edge_grid);
            if (obstacle_free)
            {
                for (auto &idx : ray_array)
                {
                    updateLogOdds(false , idx);
                }
            }
            else
            {
                for (int m = 0 ; m < ray_array.size()-1 ; ++m)
                {
                    bot_utils::Index idx = ray_array[m];
                    updateLogOdds(false , idx);
                }
                updateLogOdds(true,ray_array.back());
            }

        }
        
        mapinfo_logodds_.data.clear();
        mapinfo_inflation_.data.clear();

        for (int k = 0 ; k<grid_log_odds_.size() ; ++k)
        {
            if (grid_inflation_.at(k) > 0)
            {   
                map_inflation_.data.at(k) = std::min(grid_inflation_.at(k) - 1 , 3);
            }
            else
            {
                 map_inflation_.data.at(k) = -1;
            }
            map_logodds_.data.at(k) = (((double) grid_log_odds_.at(k)) / log_odds_cap_ + 1) * 50;
            mapinfo_logodds_.data.push_back(grid_log_odds_.at(k));
            mapinfo_inflation_.data.push_back(grid_inflation_.at(k));
        }
        logsodd_pub_.publish(map_logodds_);
        inflation_pub_.publish(map_inflation_);
        mapinfo_inflation_pub_.publish(mapinfo_inflation_);
        mapinfo_logodds_pub_.publish(mapinfo_logodds_);
    }
}

void OccupancyGrid::updateLogOdds(bool occupy , bot_utils::Index &idx)
{
    //first, we make use the idx is in the map
    if (oob(idx))
    {
        return;
    }
    int increment = occupy ? 1 : -1;
    int flattened_id = flatten(idx);
    int prev_log_odds = grid_log_odds_.at(flattened_id);

    if (occupy && (prev_log_odds < log_odds_cap_))
    {
        grid_log_odds_.at(flattened_id) += 1;
    }

    else if (!occupy && (prev_log_odds > -log_odds_cap_))
    {
        grid_log_odds_.at(flattened_id) -= 1;
    }

    if (prev_log_odds < log_odds_thresh_ && grid_log_odds_.at(flattened_id) >= log_odds_thresh_)
    {
        updateInflation(true , idx);
    }

    else if (prev_log_odds >= log_odds_thresh_ && grid_log_odds_.at(flattened_id) < log_odds_thresh_)
    {
        updateInflation(false , idx);
    }
}

void OccupancyGrid::updateInflation(bool inflate , bot_utils::Index &idx)
{
    int inf = inflate ? 1 : -1;
    for (int k = 0 ; k < inflation_mask_.size() ; ++k)
    {
        int i_inflate = inflation_mask_.at(k).i + idx.i;
        int j_inflate = inflation_mask_.at(k).j + idx.j;
        bot_utils::Index to_inflate(i_inflate,j_inflate);
        if (oob(to_inflate))
        {
            continue;
        }
        int flattened_id = flatten(to_inflate);
        grid_inflation_.at(flattened_id) += inf;
    }
}

int OccupancyGrid::flatten(bot_utils::Index &idx)
{
    return idx.i * map_size_.j + idx.j;
}

bool OccupancyGrid::oob(bot_utils::Index &idx)
{
    return (idx.i <=0 || idx.i >=map_size_.i || idx.j < 0 && idx.j >= map_size_.j);
}


bot_utils::Index OccupancyGrid::pos2idx(bot_utils::Pos2D &pos)
{
    int i = round((pos.x - origin_.x) / cell_size_);
    int j = round((pos.y - origin_.y) / cell_size_);
    return bot_utils::Index(i,j);
}

bot_utils::Pos2D OccupancyGrid::idx2pos(bot_utils::Index &idx)
{
    double x = idx.i * cell_size_ + origin_.x;
    double y = idx.j * cell_size_ + origin_.y;
    return bot_utils::Pos2D(x,y);
}


//11 params
bool OccupancyGrid::load_params()
{
    bool status = true;
    if (!nh_.param("cell_size" , this->cell_size_ , 0.05))
    {
        ROS_WARN("[OccupancyGrid]: Param cell_size not found, defaulting to 0.05");
        status = false;
    }

    if (!nh_.param("max_scan_range" , this->max_scan_range_ , 3.499999))
    {
        ROS_WARN("[OccupancyGrid]: Param cell_size not found, defaulting to 3.499999");
        status = false;
    }

    if (!nh_.param("inflation_radius" , this->inflation_radius_ , 0.2))
    {
        ROS_WARN("[OccupancyGrid]: Param inflation_radius not found, defaulting to 0.2");
        status = false;
    }

    if (!nh_.param("log_odds_thresh" , this->log_odds_thresh_ , 10))
    {
        ROS_WARN("[OccupancyGrid]: Param log_odds_thresh not found, defaulting to 10");
        status = false;
    }

    if (!nh_.param("log_odds_cap" , this->log_odds_cap_ , 20))
    {
        ROS_WARN("[OccupancyGrid]: Param log_odds_cap not found, defaulting to 20");
        status = false;
    }

    if (!nh_.param("occ_rate" , this->rate_ , 25.0))
    {
        ROS_WARN("[OccupancyGrid]: Param occ_rate not found, defaulting to 25.0");
        status = false;
    }

    if (!nh_.param("min_x" , this->pos_min_.x , -10.0))
    {
        ROS_WARN("[OccupancyGrid]: Param min_x not found, defaulting to -10");
        status = false;
    }

    if (!nh_.param("min_y" , this->pos_min_.y , -10.0))
    {
        ROS_WARN("[OccupancyGrid]: Param min_y not found, defaulting to -10");
        status = false;
    }

    if (!nh_.param("max_x" , this->pos_max_.x , 10.0))
    {
        ROS_WARN("[OccupancyGrid]: Param max_x not found, defaulting to 10");
        status = false;
    }

    if (!nh_.param("max_y" , this->pos_max_.y , 10.0))
    {
        ROS_WARN("[OccupancyGrid]: Param max_y not found, defaulting to 10");
        status = false;
    }

    if (!nh_.param("occ_verbose" , this->verbose_ , false))
    {
        ROS_WARN("[OccupancyGrid]: Param occ_verbose not found, defaulting to false");
        status = false;
    }

    return status;
}