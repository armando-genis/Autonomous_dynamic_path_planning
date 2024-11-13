#ifndef DYNAMIC_HYBRID_PATH_PLANNING_NODE_H
#define DYNAMIC_HYBRID_PATH_PLANNING_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Custom msgs obstacles_information_msgs for Obstacle and ObstacleCollection
#include "obstacles_information_msgs/msg/obstacle.hpp"
#include "obstacles_information_msgs/msg/obstacle_collection.hpp"

#include <vector>
#include <cmath>
#include <iostream>
#include <mutex>

#include "State.h"
#include "Grid_map.h"
#include "CarData.h"
#include "HybridAstar.h"
#include "Node.h"

using namespace std;

class dynamic_hybrid_path_planning_node : public rclcpp::Node
{
private:
    /* data */

    double maxSteerAngle;
    double wheelBase;
    double axleToFront;
    double axleToBack;
    double width;
    double pathLength;
    double step_car;
    std::string grid_map_topic;

    // Car Data
    CarData car_data_;
    State car_state_;
    State car_state_2;

    // Grid Map
    std::shared_ptr<Grid_map>
        grid_map_;

    // colors for the terminal
    string green = "\033[1;32m";
    string red = "\033[1;31m";
    string blue = "\033[1;34m";
    string yellow = "\033[1;33m";
    string purple = "\033[1;35m";
    string reset = "\033[0m";

    // for path planning
    State waypoint_target_;

    // tf2 buffer & listener
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;

    // store start and goal point
    std::vector<State> start_goal_points_;

    // publishers & subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_grid_map_sub_;
    rclcpp::Subscription<obstacles_information_msgs::msg::ObstacleCollection>::SharedPtr obstacle_info_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_test_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr target_waypoint_subscriber_;

    // debug publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arrow_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hybrid_astar_path_pub_;

    // callback functions
    void global_gridMapdata(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg);
    void timer_callback();
    void markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
    void hybridAstarPathPlanning();
    void Star_End_point_visualization();

    // obstacle information data
    std::shared_ptr<obstacles_information_msgs::msg::ObstacleCollection> latest_obstacles_;

    // map data
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> global_map_;

    // rescaled_chunk; mutex
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> rescaled_chunk_;
    std::mutex rescaled_chunk_mutex_;

    // functions
    void getCurrentRobotState();
    // functions for map combination
    void mapCombination();
    cv::Mat toMat(const nav_msgs::msg::OccupancyGrid &map);
    cv::Mat rescaleChunk(const cv::Mat &chunk_mat, double scale_factor);

public:
    dynamic_hybrid_path_planning_node(/* args */);
    ~dynamic_hybrid_path_planning_node();
};

#endif // DYNAMIC_HYBRID_PATH_PLANNING_NODE_H