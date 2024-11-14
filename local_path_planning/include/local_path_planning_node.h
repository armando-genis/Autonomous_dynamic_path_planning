#ifndef LOCAL_PATH_PLANNING_NODE_H
#define LOCAL_PATH_PLANNING_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Eigen (must be included before OpenCV!!)
#include <Eigen/Dense>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

// Custom msgs obstacles_information_msgs for Obstacle and ObstacleCollection
#include "obstacles_information_msgs/msg/obstacle.hpp"
#include "obstacles_information_msgs/msg/obstacle_collection.hpp"

// Custom msgs traffic_information_msgs for RoadElements and RoadElementsCollection
#include "traffic_information_msgs/msg/road_elements.hpp"
#include "traffic_information_msgs/msg/road_elements_collection.hpp"

// STA collision checker
#include "sat_collision_checker.h"
// state file
#include "State.h"

// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <utility>

using namespace std;

class local_path_planning_node : public rclcpp::Node
{
private:
    // colors for the terminal
    std::string green = "\033[1;32m";
    std::string red = "\033[1;31m";
    std::string blue = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string purple = "\033[1;35m";
    std::string reset = "\033[0m";

    fop::SATCollisionChecker collision_checker; // Collision checker

    double turning_radius = 3.0;
    double num_points = 15;
    double track_car = 1;
    double start_offset = 0.5; // 0.5 meters offset in front of the car
    bool collision_detected_trajectory = false;
    bool collision_detected_path = false;
    int segment_start_index = 0;
    int segment_start_index_temp = 0;
    int closest_waypoint = 0;

    std::shared_ptr<std::vector<bool>> collision_vector;
    std::shared_ptr<geometry_msgs::msg::Polygon> vehicle_path;
    std::shared_ptr<geometry_msgs::msg::Polygon> segment_path;
    std::shared_ptr<vector<Eigen::VectorXd>> waypoints;              // [x, y, yaw]
    std::shared_ptr<vector<Eigen::VectorXd>> waypoints_segmentation; // [x, y, yaw]
    std::shared_ptr<vector<Eigen::VectorXd>> waypoints_historical;   // [x, y, yaw]
    std::shared_ptr<State> car_state_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> global_map_;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> rescaled_chunk_;
    std::vector<int> skip_ids = {}; // {363, 391};

    // tf2 buffer & listener
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;

    // function to get the state (position) of the car
    void getCurrentRobotState();
    // function to get the closest waypoint to the car
    double getDistanceFromOdom(Eigen::VectorXd wapointPoint);
    void compute_closest_waypoint();

    // function to compute the polygon of the path
    void compute_path_polygon();

    // Callback function
    void obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg);
    void yawCarCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void roadElementsCallback(const traffic_information_msgs::msg::RoadElementsCollection::SharedPtr msg);
    void globalMap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

    // functions to calculate the trajectory of the car
    vector<pair<double, double>> calculate_trajectory(double steering_angle, double wheelbase, int num_points);
    void extract_segment(const std::vector<std::pair<double, double>> &path, std::vector<double> &segment_x, std::vector<double> &segment_y, double length);

    // function to check if the crosswalk is visited
    void crosswalk_visited_check(const traffic_information_msgs::msg::RoadElements &crosswalk);

    // functions to combine and rescale the map
    void map_combination(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg);
    cv::Mat toMat(const nav_msgs::msg::OccupancyGrid &map);
    cv::Mat rescaleChunk(const cv::Mat &chunk_mat, double scale_factor);

    // Subscribers for the obstacle information
    rclcpp::Subscription<obstacles_information_msgs::msg::ObstacleCollection>::SharedPtr obstacle_info_subscription_;
    // subscriber for the yaw angle of the car & publisher for the lane steering
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_car_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lane_steering_publisher_;
    // subscriber for waypoints
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_subscription_;
    // subscriber for road elements
    rclcpp::Subscription<traffic_information_msgs::msg::RoadElementsCollection>::SharedPtr road_elements_subscription_;
    // publisher for the crosswalk markers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr crosswalk_marker_publisher_;
    // subcriber for the map complete of the hd map (maybe make this in anoter pkg more futher)
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_grid_map_sub_;
    // publisher for the occupancy grid
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_test_;

public:
    local_path_planning_node(/* args */);
    ~local_path_planning_node();
};

#endif // LOCAL_PATH_PLANNING_NODE_H