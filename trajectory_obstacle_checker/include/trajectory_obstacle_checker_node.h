#ifndef TRAJECTORY_OBSTACLE_CHECKER_NODE_H
#define TRAJECTORY_OBSTACLE_CHECKER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/polygon.hpp>
// Custom msgs obstacles_information_msgs for Obstacle and ObstacleCollection
#include "obstacles_information_msgs/msg/obstacle.hpp"
#include "obstacles_information_msgs/msg/obstacle_collection.hpp"
#include "sat_collision_checker.h"

// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>

// Eigen
#include <Eigen/Dense>

using namespace std;

class trajectory_obstacle_checker_node : public rclcpp::Node
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

    bool collision_detected = false;
    std::vector<bool> collision_vector;

    geometry_msgs::msg::Polygon vehicle_path;

    // Callback function
    void obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg);
    void yawCarCallback(const std_msgs::msg::Float64::SharedPtr msg);

    // functions to calculate the trajectory of the car
    vector<pair<double, double>> calculate_trajectory(double steering_angle, double wheelbase, int num_points);
    void extract_segment(const std::vector<std::pair<double, double>> &path, std::vector<double> &segment_x, std::vector<double> &segment_y, double length);

    // subscriber & publisher
    rclcpp::Subscription<obstacles_information_msgs::msg::ObstacleCollection>::SharedPtr obstacle_info_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_car_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lane_steering_publisher_;

    // subscriber & publisher for debugging
    // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lane_publisher_;

public:
    trajectory_obstacle_checker_node(/* args */);
    ~trajectory_obstacle_checker_node();
};

#endif // TRAJECTORY_OBSTACLE_CHECKER_NODE_H