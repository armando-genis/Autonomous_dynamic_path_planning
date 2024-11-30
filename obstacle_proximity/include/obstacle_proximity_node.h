#ifndef OBSTACLE_PROXIMITY_NODE_H
#define OBSTACLE_PROXIMITY_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include "geometry_msgs/msg/point32.hpp"
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

class obstacle_proximity_node : public rclcpp::Node
{
private:
    /* data */
    // colors for the terminal
    std::string green = "\033[1;32m";
    std::string red = "\033[1;31m";
    std::string blue = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string purple = "\033[1;35m";
    std::string reset = "\033[0m";

    fop::SATCollisionChecker collision_checker; // Collision checker object

    geometry_msgs::msg::Polygon vehicle_polygon; // Vehicle polygon

    bool collision_detected = false; // Collision detected flag

    std::vector<bool> collision_vector;

    // function to create a vehicle polygon
    void create_vehicle_polygon();

    // Callback function
    void obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg);

    rclcpp::Subscription<obstacles_information_msgs::msg::ObstacleCollection>::SharedPtr obstacle_info_subscription_;

public:
    obstacle_proximity_node(/* args */);
    ~obstacle_proximity_node();
};

#endif // OBSTACLE_PROXIMITY_NODE_H
