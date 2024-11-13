#ifndef LOCAL_PATH_PLANNING_NODE_H
#define LOCAL_PATH_PLANNING_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>

// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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

// Eigen
#include <Eigen/Dense>

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
    bool collision_detected = false;

    std::shared_ptr<std::vector<bool>> collision_vector;
    std::shared_ptr<geometry_msgs::msg::Polygon> vehicle_path;
    std::shared_ptr<geometry_msgs::msg::Polygon> segment_path;
    std::shared_ptr<vector<Eigen::VectorXd>> waypoints; // [x, y, yaw]
    std::shared_ptr<State> car_state_;
    std::shared_ptr<traffic_information_msgs::msg::RoadElementsCollection> road_elements_;

    // tf2 buffer & listener
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;

    // function to get the state (position) of the car
    void getCurrentRobotState();

    // Callback function
    void obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg);
    void yawCarCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void roadElementsCallback(const traffic_information_msgs::msg::RoadElementsCollection::SharedPtr msg);

    // functions to calculate the trajectory of the car
    vector<pair<double, double>> calculate_trajectory(double steering_angle, double wheelbase, int num_points);
    void extract_segment(const std::vector<std::pair<double, double>> &path, std::vector<double> &segment_x, std::vector<double> &segment_y, double length);

    // function to calculate the next element of the path to get a segment of the trajectory
    double calculateDistance(double x1, double y1, double x2, double y2);

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

public:
    local_path_planning_node(/* args */);
    ~local_path_planning_node();
};

#endif // LOCAL_PATH_PLANNING_NODE_H