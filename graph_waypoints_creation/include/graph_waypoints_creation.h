#ifndef GRAPH_WAYPOINTS_CREATION_H
#define GRAPH_WAYPOINTS_CREATION_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Dense>

// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <utility>
#include <fstream>
#include <iomanip>
#include <filesystem>

#include "State.h"

using namespace std;

class graph_waypoints_creation : public rclcpp::Node
{
private:
    // colors for the terminal
    std::string green = "\033[1;32m";
    std::string red = "\033[1;31m";
    std::string blue = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string purple = "\033[1;35m";
    std::string reset = "\033[0m";

    // state of the car
    std::shared_ptr<State> car_state_;
    std::shared_ptr<State> previous_pose_;

    // tf2 buffer & listener
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;

    // function to get the state (position) of the car
    void getCurrentRobotState();

    // write the waypoints to a file
    void writeHeaderToFile();
    void writePoseToFile(const State &odom_pos_, bool change_flag);
    void pub_callback();

    std::ofstream ofs_;
    std::string file_path_;

    double interval_ = 2;
    bool first_data_received_ = false;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_saver_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_info_pub_;

public:
    graph_waypoints_creation(/* args */);
    ~graph_waypoints_creation();
};

#endif // GRAPH_WAYPOINTS_CREATION_Hs