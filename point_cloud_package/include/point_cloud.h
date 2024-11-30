#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>


// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <utility>

class point_cloud: public rclcpp::Node
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

    double x_state = 0;
    double y_state = 0;
    double z_state = 0;

    double yaw_state = 0;

    // tf2 buffer & listener
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;

    // function to get the state (position) of the car
    void getCurrentRobotState();

    // callback
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // sub
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    // pub
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr to_map_pub_;


public:
    point_cloud(/* args */);
    ~point_cloud();
};


#endif