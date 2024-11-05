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

#include <vector>
#include <cmath>
#include <iostream>

#include "State.h"
#include "Grid_map.h"
#include "CarData.h"
#include "HybridAstar.h"
#include "Node.h"

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

    // Grid Map
    std::shared_ptr<Grid_map> grid_map_;

public:
    dynamic_hybrid_path_planning_node(/* args */);
    ~dynamic_hybrid_path_planning_node();
};

#endif // DYNAMIC_HYBRID_PATH_PLANNING_NODE_H
