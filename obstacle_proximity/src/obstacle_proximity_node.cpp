#include "obstacle_proximity_node.h"

obstacle_proximity_node::obstacle_proximity_node(/* args */) : Node("trajectory_obstacle_checker_node")
{

    obstacle_info_subscription_ = this->create_subscription<obstacles_information_msgs::msg::ObstacleCollection>(
        "/obstacle_info", 10, std::bind(&obstacle_proximity_node::obstacle_info_callback, this, std::placeholders::_1));

    create_vehicle_polygon();

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> obstacle_proximity_node initialized.\033[0m");
}

obstacle_proximity_node::~obstacle_proximity_node()
{
}

void obstacle_proximity_node::obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received obstacle info");
    collision_vector.clear();

    collision_detected = false;
    for (const auto &obstacle : msg->obstacles)
    {
        bool current_collision = collision_checker.check_collision(vehicle_polygon, obstacle.polygon);
        collision_vector.push_back(current_collision);

        if (current_collision)
        {
            collision_detected = true;
        }
    }
}

void obstacle_proximity_node::create_vehicle_polygon()
{
    // Create a vehicle polygon
    geometry_msgs::msg::Point32 p1, p2, p3, p4;

    // Define points with float literals for compatibility with Point32
    p1.x = 0.0f;
    p1.y = 0.0f;
    p1.z = 0.0f;

    p2.x = 0.0f;
    p2.y = 1.0f;
    p2.z = 0.0f;

    p3.x = 2.0f;
    p3.y = 1.0f;
    p3.z = 0.0f;

    p4.x = 2.0f;
    p4.y = 0.0f;
    p4.z = 0.0f;

    // Push points to the polygon
    vehicle_polygon.points.push_back(p1);
    vehicle_polygon.points.push_back(p2);
    vehicle_polygon.points.push_back(p3);
    vehicle_polygon.points.push_back(p4);

    // Optionally close the polygon by adding the first point again
    vehicle_polygon.points.push_back(p1);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<obstacle_proximity_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}