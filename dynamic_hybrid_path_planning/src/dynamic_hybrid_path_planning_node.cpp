#include "dynamic_hybrid_path_planning_node.h"

dynamic_hybrid_path_planning_node::dynamic_hybrid_path_planning_node(/* args */) : Node("dynamic_hybrid_path_planning_node"), tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer)
{

    this->declare_parameter<double>("maxSteerAngle", 0.0);
    this->declare_parameter<double>("wheelBase", 0.0);
    this->declare_parameter<double>("axleToFront", 0.0);
    this->declare_parameter<double>("axleToBack", 0.0);
    this->declare_parameter<double>("width", 0.0);
    this->declare_parameter<double>("pathLength", 0.0);
    this->declare_parameter<double>("step_car", 0.0);
    this->declare_parameter<std::string>("grid_map_topic", "/grid_map");

    this->get_parameter("maxSteerAngle", maxSteerAngle);
    this->get_parameter("wheelBase", wheelBase);
    this->get_parameter("axleToFront", axleToFront);
    this->get_parameter("axleToBack", axleToBack);
    this->get_parameter("width", width);
    this->get_parameter("pathLength", pathLength);
    this->get_parameter("step_car", step_car);
    this->get_parameter("grid_map_topic", grid_map_topic);

    // publishers & subscribers
    global_grid_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/occupancy_grid_complete_map", 10, std::bind(&dynamic_hybrid_path_planning_node::global_gridMapdata, this, std::placeholders::_1));

    obstacle_info_subscription_ = this->create_subscription<obstacles_information_msgs::msg::ObstacleCollection>(
        "/obstacle_info", 10, std::bind(&dynamic_hybrid_path_planning_node::obstacle_info_callback, this, std::placeholders::_1));

    // Create the vehicle geometry
    car_data_ = CarData(maxSteerAngle, wheelBase, axleToFront, axleToBack, width);
    car_data_.createVehicleGeometry();

    RCLCPP_INFO(this->get_logger(), "\033[1;34mmaxSteerAngle: %f\033[0m", maxSteerAngle);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mwheelBase: %f\033[0m", wheelBase);
    RCLCPP_INFO(this->get_logger(), "\033[1;34maxleToFront: %f\033[0m", axleToFront);
    RCLCPP_INFO(this->get_logger(), "\033[1;34maxleToBack: %f\033[0m", axleToBack);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mwidth: %f\033[0m", width);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mpathLength: %f\033[0m", pathLength);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mstep_car: %f\033[0m", step_car);
    RCLCPP_INFO(this->get_logger(), "\033[1;34mgrid_map_topic: %s\033[0m", grid_map_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> optimal_planner_node initialized.\033[0m");
}

dynamic_hybrid_path_planning_node::~dynamic_hybrid_path_planning_node()
{
}

void dynamic_hybrid_path_planning_node::getCurrentRobotState()
{
    geometry_msgs::msg::Transform pose_tf;
    try
    {
        pose_tf = tf2_buffer.lookupTransform("map", "velodyne", this->get_clock()->now(), std::chrono::milliseconds(50)).transform;
        car_state_.x = pose_tf.translation.x;
        car_state_.y = pose_tf.translation.y;
        tf2::Quaternion quat;
        tf2::fromMsg(pose_tf.rotation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        car_state_.heading = yaw;
    }

    catch (tf2::TransformException &ex)
    {
        std::cout << red << "Transform error: " << ex.what() << reset << std::endl;
    }
}

// callback function for global grid map
void dynamic_hybrid_path_planning_node::global_gridMapdata(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    std::cout << blue << "Received global grid map data" << reset << std::endl;
}

void dynamic_hybrid_path_planning_node::obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg)
{

    // geometry_msgs / Polygon polygon

    // for (const auto &obstacle : msg->obstacles)
    // {
    //     std::cout << "Obstacle ID: " << obstacle.id << std::endl;
    // }

    // cout numnbers of obstacles
    cout << "Number of obstacles: " << msg->obstacles.size() << endl;

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "base_footprint";
    grid.header.stamp = this->now();
    grid.info.resolution = 0.1;         // in meters
    grid.info.width = 120;              // grid width
    grid.info.height = 120;             // grid height
    grid.info.origin.position.x = -5.0; // Center the origin of the grid
    grid.info.origin.position.y = -5.0; // Center the origin of the grid
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    // Initialize grid data
    grid.data.resize(grid.info.width * grid.info.height, 0); // Initialize all cells as free

    auto mark_grid = [&](int x, int y, int value)
    {
        if (x >= 0 && x < static_cast<int>(grid.info.width) && y >= 0 && y < static_cast<int>(grid.info.height))
        {
            grid.data[y * grid.info.width + x] = value; // Mark the cell
        }
    };

    // Inflate cells around the given point
    auto inflate_point = [&](int x, int y, int radius, int value)
    {
        for (int dx = -radius; dx <= radius; ++dx)
        {
            for (int dy = -radius; dy <= radius; ++dy)
            {
                if (dx * dx + dy * dy <= radius * radius)
                { // Circle equation
                    mark_grid(x + dx, y + dy, value);
                }
            }
        }
    };

    // Simple line drawing between two points with inflation
    auto draw_inflated_line = [&](int x0, int y0, int x1, int y1, int radius, int value)
    {
        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        int n = 1 + dx + dy;
        int x_inc = (x1 > x0) ? 1 : -1;
        int y_inc = (y1 > y0) ? 1 : -1;
        int error = dx - dy;
        dx *= 2;
        dy *= 2;

        for (; n > 0; --n)
        {
            inflate_point(x0, y0, radius, value);

            if (error > 0)
            {
                x0 += x_inc;
                error -= dy;
            }
            else
            {
                y0 += y_inc;
                error += dx;
            }
        }
    };

    int inflation_radius = 3; // Inflated cells around the obstacles
    int value_to_mark = 100;
    for (size_t i = 0; i < msg->obstacles.size(); ++i)
    {
        const auto &obstacle = msg->obstacles[i];
        std::cout << green << "Obstacle size: " << obstacle.polygon.points.size() << reset << std::endl;
        std::cout << green << "-------------------" << reset << std::endl;
        // for (size_t j = 0; j < obstacle.polygon.points.size(); ++j)
        // {
        //     const auto &p0 = obstacle.polygon.points[j];
        //     const auto &p1 = obstacle.polygon.points[(j + 1) % obstacle.polygon.points.size()];
        //     draw_inflated_line((p0.x - grid.info.origin.position.x) / grid.info.resolution,
        //                        (p0.y - grid.info.origin.position.y) / grid.info.resolution,
        //                        (p1.x - grid.info.origin.position.x) / grid.info.resolution,
        //                        (p1.y - grid.info.origin.position.y) / grid.info.resolution,
        //                        inflation_radius, value_to_mark);
        // }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dynamic_hybrid_path_planning_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}