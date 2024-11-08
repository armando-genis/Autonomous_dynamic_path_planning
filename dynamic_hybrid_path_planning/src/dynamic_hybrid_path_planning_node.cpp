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

    occupancy_grid_pub_test_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_grid_obstacles", 10);

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
        pose_tf = tf2_buffer.lookupTransform("map", "velodyne", tf2::TimePointZero).transform;
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

cv::Mat dynamic_hybrid_path_planning_node::toMat(const nav_msgs::msg::OccupancyGrid &map)
{
    cv::Mat im(map.info.height, map.info.width, CV_8UC1);
    for (size_t i = 0; i < map.data.size(); i++)
    {
        if (map.data[i] == 0)
            im.data[i] = 254; // Free space
        else if (map.data[i] == 100)
            im.data[i] = 0; // Occupied space
        else
            im.data[i] = 205; // Unknown space
    }
    return im;
}

cv::Mat dynamic_hybrid_path_planning_node::rescaleChunk(const cv::Mat &chunk_mat, double scale_factor)
{
    cv::Mat rescaled_chunk;
    cv::resize(chunk_mat, rescaled_chunk, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);
    return rescaled_chunk;
}

nav_msgs::msg::OccupancyGrid dynamic_hybrid_path_planning_node::matToOccupancyGrid(const cv::Mat &mat, const nav_msgs::msg::OccupancyGrid &reference_map)
{
    nav_msgs::msg::OccupancyGrid grid = reference_map;
    grid.data.resize(mat.rows * mat.cols);

    for (int i = 0; i < mat.rows * mat.cols; i++)
    {
        if (mat.data[i] == 254)
            grid.data[i] = 0; // Free space
        else if (mat.data[i] == 0)
            grid.data[i] = 100; // Occupied space
        else
            grid.data[i] = -1; // Unknown space
    }
    return grid;
}

// callback function for global grid map
void dynamic_hybrid_path_planning_node::global_gridMapdata(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{

    auto init_time = std::chrono::system_clock::now();

    // Call the function to get the current robot state
    getCurrentRobotState();

    // Define chunk size (e.g., 20x20 meters)
    int chunk_size = 20; // 20 meters by 20 meters
    int chunk_radius = chunk_size / 2;

    // Convert car state to grid coordinates
    int car_x_grid = static_cast<int>((car_state_.x - map->info.origin.position.x) / map->info.resolution);
    int car_y_grid = static_cast<int>((car_state_.y - map->info.origin.position.y) / map->info.resolution);

    // Define chunk boundaries
    int min_x = std::max(0, car_x_grid - chunk_radius);
    int max_x = std::min(static_cast<int>(map->info.width), car_x_grid + chunk_radius);
    int min_y = std::max(0, car_y_grid - chunk_radius);
    int max_y = std::min(static_cast<int>(map->info.height), car_y_grid + chunk_radius);

    // Initialize the chunk grid
    nav_msgs::msg::OccupancyGrid chunk;
    chunk.header = map->header;
    chunk.info.resolution = map->info.resolution;
    chunk.info.width = max_x - min_x;
    chunk.info.height = max_y - min_y;
    chunk.info.origin.position.x = map->info.origin.position.x + min_x * map->info.resolution;
    chunk.info.origin.position.y = map->info.origin.position.y + min_y * map->info.resolution;
    chunk.info.origin.orientation.w = 1.0;

    chunk.data.resize(chunk.info.width * chunk.info.height, -1); // Initialize with unknown values

    // Copy data from the global map to the chunk
    for (int y = min_y; y < max_y; ++y)
    {
        for (int x = min_x; x < max_x; ++x)
        {
            int global_index = y * map->info.width + x;
            int local_x = x - min_x;
            int local_y = y - min_y;
            int chunk_index = local_y * chunk.info.width + local_x;

            chunk.data[chunk_index] = map->data[global_index];
        }
    }

    double scale_factor = 10;
    cv::Mat chunk_mat = toMat(chunk);
    cv::Mat rescaled_chunk_mat = rescaleChunk(chunk_mat, scale_factor);

    // Convert the rescaled Mat back to an OccupancyGrid
    rescaled_chunk = matToOccupancyGrid(rescaled_chunk_mat, chunk);

    // Update the resolution and adjust the width and height of the rescaled chunk
    rescaled_chunk.header = map->header;
    rescaled_chunk.info.resolution = 0.1;
    rescaled_chunk.info.width = rescaled_chunk_mat.cols;
    rescaled_chunk.info.height = rescaled_chunk_mat.rows;

    // Publish or use the chunk for planning
    // occupancy_grid_pub_test_->publish(rescaled_chunk);

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    cout << purple << "--> Map creation: " << duration << " ms" << reset << endl;
}

void dynamic_hybrid_path_planning_node::obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg)
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "base_footprint";
    grid.header.stamp = this->now();
    grid.info.resolution = 0.1;         // in meters
    grid.info.width = 200;              // grid width
    grid.info.height = 200;             // grid height
    grid.info.origin.position.x = -6.0; // Center the origin of the grid
    grid.info.origin.position.y = -6.0; // Center the origin of the grid
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

        for (size_t j = 0; j < obstacle.polygon.points.size(); ++j)
        {
            auto &current_point = obstacle.polygon.points[j];
            auto &next_point = obstacle.polygon.points[(j + 1) % obstacle.polygon.points.size()];

            int x0 = static_cast<int>((current_point.x - grid.info.origin.position.x) / grid.info.resolution);
            int y0 = static_cast<int>((current_point.y - grid.info.origin.position.y) / grid.info.resolution);
            int x1 = static_cast<int>((next_point.x - grid.info.origin.position.x) / grid.info.resolution);
            int y1 = static_cast<int>((next_point.y - grid.info.origin.position.y) / grid.info.resolution);

            draw_inflated_line(x0, y0, x1, y1, inflation_radius, value_to_mark);
        }
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