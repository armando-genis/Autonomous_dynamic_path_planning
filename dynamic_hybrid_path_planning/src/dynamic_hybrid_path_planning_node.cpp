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

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&dynamic_hybrid_path_planning_node::timer_callback, this));

    target_waypoint_subscriber_ = this->create_subscription<visualization_msgs::msg::Marker>("target_waypoint_marker", 10,
                                                                                             std::bind(&dynamic_hybrid_path_planning_node::markerCallback, this, std::placeholders::_1));

    arrow_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/start_and_end_arrows", 10);

    hybrid_astar_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hybrid_astar_path", 10);

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
        car_state_.z = pose_tf.translation.z - 1.55;
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

void dynamic_hybrid_path_planning_node::timer_callback()
{

    hybridAstarPathPlanning();
}

void dynamic_hybrid_path_planning_node::mapCombination()
{

    // Lock rescaled_chunk_ mutex to prevent concurrent access
    std::lock_guard<std::mutex> rescaled_lock(rescaled_chunk_mutex_);
    if (!rescaled_chunk_)
    {
        rescaled_chunk_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        std::cout << green << "Mutex Rescaled chunk created" << reset << std::endl;
    }

    // auto init_time = std::chrono::system_clock::now();

    int chunk_size = 20; // 20 x 20 meters
    int chunk_radius = chunk_size / 2;
    double forward_offset = 5.0; // Offset to displace the map forward

    // Convert car state to grid coordinates
    int car_x_grid = (car_state_.x - global_map_->info.origin.position.x) / global_map_->info.resolution;
    int car_y_grid = (car_state_.y - global_map_->info.origin.position.y) / global_map_->info.resolution;

    // Calculate forward offset in grid coordinates based on the car's heading
    int forward_x_offset = static_cast<int>((forward_offset * cos(car_state_.heading)) / global_map_->info.resolution);
    int forward_y_offset = static_cast<int>((forward_offset * sin(car_state_.heading)) / global_map_->info.resolution);

    // Shift the chunk's center forward by the offset
    int new_center_x = car_x_grid + forward_x_offset;
    int new_center_y = car_y_grid + forward_y_offset;

    // Define chunk boundaries based on the new center
    int min_x = std::max(0, new_center_x - chunk_radius);
    int max_x = std::min(static_cast<int>(global_map_->info.width), new_center_x + chunk_radius);
    int min_y = std::max(0, new_center_y - chunk_radius);
    int max_y = std::min(static_cast<int>(global_map_->info.height), new_center_y + chunk_radius);

    // Initialize the chunk grid
    nav_msgs::msg::OccupancyGrid chunk;
    chunk.header = global_map_->header;
    chunk.info.resolution = global_map_->info.resolution;
    chunk.info.width = max_x - min_x;
    chunk.info.height = max_y - min_y;
    chunk.info.origin.position.x = global_map_->info.origin.position.x + min_x * global_map_->info.resolution;
    chunk.info.origin.position.y = global_map_->info.origin.position.y + min_y * global_map_->info.resolution;
    // chunk.info.origin.position.z = car_state_.z;
    chunk.info.origin.orientation.w = 1.0;

    chunk.data.resize(chunk.info.width * chunk.info.height, 0);

    for (int y = min_y; y < max_y; ++y)
    {
        for (int x = min_x; x < max_x; ++x)
        {
            int global_index = y * global_map_->info.width + x;
            int local_x = x - min_x;
            int local_y = y - min_y;
            int chunk_index = local_y * chunk.info.width + local_x;

            chunk.data[chunk_index] = global_map_->data[global_index];
        }
    }

    double scale_factor = 5;
    cv::Mat chunk_mat = toMat(chunk);
    cv::Mat rescaled_chunk_mat = rescaleChunk(chunk_mat, scale_factor);

    rescaled_chunk_->header = global_map_->header;
    rescaled_chunk_->info.resolution = 0.2;
    rescaled_chunk_->info.width = rescaled_chunk_mat.cols;
    rescaled_chunk_->info.height = rescaled_chunk_mat.rows;
    rescaled_chunk_->info.origin.position.x = chunk.info.origin.position.x;
    rescaled_chunk_->info.origin.position.y = chunk.info.origin.position.y;
    rescaled_chunk_->info.origin.position.z = car_state_.z;
    rescaled_chunk_->info.origin.orientation.w = 1.0;

    rescaled_chunk_->data.resize(rescaled_chunk_->info.width * rescaled_chunk_->info.height, 0);

    for (int i = 0; i < rescaled_chunk_mat.rows * rescaled_chunk_mat.cols; i++)
    {
        if (rescaled_chunk_mat.data[i] == 254)
            rescaled_chunk_->data[i] = 0;
        else if (rescaled_chunk_mat.data[i] == 0)
            rescaled_chunk_->data[i] = 100;
        else
            rescaled_chunk_->data[i] = -1;
    }

    auto mark_grid = [&](int x, int y, int value)
    {
        if (x >= 0 && x < static_cast<int>(rescaled_chunk_->info.width) && y >= 0 && y < static_cast<int>(rescaled_chunk_->info.height))
        {
            rescaled_chunk_->data[y * rescaled_chunk_->info.width + x] = value; // Mark the cell
        }
    };

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

    int inflation_radius = 1; // Inflated cells around the obstacles
    int value_to_mark = 100;

    // Transformation from lidar frame to map frame
    double cos_heading = cos(car_state_.heading);
    double sin_heading = sin(car_state_.heading);

    for (size_t i = 0; i < latest_obstacles_->obstacles.size(); ++i)
    {
        const auto &obstacle = latest_obstacles_->obstacles[i];
        for (size_t j = 0; j < obstacle.polygon.points.size(); ++j)
        {
            auto &current_point_lidar = obstacle.polygon.points[j];
            auto &next_point_lidar = obstacle.polygon.points[(j + 1) % obstacle.polygon.points.size()];

            geometry_msgs::msg::Point current_point_map;
            current_point_map.x = car_state_.x + cos_heading * current_point_lidar.x - sin_heading * current_point_lidar.y;
            current_point_map.y = car_state_.y + sin_heading * current_point_lidar.x + cos_heading * current_point_lidar.y;

            geometry_msgs::msg::Point next_point_map;
            next_point_map.x = car_state_.x + cos_heading * next_point_lidar.x - sin_heading * next_point_lidar.y;
            next_point_map.y = car_state_.y + sin_heading * next_point_lidar.x + cos_heading * next_point_lidar.y;

            int x0 = static_cast<int>((current_point_map.x - rescaled_chunk_->info.origin.position.x) / rescaled_chunk_->info.resolution);
            int y0 = static_cast<int>((current_point_map.y - rescaled_chunk_->info.origin.position.y) / rescaled_chunk_->info.resolution);
            int x1 = static_cast<int>((next_point_map.x - rescaled_chunk_->info.origin.position.x) / rescaled_chunk_->info.resolution);
            int y1 = static_cast<int>((next_point_map.y - rescaled_chunk_->info.origin.position.y) / rescaled_chunk_->info.resolution);

            draw_inflated_line(x0, y0, x1, y1, inflation_radius, value_to_mark);
        }
    }

    // publish the chunk
    occupancy_grid_pub_test_->publish(*rescaled_chunk_);

    // auto end_time = std::chrono::system_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    // std::cout << blue << "Execution time holonomic: " << duration << " ms" << reset << std::endl;
}

void dynamic_hybrid_path_planning_node::hybridAstarPathPlanning()
{

    // Lock and safely access rescaled_chunk_
    nav_msgs::msg::OccupancyGrid::SharedPtr local_rescaled_chunk;
    {
        std::lock_guard<std::mutex> lock(rescaled_chunk_mutex_);
        if (rescaled_chunk_)
        {
            // std::cout << green << "Mutex Rescaled chunk accessed --------------------" << reset << std::endl;
            local_rescaled_chunk = rescaled_chunk_;
        }
        else
        {
            // If rescaled_chunk_ hasn't been created, log a message and return
            std::cout << red << "No rescaled chunk data available" << reset << std::endl;
            return;
        }
    }

    grid_map_ = std::make_shared<Grid_map>(*local_rescaled_chunk);
    grid_map_->setcarData(car_data_);

    if (grid_map_->isSingleStateCollisionFree(car_state_) && grid_map_->isSingleStateCollisionFree(waypoint_target_))
    {
        RCLCPP_ERROR(this->get_logger(), "\033[1;31m --> car or waypoint points are in collision or out of the map <-- \033[0m");
        return;
    }

    HybridAstar hybrid_astar(*grid_map_, car_data_, pathLength, step_car);
    auto goal_trajectory = hybrid_astar.run(car_state_, waypoint_target_);

    visualization_msgs::msg::MarkerArray arrow_marker_array;
    int id = 4000; // Unique ID for each marker
    size_t total_states = goal_trajectory.size();

    for (size_t i = 0; i < goal_trajectory.size(); ++i)
    {
        const auto &state = goal_trajectory[i];

        // Convert heading (yaw) to quaternion for marker orientation
        tf2::Quaternion quat;
        quat.setRPY(0, 0, state.heading);

        // Create arrow marker to represent the heading
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header.frame_id = "map";
        arrow_marker.header.stamp = this->now();
        arrow_marker.ns = "state_heading_arrows";
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.id = id++;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;

        // Set arrow size (you can adjust these values as needed)
        arrow_marker.scale.x = 0.8;  // Arrow length
        arrow_marker.scale.y = 0.15; // Arrow width
        arrow_marker.scale.z = 0.15; // Arrow height

        // Set arrow color (you can use a different color for arrows)
        arrow_marker.color.r = 1.0;
        arrow_marker.color.g = 1.0;
        arrow_marker.color.b = 1.0;
        arrow_marker.color.a = 0.6;

        // Position the arrow at the same (x, y) as the state, with a slightly higher z-value
        arrow_marker.pose.position.x = state.x;
        arrow_marker.pose.position.y = state.y;
        arrow_marker.pose.position.z = 0.1; // Slightly above the ground

        // Set the orientation of the arrow based on the heading
        arrow_marker.pose.orientation.x = quat.x();
        arrow_marker.pose.orientation.y = quat.y();
        arrow_marker.pose.orientation.z = quat.z();
        arrow_marker.pose.orientation.w = quat.w();

        arrow_marker_array.markers.push_back(arrow_marker);
    }

    // Publish the MarkerArray containing the arrows
    arrow_pub_->publish(arrow_marker_array);

    // Memory clean up
    goal_trajectory.clear();
    arrow_marker_array.markers.clear();
}

void dynamic_hybrid_path_planning_node::Star_End_point_visualization()
{

    visualization_msgs::msg::MarkerArray arrow_states;
    int id = 7000; // Unique ID for each marker
    for (const auto &state : start_goal_points_)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // Assuming map frame
        marker.header.stamp = this->now();
        marker.ns = "next_state_arrows";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::ARROW;

        marker.pose.position.x = state.x;
        marker.pose.position.y = state.y;
        marker.pose.position.z = 0.0;

        // Set the orientation based on the yaw angle (heading)
        tf2::Quaternion quat;
        quat.setRPY(0, 0, state.heading);
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker.pose.orientation.w = quat.w();

        marker.scale.x = 2.0; // Length of the arrow
        marker.scale.y = 0.3; // Width of the arrow shaft
        marker.scale.z = 0.3; // Height of the arrow shaft

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        arrow_states.markers.push_back(marker);
    }

    // Publish the MarkerArray containing the arrows
    arrow_pub_->publish(arrow_states);

    start_goal_points_.clear();
}

// ============================== Callbacks ==============================
void dynamic_hybrid_path_planning_node::obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg)
{
    latest_obstacles_ = msg;
    if (global_map_)
    {
        getCurrentRobotState();
        mapCombination();
    }
    else
    {
        std::cout << red << "Global map data is not yet available" << reset << std::endl;
    }
}

void dynamic_hybrid_path_planning_node::global_gridMapdata(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    global_map_ = map;
}

void dynamic_hybrid_path_planning_node::markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
    waypoint_target_.x = msg->pose.position.x;
    waypoint_target_.y = msg->pose.position.y;
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    waypoint_target_.heading = yaw;
}

// ============================== Main Function for map rescale ==============================
cv::Mat
dynamic_hybrid_path_planning_node::toMat(const nav_msgs::msg::OccupancyGrid &map)
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dynamic_hybrid_path_planning_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}