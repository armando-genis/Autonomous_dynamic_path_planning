#include "local_path_planning_node.h"

local_path_planning_node::local_path_planning_node(/* args */) : Node("local_path_planning_node"), tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer)
{

    obstacle_info_subscription_ = this->create_subscription<obstacles_information_msgs::msg::ObstacleCollection>(
        "/obstacle_info", 10, std::bind(&local_path_planning_node::obstacle_info_callback, this, std::placeholders::_1));

    road_elements_subscription_ = this->create_subscription<traffic_information_msgs::msg::RoadElementsCollection>(
        "/road_elements", 10, std::bind(&local_path_planning_node::roadElementsCallback, this, std::placeholders::_1));

    yaw_car_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/yaw_car", 10, std::bind(&local_path_planning_node::yawCarCallback, this, std::placeholders::_1));

    waypoints_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/waypoints_routing", 10, std::bind(&local_path_planning_node::waypoints_callback, this, std::placeholders::_1));

    global_grid_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid_complete_map", 10, std::bind(&local_path_planning_node::globalMap_callback, this, std::placeholders::_1));

    lane_steering_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "steering_lane", 10);

    crosswalk_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "crosswalk_markers", 10);

    occupancy_grid_pub_test_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid_obstacles", 10);

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path_related", 10);

    vehicle_path = std::make_shared<geometry_msgs::msg::Polygon>();
    segment_path = std::make_shared<geometry_msgs::msg::Polygon>();
    collision_vector = std::make_shared<std::vector<bool>>();
    waypoints = std::make_shared<vector<Eigen::VectorXd>>();
    car_state_ = std::make_shared<State>();
    waypoints_segmentation = std::make_shared<vector<Eigen::VectorXd>>();
    waypoints_historical = std::make_shared<vector<Eigen::VectorXd>>();
    global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    rescaled_chunk_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> local_path_planning_node initialized.\033[0m");
}

local_path_planning_node::~local_path_planning_node()
{
}

void local_path_planning_node::getCurrentRobotState()
{
    geometry_msgs::msg::Transform pose_tf;
    try
    {
        pose_tf = tf2_buffer.lookupTransform("map", "velodyne", tf2::TimePointZero).transform;
        car_state_->x = pose_tf.translation.x;
        car_state_->y = pose_tf.translation.y;
        car_state_->z = pose_tf.translation.z - 1.55;
        tf2::Quaternion quat;
        tf2::fromMsg(pose_tf.rotation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        car_state_->heading = yaw;

        compute_closest_waypoint();
    }

    catch (tf2::TransformException &ex)
    {
        std::cout << red << "Transform error: " << ex.what() << reset << std::endl;
    }
}

void local_path_planning_node::crosswalk_visited_check(const traffic_information_msgs::msg::RoadElements &crosswalk)
{
    double center_x = 0.0;
    double center_y = 0.0;
    int num_points = crosswalk.points.size();

    for (const auto &point : crosswalk.points)
    {
        center_x += point.x;
        center_y += point.y;
    }
    center_x /= num_points;
    center_y /= num_points;

    // Check for collision only if the crosswalk is near the the car (e.g., within 3 meters)
    // if (std::abs(crosswalk.points[0].x - car_state_->x) < 5 && std::abs(crosswalk.points[0].y - car_state_->y) < 5)
    if (std::abs(center_x - car_state_->x) < 5 && std::abs(center_y - car_state_->y) < 5)
    {
        // append the element id to the list of visited crosswalks
        skip_ids.push_back(crosswalk.id);
        segment_start_index_temp = segment_start_index;
    }
}

void local_path_planning_node::compute_closest_waypoint()
{

    if (waypoints->empty())
    {
        std::cout << red << "Warning: waypoints is not available. Skipping close waypoint computation." << reset << std::endl;
        return;
    }

    constexpr size_t first_wp = 0;
    const size_t last_wp = waypoints->size() - 1;
    constexpr int search_offset_back = 5;
    constexpr int search_offset_forward = 15;

    int search_start = std::max(static_cast<int>(closest_waypoint) - search_offset_back, static_cast<int>(first_wp));
    int search_end = std::min(static_cast<int>(closest_waypoint) + search_offset_forward, static_cast<int>(last_wp));

    double smallest_curr_distance = std::numeric_limits<double>::max();

    for (int i = search_start; i <= search_end; i++)
    {
        double curr_distance = getDistanceFromOdom(waypoints->at(i));
        if (smallest_curr_distance > curr_distance)
        {
            closest_waypoint = i;
            smallest_curr_distance = curr_distance;
        }
    }
}

void local_path_planning_node::compute_path_polygon()
{

    if (waypoints_segmentation->empty())
    {
        std::cout << red << "Warning: waypoints segmentation is not available. Skipping polygon path creation." << reset << std::endl;
        return;
    }

    segment_path->points.clear();

    // Prepare Marker for visualization
    visualization_msgs::msg::Marker lane_maker;
    lane_maker.header.frame_id = "map";
    lane_maker.header.stamp = this->now();

    lane_maker.ns = "vehicle_path";
    lane_maker.id = 0;
    lane_maker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    lane_maker.action = visualization_msgs::msg::Marker::ADD;
    lane_maker.scale.x = 0.07;
    lane_maker.scale.y = 0.5;
    lane_maker.scale.z = 0.5;
    lane_maker.color.a = 1.0;

    lane_maker.color.r = 1.0;
    lane_maker.color.g = 0.0;
    lane_maker.color.b = 0.0;

    // Publish lane with the track of the path for the left side
    for (size_t i = 0; i < waypoints_segmentation->size(); ++i)
    {
        double angle = waypoints_segmentation->at(i)(3);
        double offset_x = track_car * cos(angle + M_PI / 2);
        double offset_y = track_car * sin(angle + M_PI / 2);

        geometry_msgs::msg::Point32 converted_point;
        converted_point.x = waypoints_segmentation->at(i)(0) + offset_x;
        converted_point.y = waypoints_segmentation->at(i)(1) + offset_y;
        converted_point.z = 0.0;
        segment_path->points.push_back(converted_point);

        geometry_msgs::msg::Point left_point;
        left_point.x = converted_point.x;
        left_point.y = converted_point.y;
        left_point.z = 0.0;
        lane_maker.points.push_back(left_point);
    }

    //  invertion of the right side to math with the left side
    for (size_t i = waypoints_segmentation->size(); i-- > 0;)
    {
        double angle = waypoints_segmentation->at(i)(3);
        double offset_x = track_car * cos(angle + M_PI / 2);
        double offset_y = track_car * sin(angle + M_PI / 2);

        geometry_msgs::msg::Point32 converted_point;
        converted_point.x = waypoints_segmentation->at(i)(0) - offset_x;
        converted_point.y = waypoints_segmentation->at(i)(1) - offset_y;
        converted_point.z = 0.0;
        segment_path->points.push_back(converted_point);

        geometry_msgs::msg::Point right_point;
        right_point.x = converted_point.x;
        right_point.y = converted_point.y;
        right_point.z = 0.0;
        lane_maker.points.push_back(right_point);
    }

    segment_path->points.push_back(segment_path->points.front());
    lane_maker.points.push_back(lane_maker.points.front());
    lane_steering_publisher_->publish(lane_maker);
}

double
local_path_planning_node::getDistanceFromOdom(Eigen::VectorXd wapointPoint)
{
    double x1 = wapointPoint(0);
    double y1 = wapointPoint(1);
    double x2 = car_state_->x;
    double y2 = car_state_->y;
    double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    return distance;
}

void local_path_planning_node::sampleLayersAlongPath()
{
    if (waypoints_segmentation->empty())
    {
        std::cout << red << "Warning: waypoints segmentation is not available. Skipping polygon path creation." << reset << std::endl;
        return;
    }

    if (rescaled_chunk_->data.empty())
    {
        std::cout << red << "Warning: combined map is not available. Skipping polygon path creation." << reset << std::endl;
        return;
    }

    // MarkerArray to hold all markers
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;

    grid_map_ = std::make_shared<Grid_map>(*rescaled_chunk_);
    std::vector<std::vector<Eigen::VectorXd>> layers_samples;

    // Vector to store accumulated costs for each sample point
    std::vector<std::vector<double>> accumulated_costs;
    std::vector<std::vector<int>> predecessors;

    for (size_t layer_idx = 0; layer_idx < waypoints_segmentation->size(); ++layer_idx)
    {
        const auto &waypoint = waypoints_segmentation->at(layer_idx);
        double x = waypoint(0);
        double y = waypoint(1);
        double yaw = waypoint(3);
        std::vector<Eigen::VectorXd> layer_points;
        std::vector<double> layer_costs;
        std::vector<int> layer_predecessors;

        for (double lateral_offset = -lateral_range; lateral_offset <= lateral_range; lateral_offset += lateral_spacing)
        {
            Eigen::VectorXd sample_point(3);
            sample_point(0) = x + lateral_offset * cos(yaw + M_PI_2);
            sample_point(1) = y + lateral_offset * sin(yaw + M_PI_2);
            sample_point(2) = yaw;

            Eigen::Vector2d pos(sample_point(0), sample_point(1));
            double obstacle_distance = grid_map_->getObstacleDistance(pos);

            if (obstacle_distance > search_threshold)
            {
                layer_points.push_back(sample_point);

                // Calculate cost to this point from previous layer
                double min_cost = std::numeric_limits<double>::max();
                int best_predecessor = -1;

                if (!accumulated_costs.empty())
                {
                    const auto &previous_layer_points = layers_samples.back();
                    const auto &previous_layer_costs = accumulated_costs.back();

                    for (size_t prev_idx = 0; prev_idx < previous_layer_points.size(); ++prev_idx)
                    {
                        double cost = computeCost(sample_point, previous_layer_points[prev_idx]) + previous_layer_costs[prev_idx];
                        if (cost < min_cost)
                        {
                            min_cost = cost;
                            best_predecessor = prev_idx;
                        }
                    }
                }
                else
                {
                    min_cost = 0.0; // Initial layer has no predecessor
                }

                layer_costs.push_back(min_cost);
                layer_predecessors.push_back(best_predecessor);

                // Create a marker for this point
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map"; // Set appropriate frame ID
                marker.header.stamp = rclcpp::Clock().now();
                marker.ns = "sampled_points";
                marker.id = marker_id++;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = sample_point(0);
                marker.pose.position.y = sample_point(1);
                marker.pose.position.z = 0.0; // Adjust if needed
                marker.scale.x = 0.4;         // Diameter of the sphere
                marker.scale.y = 0.4;
                marker.scale.z = 0.4;
                marker.color.a = 1.0; // Alpha (1.0 is fully opaque)
                marker.color.r = 0.0; // Customize color as desired
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.lifetime = rclcpp::Duration(0, 0); // Persistent markers
                marker_array.markers.push_back(marker);
            }
        }

        // Save layer information for later path extraction
        layers_samples.push_back(layer_points);
        accumulated_costs.push_back(layer_costs);
        predecessors.push_back(layer_predecessors);
    }

    // Retrieve optimal path by backtracking from the final layer
    std::vector<Eigen::VectorXd> optimal_path;
    if (!accumulated_costs.empty())
    {
        double min_final_cost = std::numeric_limits<double>::max();
        int best_final_index = -1;

        // Find the point in the last layer with the minimum cost
        for (size_t i = 0; i < accumulated_costs.back().size(); ++i)
        {
            if (accumulated_costs.back()[i] < min_final_cost)
            {
                min_final_cost = accumulated_costs.back()[i];
                best_final_index = i;
            }
        }

        // Backtrack to reconstruct the optimal path
        int layer_idx = accumulated_costs.size() - 1;
        while (layer_idx >= 0 && best_final_index >= 0)
        {
            optimal_path.push_back(layers_samples[layer_idx][best_final_index]);
            best_final_index = predecessors[layer_idx][best_final_index];
            --layer_idx;
        }

        std::reverse(optimal_path.begin(), optimal_path.end());
    }

    // cout the size of the optimal path
    cout << "Optimal path size: " << optimal_path.size() << endl;
    // Publish add the optimal path to the marker array
    for (size_t i = 0; i < optimal_path.size(); ++i)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // Set appropriate frame ID
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "optimal_path";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = optimal_path[i](0);
        marker.pose.position.y = optimal_path[i](1);
        marker.pose.position.z = 0.0; // Adjust if needed
        marker.scale.x = 0.5;         // Diameter of the sphere
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0; // Alpha (1.0 is fully opaque)
        marker.color.r = 1.0; // Customize color as desired
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.lifetime = rclcpp::Duration(0, 0); // Persistent markers
        marker_array.markers.push_back(marker);
    }

    // Publish markers for the sampled points
    crosswalk_marker_publisher_->publish(marker_array);

    // Additional: Publish or log optimal path if needed
}

double local_path_planning_node::computeCost(const Eigen::VectorXd &point, const Eigen::VectorXd &prev_point)
{
    // Example costs based on deviation and obstacle proximity
    double obstacle_distance = grid_map_->getObstacleDistance(Eigen::Vector2d(point(0), point(1)));
    double deviation_cost = fabs(point(1) - prev_point(1)); // Penalize large lateral changes
    double obstacle_cost = obstacle_distance < search_threshold ? (1.0 - (obstacle_distance / search_threshold)) : 0.0;
    double smoothness_cost = fabs(point(2) - prev_point(2)); // Penalize heading changes

    return deviation_cost * 0.5 + obstacle_cost * 1.0 + smoothness_cost * 0.3; // Weighted sum of costs
}

void local_path_planning_node::waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    waypoints->clear();

    for (const auto &marker : msg->markers)
    {
        Eigen::VectorXd waypoint(4);
        waypoint(0) = marker.pose.position.x;
        waypoint(1) = marker.pose.position.y;
        waypoint(2) = marker.pose.position.z;

        tf2::Quaternion q(
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w);

        // Extract yaw from the quaternion
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        waypoint(3) = yaw;

        waypoints->push_back(waypoint);
    }
}

void local_path_planning_node::map_combination(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg)
{
    if (!global_map_)
    {
        std::cout << red << "Warning: global_map is not available. Skipping map combination." << reset << std::endl;
        return;
    }
    // clean the rescaled_chunk_
    rescaled_chunk_->data.clear();

    int chunk_size = 40; // 20 x 20 meters
    int chunk_radius = chunk_size / 2;

    // Convert car state to grid coordinates
    int car_x_grid = static_cast<int>((car_state_->x - global_map_->info.origin.position.x) / global_map_->info.resolution);
    int car_y_grid = static_cast<int>((car_state_->y - global_map_->info.origin.position.y) / global_map_->info.resolution);

    // Define chunk boundaries
    int min_x = std::max(0, car_x_grid - chunk_radius);
    int max_x = std::min(static_cast<int>(global_map_->info.width), car_x_grid + chunk_radius);
    int min_y = std::max(0, car_y_grid - chunk_radius);
    int max_y = std::min(static_cast<int>(global_map_->info.height), car_y_grid + chunk_radius);

    // Initialize the chunk grid
    nav_msgs::msg::OccupancyGrid chunk;
    chunk.header = global_map_->header;
    chunk.info.resolution = global_map_->info.resolution;
    chunk.info.width = max_x - min_x;
    chunk.info.height = max_y - min_y;
    chunk.info.origin.position.x = global_map_->info.origin.position.x + min_x * global_map_->info.resolution;
    chunk.info.origin.position.y = global_map_->info.origin.position.y + min_y * global_map_->info.resolution;
    chunk.info.origin.position.z = car_state_->z;
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
    rescaled_chunk_->info.origin.position.z = car_state_->z;
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
    double cos_heading = cos(car_state_->heading);
    double sin_heading = sin(car_state_->heading);

    for (size_t i = 0; i < msg->obstacles.size(); ++i)
    {
        const auto &obstacle = msg->obstacles[i];
        for (size_t j = 0; j < obstacle.polygon.points.size(); ++j)
        {
            auto &current_point_lidar = obstacle.polygon.points[j];
            auto &next_point_lidar = obstacle.polygon.points[(j + 1) % obstacle.polygon.points.size()];

            geometry_msgs::msg::Point current_point_map;
            current_point_map.x = car_state_->x + cos_heading * current_point_lidar.x - sin_heading * current_point_lidar.y;
            current_point_map.y = car_state_->y + sin_heading * current_point_lidar.x + cos_heading * current_point_lidar.y;
            geometry_msgs::msg::Point next_point_map;
            next_point_map.x = car_state_->x + cos_heading * next_point_lidar.x - sin_heading * next_point_lidar.y;
            next_point_map.y = car_state_->y + sin_heading * next_point_lidar.x + cos_heading * next_point_lidar.y;

            int x0 = static_cast<int>((current_point_map.x - rescaled_chunk_->info.origin.position.x) / rescaled_chunk_->info.resolution);
            int y0 = static_cast<int>((current_point_map.y - rescaled_chunk_->info.origin.position.y) / rescaled_chunk_->info.resolution);
            int x1 = static_cast<int>((next_point_map.x - rescaled_chunk_->info.origin.position.x) / rescaled_chunk_->info.resolution);
            int y1 = static_cast<int>((next_point_map.y - rescaled_chunk_->info.origin.position.y) / rescaled_chunk_->info.resolution);

            draw_inflated_line(x0, y0, x1, y1, inflation_radius, value_to_mark);
        }
    }

    occupancy_grid_pub_test_->publish(*rescaled_chunk_);
}

// ============================== Main Function for map rescale ==============================
cv::Mat local_path_planning_node::toMat(const nav_msgs::msg::OccupancyGrid &map)
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

cv::Mat local_path_planning_node::rescaleChunk(const cv::Mat &chunk_mat, double scale_factor)
{
    cv::Mat rescaled_chunk;
    cv::resize(chunk_mat, rescaled_chunk, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);
    return rescaled_chunk;
}

vector<pair<double, double>> local_path_planning_node::calculate_trajectory(double steering_angle, double wheelbase, int num_points)
{
    vector<pair<double, double>> path;

    if (fabs(steering_angle) < 1e-6)
    {
        double step_length = 0.5;
        for (int i = 0; i < num_points; ++i)
        {
            path.push_back(make_pair(start_offset + i * step_length, 0.0));
        }
    }
    else
    {
        double radius = wheelbase / tan(fabs(steering_angle));
        double angular_step = M_PI / 2 / num_points;

        double theta = 0; // Starting angle
        for (int i = 0; i <= num_points; ++i)
        {
            double x = radius * sin(theta) + start_offset; // Start start_offset m in front
            double y = radius * (1 - cos(theta));

            if (steering_angle < 0)
            {
                y = -y;
            }

            path.push_back(make_pair(x, y));
            theta += angular_step;
        }
    }

    return path;
}

void local_path_planning_node::extract_segment(const std::vector<std::pair<double, double>> &path, std::vector<double> &segment_x, std::vector<double> &segment_y, double length)
{
    if (path.empty())
        return;

    segment_x.push_back(path.front().first);
    segment_y.push_back(path.front().second);

    double accumulated_length = 0.0;
    double previous_x = path.front().first;
    double previous_y = path.front().second;

    for (size_t i = 1; i < path.size() && accumulated_length < length; ++i)
    {
        double current_x = path[i].first;
        double current_y = path[i].second;

        double dx = current_x - previous_x;
        double dy = current_y - previous_y;
        double segment_length = sqrt(dx * dx + dy * dy);

        accumulated_length += segment_length;

        if (accumulated_length >= length)
        {
            double excess_length = accumulated_length - length;
            double ratio = segment_length > 0 ? (segment_length - excess_length) / segment_length : 0;
            current_x = previous_x + ratio * dx;
            current_y = previous_y + ratio * dy;

            segment_x.push_back(current_x);
            segment_y.push_back(current_y);
            break;
        }
        else
        {
            segment_x.push_back(current_x);
            segment_y.push_back(current_y);
            previous_x = current_x;
            previous_y = current_y;
        }
    }
}

// callback functions
void local_path_planning_node::roadElementsCallback(const traffic_information_msgs::msg::RoadElementsCollection::SharedPtr msg)
{

    if (waypoints->empty())
    {
        std::cout << red << "Warning: waypoints is not available. Skipping crosswalk calculation." << reset << std::endl;
        return;
    }

    waypoints_segmentation->clear();

    traffic_information_msgs::msg::RoadElements first_crosswalk;
    bool found_first_crosswalk = false;

    // Pre-define a 1x1 meter square polygon at the origin
    geometry_msgs::msg::Polygon base_square;
    std::vector<std::pair<double, double>> square_offsets = {{-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}};
    for (const auto &offset : square_offsets)
    {
        geometry_msgs::msg::Point32 p;
        p.x = offset.first;
        p.y = offset.second;
        base_square.points.push_back(p);
    }
    // Close the square by connecting back to the first point
    base_square.points.push_back(base_square.points[0]);

    for (size_t i = closest_waypoint; i < waypoints->size(); ++i)
    {
        const auto &waypoint = waypoints->at(i);
        double x = waypoint(0);
        double y = waypoint(1);

        // push bach of the waypoints are btw segment_start_index_temp and segment_start_index
        if (i >= segment_start_index_temp && i <= segment_start_index)
        {
            waypoints_segmentation->push_back(waypoint);
        }

        // translated square polygon around the current waypoint
        geometry_msgs::msg::Polygon waypoint_poly = base_square;
        for (auto &point : waypoint_poly.points)
        {
            point.x += x;
            point.y += y;
        }

        for (const auto &element : msg->polygons)
        {
            // if (element.type == "crosswalk" && element.id != 363 && element.id != 391 && element.id != 572 && element.id != 638 && element.id != 631)
            if (element.type == "crosswalk" && std::find(skip_ids.begin(), skip_ids.end(), element.id) == skip_ids.end())
            {
                // Create a polygon from the crosswalk points
                geometry_msgs::msg::Polygon obstacle_poly;
                for (const auto &point : element.points)
                {
                    geometry_msgs::msg::Point32 p;
                    p.x = point.x;
                    p.y = point.y;
                    obstacle_poly.points.push_back(p);
                }

                // Check for collision only if the crosswalk is near the waypoint (e.g., within 5 meters)
                // when the crosswalk is bigger than the waypoint i do not now it thi will work
                if (std::abs(element.points[0].x - x) < 5 && std::abs(element.points[0].y - y) < 5)
                {
                    bool current_collision = collision_checker.check_collision(waypoint_poly, obstacle_poly);

                    if (current_collision)
                    {
                        first_crosswalk = element;
                        found_first_crosswalk = true;
                        segment_start_index = i;
                        break;
                    }
                }
            }
        }
        if (found_first_crosswalk)
        {
            break;
        }
    }

    // check if the crosswalk has been visited
    crosswalk_visited_check(first_crosswalk);
    compute_path_polygon();

    // Output the ID of the nearest crosswalk
    // cout << "First crosswalk id: " << first_crosswalk.id << endl;
    // cout << "Segment start index: " << segment_start_index_temp << endl;
    // cout << "Segment end index: " << segment_start_index << endl;
}

void local_path_planning_node::obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg)
{

    // Check if vehicle_path is available or valid
    if (vehicle_path->points.empty())
    {
        std::cout << red << "Warning: vehicle_path is not available. Skipping collision check." << reset << std::endl;
        return;
    }

    auto init_time = std::chrono::system_clock::now();

    getCurrentRobotState();

    collision_vector->clear();

    collision_detected_trajectory = false;
    collision_detected_path = false;
    for (const auto &obstacle : msg->obstacles)
    {
        bool current_collision = collision_checker.check_collision(*vehicle_path, obstacle.polygon);
        collision_vector->push_back(current_collision);
        if (current_collision)
        {
            collision_detected_trajectory = true;
        }
    }

    // std::cout << green << "---> Collision detected trajectory: " << collision_detected_trajectory << reset << std::endl;

    map_combination(msg);
    sampleLayersAlongPath();

    // if (collision_detected_trajectory)
    // {
    //     map_combination(msg);
    // }

    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    cout << blue << "Execution time for collision checker and map conbination: " << duration << " ms" << reset << endl;
}

void local_path_planning_node::yawCarCallback(const std_msgs::msg::Float64::SharedPtr msg)
{

    // auto init_time = std::chrono::system_clock::now();

    vehicle_path->points.clear();

    // Calculate the trajectory of the car in base of the yaw angle
    vector<pair<double, double>> trajectory = calculate_trajectory(msg->data, turning_radius, num_points);

    // extract the segment of the trajectory
    std::vector<double> segment_x, segment_y;
    extract_segment(trajectory, segment_x, segment_y, 4.0);

    // Prepare Marker for visualization
    visualization_msgs::msg::Marker lane_maker;
    lane_maker.header.frame_id = "velodyne";
    lane_maker.header.stamp = this->now();

    lane_maker.ns = "vehicle_path";
    lane_maker.id = 0;
    lane_maker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    lane_maker.action = visualization_msgs::msg::Marker::ADD;
    lane_maker.scale.x = 0.07;
    lane_maker.scale.y = 0.5;
    lane_maker.scale.z = 0.5;
    lane_maker.color.a = 1.0;

    if (collision_detected_trajectory)
    {
        lane_maker.color.r = 1.0;
        lane_maker.color.g = 0.0;
        lane_maker.color.b = 0.0;
    }
    else
    {
        lane_maker.color.r = 0.0;
        lane_maker.color.g = 1.0;
        lane_maker.color.b = 0.0;
    }

    // Publish lane with the track of the car for the left side
    for (size_t i = 0; i < segment_x.size(); ++i)
    {
        double angle = atan2(segment_y[i] - segment_y[std::max(int(i) - 1, 0)], segment_x[i] - segment_x[std::max(int(i) - 1, 0)]);
        double offset_x = track_car * cos(angle + M_PI / 2);
        double offset_y = track_car * sin(angle + M_PI / 2);

        geometry_msgs::msg::Point32 converted_point;
        converted_point.x = segment_x[i] + offset_x;
        converted_point.y = segment_y[i] + offset_y;
        converted_point.z = 0.0;
        vehicle_path->points.push_back(converted_point);

        geometry_msgs::msg::Point left_point;
        left_point.x = converted_point.x;
        left_point.y = converted_point.y;
        left_point.z = 0.0;
        lane_maker.points.push_back(left_point);
    }

    //  invertion of the right side to math with the left side
    for (size_t i = segment_x.size(); i-- > 0;)
    {
        int previous_index = (i > 0) ? (i - 1) : 0;
        double angle = atan2(segment_y[i] - segment_y[previous_index], segment_x[i] - segment_x[previous_index]);
        double offset_x = track_car * cos(angle + M_PI / 2);
        double offset_y = track_car * sin(angle + M_PI / 2);

        geometry_msgs::msg::Point32 converted_point;
        converted_point.x = segment_x[i] - offset_x;
        converted_point.y = segment_y[i] - offset_y;
        converted_point.z = 0.0;
        vehicle_path->points.push_back(converted_point);

        geometry_msgs::msg::Point right_point;
        right_point.x = converted_point.x;
        right_point.y = converted_point.y;
        right_point.z = 0.0;
        lane_maker.points.push_back(right_point);
    }

    vehicle_path->points.push_back(vehicle_path->points.front());

    lane_maker.points.push_back(lane_maker.points.front());

    // auto end_time = std::chrono::system_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - init_time).count();
    // cout << blue << "Execution time for path creation: " << duration << " ms" << reset << endl;

    // lane_steering_publisher_->publish(lane_maker);
}

void local_path_planning_node::globalMap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    global_map_ = map;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<local_path_planning_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}