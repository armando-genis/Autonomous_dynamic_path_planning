#include "trajectory_obstacle_checker_node.h"

trajectory_obstacle_checker_node::trajectory_obstacle_checker_node(/* args */) : Node("trajectory_obstacle_checker_node")
{

    obstacle_info_subscription_ = this->create_subscription<obstacles_information_msgs::msg::ObstacleCollection>(
        "/obstacle_info", 10, std::bind(&trajectory_obstacle_checker_node::obstacle_info_callback, this, std::placeholders::_1));

    yaw_car_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/yaw_car", 10, std::bind(&trajectory_obstacle_checker_node::yawCarCallback, this, std::placeholders::_1));

    lane_steering_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "steering_lane", 10);

    // Publisher for debugging
    // lane_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/lane", 10);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> trajectory_obstacle_checker_node initialized.\033[0m");
}

trajectory_obstacle_checker_node::~trajectory_obstacle_checker_node()
{
}

void trajectory_obstacle_checker_node::obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg)
{

    collision_vector.clear();
    // Check if vehicle_path is available or valid
    if (vehicle_path.points.empty())
    {
        std::cout << red << "Warning: vehicle_path is not available. Skipping collision check." << reset << std::endl;
        return;
    }

    collision_detected = false;
    for (const auto &obstacle : msg->obstacles)
    {
        bool current_collision = collision_checker.check_collision(vehicle_path, obstacle.polygon);
        collision_vector.push_back(current_collision);

        if (current_collision)
        {
            collision_detected = true;
        }
    }

    // cout numnbers of obstacles
    // cout << "Number of obstacles: " << msg->obstacles.size() << endl;
}

void trajectory_obstacle_checker_node::yawCarCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    vehicle_path.points.clear();
    // Calculate the trajectory of the car in base of the yaw angle
    vector<pair<double, double>> trajectory = calculate_trajectory(msg->data, turning_radius, num_points);

    // extract the segment of the trajectory
    std::vector<double> segment_x, segment_y;
    extract_segment(trajectory, segment_x, segment_y, 4.0);

    // Publish the lane
    // nav_msgs::msg::Path path;
    // path.header.frame_id = "velodyne";
    // path.header.stamp = this->now();

    // for (size_t i = 0; i < segment_x.size(); ++i)
    // {
    //     geometry_msgs::msg::PoseStamped pose;
    //     pose.header.frame_id = "velodyne";
    //     pose.pose.position.x = segment_x[i];
    //     pose.pose.position.y = segment_y[i];
    //     path.poses.push_back(pose);
    // }

    // lane_publisher_->publish(path);

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

    if (collision_detected)
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
        vehicle_path.points.push_back(converted_point);

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
        vehicle_path.points.push_back(converted_point);

        geometry_msgs::msg::Point right_point;
        right_point.x = converted_point.x;
        right_point.y = converted_point.y;
        right_point.z = 0.0;
        lane_maker.points.push_back(right_point);
    }

    vehicle_path.points.push_back(vehicle_path.points.front());

    lane_maker.points.push_back(lane_maker.points.front());
    lane_steering_publisher_->publish(lane_maker);
}

// =============================== TRAYECTORY YAW CAR ===============================

vector<pair<double, double>> trajectory_obstacle_checker_node::calculate_trajectory(double steering_angle, double wheelbase, int num_points)
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
            double x = radius * sin(theta) + start_offset; // Start 0.5 meters in front
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

void trajectory_obstacle_checker_node::extract_segment(const std::vector<std::pair<double, double>> &path, std::vector<double> &segment_x, std::vector<double> &segment_y, double length)
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<trajectory_obstacle_checker_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}