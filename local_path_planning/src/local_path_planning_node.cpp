#include "local_path_planning_node.h"

local_path_planning_node::local_path_planning_node(/* args */) : Node("local_path_planning_node"), tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer)
{

    obstacle_info_subscription_ = this->create_subscription<obstacles_information_msgs::msg::ObstacleCollection>(
        "/obstacle_info", 10, std::bind(&local_path_planning_node::obstacle_info_callback, this, std::placeholders::_1));

    road_elements_subscription_ = this->create_subscription<traffic_information_msgs::msg::RoadElementsCollection>(
        "/road_elements", 10, std::bind(&local_path_planning_node::roadElementsCallback, this, std::placeholders::_1));

    yaw_car_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/yaw_car", 10, std::bind(&local_path_planning_node::yawCarCallback, this, std::placeholders::_1));

    lane_steering_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "steering_lane", 10);

    waypoints_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/waypoints_routing", 10, std::bind(&local_path_planning_node::waypoints_callback, this, std::placeholders::_1));

    crosswalk_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("crosswalk_markers", 10);

    vehicle_path = std::make_shared<geometry_msgs::msg::Polygon>();
    collision_vector = std::make_shared<std::vector<bool>>();
    waypoints = std::make_shared<vector<Eigen::VectorXd>>();
    car_state_ = std::make_shared<State>();
    road_elements_ = std::make_shared<traffic_information_msgs::msg::RoadElementsCollection>();

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> local_path_planning_node initialized.\033[0m");
}

local_path_planning_node::~local_path_planning_node()
{
}

double local_path_planning_node::calculateDistance(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

void local_path_planning_node::roadElementsCallback(const traffic_information_msgs::msg::RoadElementsCollection::SharedPtr msg)
{
    // # RoadElementsCollection.msg
    // std_msgs/Header header
    // traffic_information_msgs / RoadElements[] polygons

    // # RoadElements.msg
    // polygon_msgs / Point2D[] points
    // int32 id
    // string type

    if (waypoints->empty())
    {
        std::cout << red << "Warning: waypoints is not available. Skipping collision check." << reset << std::endl;
        return;
    }

    double min_distance = std::numeric_limits<double>::max();
    traffic_information_msgs::msg::RoadElements nearest_crosswalk;
    visualization_msgs::msg::MarkerArray marker_array;

    for (const auto &element : msg->polygons)
    {
        if (element.type == "crosswalk")
        {
            double cx = 0.0;
            double cy = 0.0;
            int n = element.points.size();

            for (const auto &point : element.points)
            {
                cx += point.x;
                cy += point.y;
            }
            cx /= n;
            cy /= n;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map"; // Replace with appropriate frame if different
            marker.header.stamp = this->now();
            marker.ns = "crosswalk_centers";
            marker.id = element.id;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = cx;
            marker.pose.position.y = cy;
            marker.pose.position.z = 1.0; // Adjust height if necessary
            marker.scale.z = 1.0;         // Text height
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;

            // Set text to display the crosswalk ID and center coordinates
            marker.text = "ID: " + std::to_string(element.id) +
                          "\n(" + std::to_string(cx) + ", " + std::to_string(cy) + ")";

            // Add marker to marker array
            marker_array.markers.push_back(marker);
        }
    }

    const auto &first_waypoint = waypoints->at(0); // First waypoint position
    visualization_msgs::msg::Marker first_waypoint_marker;
    first_waypoint_marker.header.frame_id = "map"; // Replace with appropriate frame if different
    first_waypoint_marker.header.stamp = this->now();
    first_waypoint_marker.ns = "waypoints";
    first_waypoint_marker.id = 0; // Unique ID
    first_waypoint_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    first_waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
    first_waypoint_marker.pose.position.x = first_waypoint[0];
    first_waypoint_marker.pose.position.y = first_waypoint[1];
    first_waypoint_marker.pose.position.z = 1.5; // Adjust height if necessary
    first_waypoint_marker.scale.z = 1.0;         // Text height
    first_waypoint_marker.color.a = 1.0;
    first_waypoint_marker.color.r = 0.0;
    first_waypoint_marker.color.g = 1.0;
    first_waypoint_marker.color.b = 0.0;

    // Set text to "First" to label the first waypoint
    first_waypoint_marker.text = "First";

    // Add first waypoint marker to marker array
    marker_array.markers.push_back(first_waypoint_marker);

    // cout the id of the nearest crosswalk
    cout << "Nearest crosswalk id: " << nearest_crosswalk.id << endl;
    crosswalk_marker_publisher_->publish(marker_array);
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
    }

    catch (tf2::TransformException &ex)
    {
        std::cout << red << "Transform error: " << ex.what() << reset << std::endl;
    }
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

void local_path_planning_node::obstacle_info_callback(const obstacles_information_msgs::msg::ObstacleCollection::SharedPtr msg)
{

    collision_vector->clear();
    // Check if vehicle_path is available or valid
    if (vehicle_path->points.empty())
    {
        std::cout << red << "Warning: vehicle_path is not available. Skipping collision check." << reset << std::endl;
        return;
    }

    collision_detected = false;
    for (const auto &obstacle : msg->obstacles)
    {
        bool current_collision = collision_checker.check_collision(*vehicle_path, obstacle.polygon);
        collision_vector->push_back(current_collision);

        if (current_collision)
        {
            collision_detected = true;
        }
    }
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

    lane_steering_publisher_->publish(lane_maker);
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<local_path_planning_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}