
#include "graph_waypoints_creation.h"

graph_waypoints_creation::graph_waypoints_creation(/* args */) : Node("graph_waypoints_creation"), tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer)
{

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&graph_waypoints_creation::pub_callback, this));
    waypoint_saver_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_creation", 10);
    waypoint_info_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_info", 10);

    car_state_ = std::make_shared<State>();
    previous_pose_ = std::make_shared<State>();

    std::filesystem::path base_path = std::filesystem::current_path();
    // create a foler in the base path and the points.csv file
    base_path /= "waypoints";
    std::filesystem::create_directory(base_path);
    std::filesystem::path file_path = base_path / "points.csv";
    file_path_ = file_path.string();

    // print the file path in green
    RCLCPP_INFO(this->get_logger(), "\033[1;32mFile path: %s\033[0m", file_path_.c_str());

    writeHeaderToFile();

    RCLCPP_INFO(this->get_logger(), "\033[1;32m--------------> graph_waypoints_creation_node initialized.\033[0m");
}

graph_waypoints_creation::~graph_waypoints_creation()
{
    ofs_.close(); // Close the file
}

double graph_waypoints_creation::computeDistance(const State &state1, const State &state2)
{
    double x1 = state1.x;
    double y1 = state1.y;
    double x2 = state2.x;
    double y2 = state2.y;
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void graph_waypoints_creation::getCurrentRobotState()
{
    geometry_msgs::msg::Transform pose_tf;
    try
    {
        pose_tf = tf2_buffer.lookupTransform("map", "velodyne", tf2::TimePointZero).transform;
        car_state_->x = pose_tf.translation.x;
        car_state_->y = pose_tf.translation.y;
        car_state_->z = pose_tf.translation.z;
        tf2::Quaternion quat;
        tf2::fromMsg(pose_tf.rotation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        car_state_->heading = yaw;

        if (!first_data_received_)
        {
            writePoseToFile(*car_state_, false);
            displayMarker(*car_state_);
            first_data_received_ = true;
            RCLCPP_INFO(this->get_logger(), "First data received!");
            previous_pose_ = std::make_shared<State>(*car_state_);
        }

        cout << blue << "Current pose: " << car_state_->x << " " << car_state_->y << " " << car_state_->z << " " << car_state_->heading << reset << endl;
    }

    catch (tf2::TransformException &ex)
    {
        std::cout << red << "Transform error: " << ex.what() << reset << std::endl;
    }
}

void graph_waypoints_creation::pub_callback()
{
    getCurrentRobotState();
    if (first_data_received_)
    {
        double distance = computeDistance(*car_state_, *previous_pose_);
        // std::cout << red << "<-- DISTANCE added -->" << distance << reset << std::endl;
        if (distance > interval_)
        {
            writePoseToFile(*car_state_, true);
            displayMarker(*car_state_);
            std::cout << green << "<-- new waypoint added -->" << reset << std::endl;
            previous_pose_ = std::make_shared<State>(*car_state_);
        }
    }
}

void graph_waypoints_creation::writeHeaderToFile()
{
    ofs_.open(file_path_, std::ios::out); // Open the file here
    if (!ofs_)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open the file: %s", file_path_.c_str());
        return;
    }
    ofs_ << "x,y,z,yaw,velocity,change_flag" << std::endl;
}

void graph_waypoints_creation::writePoseToFile(const State &odom_pos_, bool change_flag)
{
    if (ofs_)
    {
        double current_pose_yaw_ = odom_pos_.heading;
        ofs_ << std::fixed << std::setprecision(4)
             << odom_pos_.x << ","
             << odom_pos_.y << ","
             << odom_pos_.z << ","
             << current_pose_yaw_ << ","
             << 0 << ","
             << change_flag
             << std::endl;
    }
    RCLCPP_INFO(this->get_logger(), "Enter to writePoseToFile");
}

void graph_waypoints_creation::displayMarker(const State &state_to_display)
{
    static int id = 3000;
    static visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "waypoints";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = state_to_display.x;
    marker.pose.position.y = state_to_display.y;
    marker.pose.position.z = state_to_display.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.scale.z = 0.6;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);
    waypoint_saver_pub_->publish(marker_array);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<graph_waypoints_creation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}