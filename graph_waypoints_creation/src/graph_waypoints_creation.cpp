
#include "graph_waypoints_creation.h"

graph_waypoints_creation::graph_waypoints_creation(/* args */) : Node("graph_waypoints_creation"), tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer)
{

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&graph_waypoints_creation::pub_callback, this));
    waypoint_saver_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", 10);
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

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> graph_waypoints_creation_node initialized.\033[0m");
}

graph_waypoints_creation::~graph_waypoints_creation()
{
    ofs_.close(); // Close the file
}

void graph_waypoints_creation::getCurrentRobotState()
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

        if (!first_data_received_)
        {
            first_data_received_ = true;
            writePoseToFile(*car_state_, false);
            RCLCPP_INFO(this->get_logger(), "First data received!");
            previous_pose_ = car_state_;
        }
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
        double distance = sqrt(pow(car_state_->x - previous_pose_->x, 2) + pow(car_state_->y - previous_pose_->y, 2));
        if (distance >= interval_)
        {
            writePoseToFile(*car_state_, true);
            previous_pose_ = car_state_;
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<graph_waypoints_creation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}