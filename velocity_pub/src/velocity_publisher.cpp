#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

class VelocityPublisher : public rclcpp::Node
{
public:
    VelocityPublisher()
        : Node("velocity_publisher")
    {
        // Create a subscription to the `/vectornav/velocity_body` topic
        subscription_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/vectornav/velocity_body",
            10,
            std::bind(&VelocityPublisher::velocityCallback, this, std::placeholders::_1));

        // Create a publisher for the `/timer` topic
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/timer", 10);

        // Timer to publish velocity at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), // Publish at 1 Hz
            std::bind(&VelocityPublisher::publishVelocity, this));
    }

private:
    void velocityCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
    {
        // Extract linear velocity components
        double x = msg->twist.twist.linear.x;
        double y = msg->twist.twist.linear.y;
        double z = msg->twist.twist.linear.z;

        // Compute the total speed in m/s
        double speed_m_s = std::sqrt(x * x + y * y + z * z);

        // Convert to km/h
        current_speed_kmh_ = speed_m_s * 3.6;
    }

    void publishVelocity()
    {
        // Create a Float32 message
        auto message = std_msgs::msg::Float32();
        message.data = static_cast<float>(current_speed_kmh_);

        // Publish the message
        publisher_->publish(message);

        // Log the published speed
        RCLCPP_INFO(this->get_logger(), "Published Speed: %.2f km/h", current_speed_kmh_);
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double current_speed_kmh_ = 0.0; // Store the latest computed speed
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create the node and spin it
    auto node = std::make_shared<VelocityPublisher>();
    rclcpp::spin(node);

    // Shutdown the ROS2 system
    rclcpp::shutdown();
    return 0;
}
