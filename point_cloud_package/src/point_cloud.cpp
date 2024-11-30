#include "point_cloud.h"

point_cloud::point_cloud(/* args */): Node("point_cloud_node"), tf2_buffer(this->get_clock()), tf2_listener(tf2_buffer)
{

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_rotated", 10, std::bind(&point_cloud::pointCloudCallback, this, std::placeholders::_1));
    to_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_to_map", 10);

    std::cout << green << "----> point_cloud_node initialized" << reset << std::endl;

}

point_cloud::~point_cloud()
{
}

void point_cloud::getCurrentRobotState()
{
    geometry_msgs::msg::Transform pose_tf;
    try
    {
        pose_tf = tf2_buffer.lookupTransform("map", "velodyne", tf2::TimePointZero).transform;
        x_state = pose_tf.translation.x;
        y_state = pose_tf.translation.y;
        z_state = pose_tf.translation.z;
        tf2::Quaternion quat;
        tf2::fromMsg(pose_tf.rotation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        yaw_state = yaw;

    }
    catch (tf2::TransformException &ex)
    {
        std::cout << red << "Transform error: " << ex.what() << reset << std::endl;
    }
}


void point_cloud::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    getCurrentRobotState();

    // Convertir PointCloud2 a pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_in);

    // Crear la matriz de transformación
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << x_state, y_state, z_state;
    transform.rotate(Eigen::AngleAxisf(yaw_state, Eigen::Vector3f::UnitZ()));

    // Transformar la nube de puntos
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_in, *cloud_out, transform);

    // Convertir pcl::PointCloud de nuevo a PointCloud2
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_out, output_msg);

    // Asignar frame_id y timestamp
    output_msg.header.frame_id = "map";
    output_msg.header.stamp = this->get_clock()->now();

    // Publicar la nube transformada
    to_map_pub_->publish(output_msg);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<point_cloud>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}