// #pragma once

#include <rclcpp/rclcpp.hpp>

// Ros2
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>

// C++
#include <iostream>
#include <vector>
#include <algorithm>

// Eigen
#include <Eigen/Dense>

using namespace std;

class VoxelGrid : public rclcpp::Node
{
private:
    // voxelgrid resolution
    float voxel_leaf_size_x_ = 0.0;
    float voxel_leaf_size_y_ = 0.0;
    float voxel_leaf_size_z_ = 0.0;

    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    using PointCloudMsg2 = sensor_msgs::msg::PointCloud2;

    // ROI boundaries
    // The velodyne is mounted on the car with a bad orientation, so the x is the y axis.
    double roi_max_x_ = 0.0; // FRONT THE CAR
    double roi_max_y_ = 0.0; // LEFT THE CAR
    double roi_max_z_ = 0.0; // UP THE VELODYNE

    double roi_min_x_ = 0.0; // RIGHT THE CAR
    double roi_min_y_ = 0.0; // BACK THE CAR
    double roi_min_z_ = 0.0; // DOWN THE VELODYNE

    bool voxel_condition = false;

    Eigen::Vector4f ROI_MAX_POINT, ROI_MIN_POINT;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr roi_sampling_pub_;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

public:
    VoxelGrid(/* args */);
    ~VoxelGrid();
};

VoxelGrid::VoxelGrid(/* args */) : Node("voxel_grid_filter")
{

    this->declare_parameter("voxel_leaf_size_x", double(0.0));
    this->declare_parameter("voxel_leaf_size_y", double(0.0));
    this->declare_parameter("voxel_leaf_size_z", double(0.0));

    this->declare_parameter("roi_max_x_", double(0.0));
    this->declare_parameter("roi_max_y_", double(0.0));
    this->declare_parameter("roi_max_z_", double(0.0));

    this->declare_parameter("roi_min_x_", double(0.0));
    this->declare_parameter("roi_min_y_", double(0.0));
    this->declare_parameter("roi_min_z_", double(0.0));

    this->declare_parameter("voxel_condition", false);

    this->get_parameter("voxel_leaf_size_x", voxel_leaf_size_x_);
    this->get_parameter("voxel_leaf_size_y", voxel_leaf_size_y_);
    this->get_parameter("voxel_leaf_size_z", voxel_leaf_size_z_);

    this->get_parameter("roi_max_x_", roi_max_x_);
    this->get_parameter("roi_max_y_", roi_max_y_);
    this->get_parameter("roi_max_z_", roi_max_z_);

    this->get_parameter("roi_min_x_", roi_min_x_);
    this->get_parameter("roi_min_y_", roi_min_y_);
    this->get_parameter("roi_min_z_", roi_min_z_);

    this->get_parameter("voxel_condition", voxel_condition);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_rotated", 10, std::bind(&VoxelGrid::pointCloudCallback, this, std::placeholders::_1));

    roi_sampling_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_roi", 10);

    ROI_MAX_POINT = Eigen::Vector4f(roi_max_x_, roi_max_y_, roi_max_z_, 1);
    ROI_MIN_POINT = Eigen::Vector4f(roi_min_x_, roi_min_y_, roi_min_z_, 1);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> voxel_grid_node initialized.\033[0m");

    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->voxel_leaf_size_x: %f \033[0m", voxel_leaf_size_x_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->voxel_leaf_size_y: %f \033[0m", voxel_leaf_size_y_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->voxel_leaf_size_z: %f \033[0m", voxel_leaf_size_z_);

    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->roi_max_x: %f \033[0m", roi_max_x_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->roi_max_y: %f \033[0m", roi_max_y_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->roi_max_z: %f \033[0m", roi_max_z_);

    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->roi_min_x: %f \033[0m", roi_min_x_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->roi_min_y: %f \033[0m", roi_min_y_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->roi_min_z: %f \033[0m", roi_min_z_);

    RCLCPP_INFO(this->get_logger(), "\033[1;34m---->voxel_condition: %d \033[0m", voxel_condition);
}

VoxelGrid::~VoxelGrid()
{
}

void VoxelGrid::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *input_cloud);

    // Apply ROI filtering
    pcl::CropBox<pcl::PointXYZI> roi_filter;
    roi_filter.setInputCloud(input_cloud);
    roi_filter.setMax(ROI_MAX_POINT);
    roi_filter.setMin(ROI_MIN_POINT);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);
    roi_filter.filter(*cloud_roi);

    if (voxel_condition)
    {

        // create voxel grid object
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud_roi);
        vg.setLeafSize(voxel_leaf_size_x_, voxel_leaf_size_y_, voxel_leaf_size_z_);

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        vg.filter(*filtered_cloud);

        // convert back to ROS datatype
        VoxelGrid::PointCloudMsg downsampled_cloud_msg;
        pcl::toROSMsg(*filtered_cloud, downsampled_cloud_msg);

        roi_sampling_pub_->publish(downsampled_cloud_msg);
    }
    else
    {
        VoxelGrid::PointCloudMsg2 downsampled_cloud_msg_rio;
        pcl::toROSMsg(*cloud_roi, downsampled_cloud_msg_rio);

        roi_sampling_pub_->publish(downsampled_cloud_msg_rio);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoxelGrid>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}