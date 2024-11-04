
#include <rclcpp/rclcpp.hpp>

// Ros2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>



// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

// C++
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>

// Eigen
#include <Eigen/Dense>

using namespace std;




class LidarGround : public rclcpp::Node
{
private:

    double treshold_ground_ = 0.0;
    /* data */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

public:
    LidarGround(/* args */);
    ~LidarGround();
};

LidarGround::LidarGround(/* args */): Node("lidar_ground_node")
{
    // for mty is 0.3 and for ccm is 0.2 becasue of the lidar.
    this->declare_parameter("treshold_ground_", double(0.0));
    this->get_parameter("treshold_ground_", treshold_ground_);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_roi", 10, std::bind(&LidarGround::pointCloudCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_removal", 10);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> lidar_ground_getter_node initialized.\033[0m");

    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> treshold_ground: %f \033[0m", treshold_ground_);

}

LidarGround::~LidarGround()
{
}

void LidarGround::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *input_cloud);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_segmented_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(treshold_ground_);

    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);


    extract.setNegative(true);
    extract.filter(*plane_segmented_cloud);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*plane_segmented_cloud, output);
    output.header.frame_id = msg->header.frame_id;
    pub_->publish(output);


}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarGround>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
