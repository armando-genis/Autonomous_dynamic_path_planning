
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
    int num_seg_;
    int num_iter_;
    int num_lpr_;
    float th_seeds_;
    float th_dist_;
    float sensor_height_;

    /* data */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    struct Model
    {
        Eigen::MatrixXf normal;
        double d = 0.;
    };

    Model estimatePlane(const pcl::PointCloud<pcl::PointXYZI> &seed_points);
    void extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &seed_points);

public:
    LidarGround(/* args */);
    ~LidarGround();
};

LidarGround::LidarGround(/* args */) : Node("lidar_ground_node")
{
    this->declare_parameter("num_seg_", 50);
    this->declare_parameter("num_iter_", 25);
    this->declare_parameter("num_lpr_", 10);
    this->declare_parameter("th_seeds_", 1.0);
    this->declare_parameter("th_dist_", 0.3);
    this->declare_parameter("sensor_height_", 1.73);

    this->get_parameter("num_seg_", num_seg_);
    this->get_parameter("num_iter_", num_iter_);
    this->get_parameter("num_lpr_", num_lpr_);
    this->get_parameter("th_seeds_", th_seeds_);
    this->get_parameter("th_dist_", th_dist_);
    this->get_parameter("sensor_height_", sensor_height_);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_roi", 10, std::bind(&LidarGround::pointCloudCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_removal", 10);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> lidar_ground_getter_node initialized.\033[0m");

    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> num_seg: %d \033[0m", num_seg_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> num_iter: %d \033[0m", num_iter_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> num_lpr: %d \033[0m", num_lpr_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> th_seeds: %f \033[0m", th_seeds_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> th_dist: %f \033[0m", th_dist_);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> sensor_height: %f \033[0m", sensor_height_);
}

LidarGround::~LidarGround()
{
}

LidarGround::Model LidarGround::estimatePlane(const pcl::PointCloud<pcl::PointXYZI> &seed_points)
{
    Eigen::Matrix3f cov_matrix;
    Eigen::Vector4f centroid;
    pcl::computeMeanAndCovarianceMatrix(seed_points, cov_matrix, centroid);

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_matrix, Eigen::ComputeFullU);
    Model model;
    model.normal = svd.matrixU().col(2);
    model.d = -(model.normal.transpose() * centroid.head<3>())(0, 0);

    return model;
}

void LidarGround::extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &seed_points)
{
    std::vector<pcl::PointXYZI> cloud_sorted(cloud_in->points.begin(), cloud_in->points.end());
    std::sort(cloud_sorted.begin(), cloud_sorted.end(), [](const pcl::PointXYZI &a, const pcl::PointXYZI &b)
              { return a.z < b.z; });

    auto it = std::find_if(cloud_sorted.begin(), cloud_sorted.end(), [&](const pcl::PointXYZI &point)
                           { return point.z >= -1.5 * sensor_height_; });

    cloud_sorted.erase(cloud_sorted.begin(), it);

    double LPR_height = 0;
    for (int i = 0; i < num_lpr_ && i < static_cast<int>(cloud_sorted.size()); ++i)
    {
        LPR_height += cloud_sorted[i].z;
    }
    LPR_height /= std::max(1, num_lpr_);

    seed_points->points.clear();
    for (const auto &point : cloud_sorted)
    {
        if (point.z < LPR_height + th_seeds_)
        {
            seed_points->points.push_back(point);
        }
    }
}

void LidarGround::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *input_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points(new pcl::PointCloud<pcl::PointXYZI>());
    extractInitialSeeds(input_cloud, seed_points);

    Model model = estimatePlane(*seed_points);

    for (auto &point : input_cloud->points)
    {
        float dist = model.normal(0) * point.x + model.normal(1) * point.y + model.normal(2) * point.z + model.d;
        if (dist < th_dist_)
        {
            ground_points->points.push_back(point);
        }
        else
        {
            notground_points->points.push_back(point);
        }
    }

    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*notground_points, ground_msg);
    ground_msg.header.frame_id = msg->header.frame_id;
    ground_msg.header.stamp = this->now();
    pub_->publish(ground_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarGround>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
