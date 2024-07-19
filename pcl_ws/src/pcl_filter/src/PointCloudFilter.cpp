#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <unordered_set>
#include <cmath>

class PointCloudFilter : public rclcpp::Node
{
public:
    PointCloudFilter() : Node("point_cloud_filter")
    {
        // Initialize the publisher for the filtered point cloud
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_clouds", 10);

        // Subscribe to the original point cloud topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", 10, std::bind(&PointCloudFilter::cloud_callback, this, std::placeholders::_1));

        // Subscribe to cluster topics and store cluster points
        for (int i = 0; i <= 20; ++i) {
            auto callback = [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->cluster_callback(i, msg);
            };
            cluster_subscriptions_.push_back(
                this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    "cluster_" + std::to_string(i), 10, callback));
        }
    }

private:
    // Initialize struct for timestamp
    struct ClusterData {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        std::chrono::steady_clock::time_point last_update;
    };

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::vector<ClusterData> cluster_data_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cluster_subscriptions_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_points_; // Store cluster points

    void cluster_callback(int cluster_id, const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (cluster_id >= static_cast<int>(cluster_data_.size())) {
            cluster_data_.resize(cluster_id + 1);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        cluster_data_[cluster_id].cloud = cloud;
        cluster_data_[cluster_id].last_update = std::chrono::steady_clock::now();
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Threshold for considering a point to be overlapping with a cluster
        const float distance_threshold = 0.5; // Adjust as necessary

        for (const auto& point : cloud->points) {
            bool keep_point = true;
            for (const auto& cluster : cluster_data_) {
                for (const auto& cluster_point : cluster.cloud->points) {
                    if (pcl::euclideanDistance(point, cluster_point) < distance_threshold) {
                        keep_point = false;
                        break;
                    }
                }
                if (!keep_point) break;
            }
            if (keep_point) {
                filtered_cloud->points.push_back(point);
            }
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.frame_id = "livox_frame";
        publisher_->publish(output);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "Starting point cloud filter node..." << std::endl;
    auto node = std::make_shared<PointCloudFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}