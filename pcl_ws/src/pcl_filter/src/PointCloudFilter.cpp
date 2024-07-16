#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <unordered_set>

class PointCloudFilter : public rclcpp::Node
{
public:
    PointCloudFilter() : Node("point_cloud_filter")
    {
        // Subscribe to the original point cloud topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10, std::bind(&PointCloudFilter::cloud_callback, this, std::placeholders::_1));

        // Subscribe to cluster topics
        for (int i = 0; i < 6; ++i) {
            auto callback = [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->cluster_callback(i, msg);
            };
            cluster_subscriptions_.push_back(
                this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    "cluster_" + std::to_string(i), 10, callback));
        }

        // Publisher for the filtered point cloud
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_clouds", 10);
    }

private:
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds{6};

    void cluster_callback(int cluster_id, const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the incoming PointCloud2 to PCL::PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        cluster_clouds[cluster_id] = cloud;
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the original PointCloud2 to PCL::PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *original_cloud);

        // Filter out points that are present in any of the cluster clouds
        pcl::PointIndices::Ptr toRemove(new pcl::PointIndices);
        for (const auto& cluster_cloud : cluster_clouds) {
            if (cluster_cloud) {
                for (size_t i = 0; i < original_cloud->points.size(); ++i) {
                    for (const auto& point : cluster_cloud->points) {
                        if (original_cloud->points[i].x == point.x && original_cloud->points[i].y == point.y && original_cloud->points[i].z == point.z) {
                            toRemove->indices.push_back(i);
                            break;
                        }
                    }
                }
            }
        }

        // Remove the points from the original cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(original_cloud);
        extract.setIndices(toRemove);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*filtered_cloud);

        // Convert the filtered cloud back to PointCloud2 and publish it
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.frame_id = msg->header.frame_id; // Preserve the original frame ID
        publisher_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cluster_subscriptions_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
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