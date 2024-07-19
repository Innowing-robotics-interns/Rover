#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h> 

class ProjectedPCLNode : public rclcpp::Node {
public:
    ProjectedPCLNode() : Node("projected_pcl_node") {
        // // Initialize parameters from the launch file or default values
        // this->declare_parameter("target_frame", "livox_frame");
        // this->declare_parameter("transform_tolerance", 0.01);
        // this->declare_parameter("min_height", -10.0);
        // this->declare_parameter("max_height", 1.0);
        // this->declare_parameter("angle_min", -3.1416); // -M_PI/2
        // this->declare_parameter("angle_max", 3.1416);  // M_PI/2
        // this->declare_parameter("angle_increment", 0.0087); // M_PI/360.0
        // this->declare_parameter("scan_time", 0.3333);
        // this->declare_parameter("range_min", 0.0);
        // this->declare_parameter("range_max", 5.0);
        // this->declare_parameter("use_inf", true);
        // this->declare_parameter("inf_epsilon", 1.0);

        // Subscribe to input point cloud topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10, std::bind(&ProjectedPCLNode::filterCallback, this, std::placeholders::_1));

        // Publisher for the projected 2D point cloud
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
    }

private:
    void filterCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Convert PointCloud2 to pcl::PointCloud
        pcl::fromROSMsg(*msg, *cloud);

        // Height Filtering
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-10.0, 1.0); // Assuming -10.0m is the min height, 1.0m is the max height
        pass.filter(*cloud_filtered);

        // Downsample the filtered cloud to reduce density
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        vg.setInputCloud(cloud_filtered);
        vg.setLeafSize(0.1f, 0.1f, 0.1f); // Set the voxel size to 10cm x 10cm x 10cm
        vg.filter(*cloud_downsampled);

        // Project the downsampled point cloud to a 2D plane
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        coefficients->values.resize(4);
        coefficients->values[0] = coefficients->values[1] = 0;
        coefficients->values[2] = 1.0; // Project onto the XY plane
        coefficients->values[3] = 0;

        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud_downsampled); // Use the downsampled cloud
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_projected);

        // Range Filtering on the projected (and downsampled) cloud
        for (const auto& point : *cloud_projected) {
            float distance = sqrt(pow(point.x, 2) + pow(point.y, 2));
            if (distance <= 5.0) { // Ensure points within 3 meters are considered
                temp_cloud->push_back(point);
            }
        }

        // Assign the result of range filtering to cloud_projected
        *cloud_projected = *temp_cloud;

        // Convert to ROS data type
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_projected, output);
        output.header.frame_id = "livox_frame"; // Use the frame ID from parameters

        // Publish the projected point cloud
        publisher_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::cout << "Starting projected_pcl_node..." << std::endl;
    auto node = std::make_shared<ProjectedPCLNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}