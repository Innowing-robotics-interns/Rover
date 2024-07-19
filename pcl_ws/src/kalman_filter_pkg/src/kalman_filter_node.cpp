#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

class KalmanFilterNode : public rclcpp::Node {
public:
    KalmanFilterNode() : Node("kalman_filter_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", 10, std::bind(&KalmanFilterNode::pointCloudCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_clouds", 10);

        // Kalman Filter Initialization
        int stateSize = 4; // [x, y, vx, vy]
        int measSize = 2;  // [z_x, z_y]
        int contrSize = 0; // Control vector size (no control)
        unsigned int type = CV_32F;
        kf.init(stateSize, measSize, contrSize, type);

        // State Transition Matrix
        kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,   0, 1, 0, 1,   0, 0, 1, 0,   0, 0, 0, 1);
        // Measurement Matrix
        kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
        kf.measurementMatrix.at<float>(0) = 1.0f;
        kf.measurementMatrix.at<float>(5) = 1.0f;

        // Process Noise Covariance Matrix
        kf.processNoiseCov = cv::Mat::eye(stateSize, stateSize, type) * 1e-2;
        // Measurement Noise Covariance Matrix
        kf.measurementNoiseCov = cv::Mat::eye(measSize, measSize, type) * 1e-1;
        // Error Covariance Matrix
        kf.errorCovPost = cv::Mat::eye(stateSize, stateSize, type);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Kalman Filter Prediction
        cv::Mat prediction = kf.predict();
        float predictX = prediction.at<float>(0);
        float predictY = prediction.at<float>(1);

        // Assuming single point for simplicity. In practice, loop through detected objects.
        pcl::PointXYZ& point = cloud->points[0];
        cv::Mat measurement = (cv::Mat_<float>(2, 1) << point.x, point.y);

        // Kalman Filter Update
        kf.correct(measurement);

        // Use the predicted position as the filtered result
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ filtered_point;
        filtered_point.x = predictX;
        filtered_point.y = predictY;
        filtered_cloud->points.push_back(filtered_point);

        // Convert back to ROS message and publish
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.frame_id = msg->header.frame_id;
        publisher_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    cv::KalmanFilter kf;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}