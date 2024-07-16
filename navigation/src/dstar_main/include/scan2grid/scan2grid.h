#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <string>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using namespace std;
class scan2grid : public rclcpp::Node {
    public:
        scan2grid();
        ~scan2grid();
        void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        
        //tf tools
        tf2_ros::Buffer* tfBuffer;
        tf2_ros::TransformListener* tfListener;
        geometry_msgs::msg::TransformStamped get_transform(std::string target_frame, std::string source_frame);
        geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point& point, const std::string& target_frame, const std::string& source_frame, geometry_msgs::msg::TransformStamped transformstamped);
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub;
        std::string goal_frame;
        double map_size, map_resolution;
};