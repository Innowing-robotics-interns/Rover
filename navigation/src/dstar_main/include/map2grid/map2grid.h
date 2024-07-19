#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <string>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
using namespace std;

class map2grid : public rclcpp::Node {
    public:
        map2grid();
        ~map2grid();
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        
        //tf tools
        tf2_ros::Buffer* tfBuffer;
        tf2_ros::TransformListener* tfListener;
        geometry_msgs::msg::TransformStamped get_transform(string target_frame, string source_frame);
        geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point& point, const string& target_frame, const string& source_frame, geometry_msgs::msg::TransformStamped transformstamped);
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub;
        string goal_frame;
        double map_size, map_resolution;
};