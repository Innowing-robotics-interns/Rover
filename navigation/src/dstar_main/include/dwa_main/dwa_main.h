// 在dstar_main.h的开始添加
#ifndef DSTAR_MAIN_H
#define DSTAR_MAIN_H
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "tf2/LinearMath/Quaternion.h"
#include <queue>
#include <vector>
#include <stack>
#include <algorithm>
#include <cmath>
#include <set>
#include <string.h>
#define INF 1e9
using namespace std;
class dwaNode : public rclcpp::Node{
    public:
        dwaNode();
        ~dwaNode();
        void grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
        void dwa_callback();
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
        rclcpp::TimerBase::SharedPtr dwa_timer;

        nav_msgs::msg::OccupancyGrid grid_copy;
        nav_msgs::msg::Path path_copy;
        bool grid_received, path_received, main_process;
        std::string grid_topic, cmd_vel_topic, robot_frame, static_frame;
        //map_parameters
        double **costmap;
        double map_resolution, map_origin_x, map_origin_y;
        int map_width, map_height;

        //tf tools
        tf2_ros::Buffer* tfBuffer;
        tf2_ros::TransformListener* tfListener;
        geometry_msgs::msg::TransformStamped get_transform(std::string target_frame, std::string source_frame);
        geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point& point, const std::string& target_frame, const std::string& source_frame, geometry_msgs::msg::TransformStamped transformstamped);

        //DWA parameters
        double max_axis_speed, min_axis_speed, axis_speed_resolution, min_omega_speed, max_omega_speed, omega_speed_resolution, dt, sample_time;
        //Position define
        struct Position{
            double x, y, theta;
            Position operator +(const Position& p){
                Position res;
                res.x = x + p.x * cos(theta) - p.y * sin(theta);
                res.y = y + p.x * sin(theta) + p.y * cos(theta);
                res.theta = theta + p.theta;
                return res;
            }
            // Position operator =(const Position& p){
            //     x = p.x;
            //     y = p.y;
            //     theta = p.theta;
            //     return *this;
            // }
        };
        struct Velocity{
            double v, w; // 线速度和角速度
            Position operator *(double t){
                Position p;
                double delta_theta = w * t; // 方向改变量
                p.theta = delta_theta; // 只设置方向改变量，假设初始方向为0
                // 计算位移
                p.x = v * t * cos(delta_theta / 2.0); // 假设初始方向为0
                p.y = v * t * sin(delta_theta / 2.0); // 假设初始方向为0
                return p;
            }
            // Velocity operator =(const Velocity& v){
            //     this->v = v.v;
            //     this->w = v.w;
            //     return *this;
            // }
        };

        //Get_position
        Position get_current_position();

        //Get_trajectory
        vector<Position> get_trajectory(Position robot_position, Velocity robot_velocity);
        double evaluate_trajectory(vector<Position> trajectory, Position goal_position);
        //solve_message
        void solve_grid();
        Position solve_path();
        Velocity find_best_velocity(Position robot_position, Position goal_position);
};
#endif // DSTAR_MAIN_H