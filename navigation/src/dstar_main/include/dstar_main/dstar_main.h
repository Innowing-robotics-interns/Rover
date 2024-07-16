// 在dstar_main.h的开始添加
#ifndef DSTAR_MAIN_H
#define DSTAR_MAIN_H
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
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
#include <queue>
#include <vector>
#include <stack>
#include <algorithm>
#include <cmath>
#include <set>
#include <string.h>
#define INF 1e9
#define NEW 0
#define IN_LIST 1
#define OUT_LIST 2
class dstarNode : public rclcpp::Node{
    public:
        dstarNode();
        ~dstarNode();
        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
        void esdf_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void dstar_callback();
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_sub;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr status_pub;
        rclcpp::TimerBase::SharedPtr dstar_timer;
        class Node;
        typedef Node* Nodeptr;
        class LinkCutTree;
        typedef LinkCutTree* LinkCutTreeptr;
        //Node class
        class Node{
            public:
                double dis_to_goal, rhs;
                double dis_to_obstacle, edge_value;
                double obstacle_possibility, static_obstacle_possibility;
                bool reversed;
                int x, y, dstar_list_status;
                Nodeptr succ, father, son[2];
                Node(int x, int y){
                    dis_to_goal = INF + 1;
                    rhs = INF;
                    this->obstacle_possibility = this->static_obstacle_possibility = 0;
                    this->x = x;
                    this->y = y;
                    dstar_list_status = NEW;
                    succ = father = son[0] = son[1] = nullptr;
                }
                Node(){}
                void clear(){
                    dis_to_goal = INF + 1;
                    rhs = INF;
                    succ = father = son[0] = son[1] = nullptr;
                    reversed = false;
                    dstar_list_status = NEW;
                }
        };
        
        //planning map information
        int global_width, global_height;
        double global_resolution;
        double global_ori_x, global_ori_y;
        Nodeptr **node_map;
        Nodeptr goal_node, start_node;
        
        //params defination
        std::string node_name = "dstar_node";
        bool esdf_or_grid_map;
        std::string read_in_topic;
        std::string path_topic;
        std::string goal_topic;
        std::string robot_frame;
        std::string map_frame;

        //subscriber record
        bool get_grid_msg, get_esdf_msg, get_goal_msg;
        bool get_new_goal;
        nav_msgs::msg::OccupancyGrid grid_msg_copy;
        sensor_msgs::msg::PointCloud2 esdf_msg_copy;
        geometry_msgs::msg::PointStamped goal_msg_copy;

        //process flag
        bool main_process_flag;

        //deal with map
        void solve_grid_msg();
        void solve_esdf_msg();
        void solve_goal_msg();
        void solve_current_pose();

        //tf information
        tf2_ros::Buffer* tfBuffer;
        tf2_ros::TransformListener* tfListener;
        geometry_msgs::msg::TransformStamped get_transform(std::string target_frame, std::string source_frame);
        geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point& point, const std::string& target_frame, const std::string& source_frame, geometry_msgs::msg::TransformStamped transformstamped);

        //dstar functions
        void update_node(Nodeptr cur);
        double calculate_edge_value(Nodeptr cur);
        void dstar_update();
        class Compare_in_Dstar{
            public:
                bool operator()(const Nodeptr a, const Nodeptr b) const {
                    if(std::min(a->dis_to_goal, a->rhs) == std::min(b->dis_to_goal, b->rhs)){
                        if(a -> x == b -> x){
                            return a -> y < b -> y;
                        }
                        return a -> x < b -> x;
                    }
                    return std::min(a->dis_to_goal, a->rhs) < std::min(b->dis_to_goal, b->rhs);
                }
        };
        void clear();
        void publish_path();
        void publish_map_status();

        //dstar params
        std::set<Nodeptr, Compare_in_Dstar> dstar_list;
        LinkCutTreeptr lct;
        double x0 = 75;
        double k = 0.25;
        double L = 10; 
        int dx[8]={-1, 0, 1, 0, -1, 1, 1, -1};
        int dy[8]={0, 1, 0, -1, 1, 1, -1, -1};

};
#endif // DSTAR_MAIN_H