#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>
#include <iostream>

class DwaLocalPlanner : public rclcpp::Node
{   
public:
    DwaLocalPlanner() : Node("dwa_local_planner") , tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this(), false)) 
    {
        this->declare_parameter("max_vel_x", 0.5);
        this->declare_parameter("max_vel_theta", 0.5);
        this->declare_parameter("vx_samples", 10);
        this->declare_parameter("vtheta_samples", 20);
        this->declare_parameter("acc_lim_x", 1.0);
        this->declare_parameter("acc_lim_theta", 1.0);
        this->declare_parameter("path_topic", "/global_path");
        this->declare_parameter("odom_topic", "/odom");
        this->declare_parameter("cmd_vel_topic", "/cmd_vel");
        this->declare_parameter("goal_tolerance", 0.1);
        this->declare_parameter("local_costmap_frame", "local_costmap");
        this->declare_parameter("local_costmap_topic", "/local_costmap");
        this->declare_parameter("world_frame", "world");
        this->declare_parameter("safe_distance", 0.5);
        this->declare_parameter("heading_coefficient", 1.0);
        this->declare_parameter("distance_coefficient", 1.0);
        this->declare_parameter("velocity_coefficient", 1.0);
        this->declare_parameter("dt", 0.1);
        this->declare_parameter("steps", 10);
        this->declare_parameter("target_select_point", 5);
        

        // this->declare_parameter("scan_topic", "/scan");

        this->get_parameter("max_vel_x", max_vel_x_);
        this->get_parameter("max_vel_theta", max_vel_theta_);
        this->get_parameter("vx_samples", vx_samples_);
        this->get_parameter("vtheta_samples", vtheta_samples_);
        this->get_parameter("acc_lim_x", acc_lim_x_);
        this->get_parameter("acc_lim_theta", acc_lim_theta_);
        this->get_parameter("path_topic", path_topic_);
        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
        // this->get_parameter("scan_topic", scan_topic_);
        this->get_parameter("goal_tolerance", goal_tolerance_);
        this->get_parameter("local_costmap_frame", local_costmap_frame_);
        this->get_parameter("local_costmap_topic", local_costmap_topic_);
        this->get_parameter("world_frame", world_frame_);
        this->get_parameter("safe_distance", safe_distance_);
        this->get_parameter("heading_coefficient", alpha);
        this->get_parameter("distance_coefficient", beta);
        this->get_parameter("velocity_coefficient", gamma);
        this->get_parameter("dt", dt_);
        this->get_parameter("target_select_point", target_select_point);
        this->get_parameter("steps", steps_);

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic_, 10, std::bind(&DwaLocalPlanner::path_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&DwaLocalPlanner::odom_callback, this, std::placeholders::_1));

        local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            local_costmap_topic_, 10, std::bind(&DwaLocalPlanner::local_costmap_callback, this, std::placeholders::_1));


        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&DwaLocalPlanner::control_loop, this));
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        global_path_ = msg->poses;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        current_velocity_ = msg->twist.twist;
    }
    bool odom_available(){
        if(current_pose_.position.x == 0.0 
            && current_pose_.position.y == 0.0 
            && current_pose_.position.z == 0.0 
            && current_pose_.orientation.x == 0.0
            && current_pose_.orientation.y == 0.0
            && current_pose_.orientation.z == 0.0
            && current_pose_.orientation.w == 0.0){
            return false;
        }
        return true;
    }
    void local_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {   

        local_costmap_ = *msg;

            geometry_msgs::msg::TransformStamped transformStamped;
        try{
            transformStamped = tf_buffer_->lookupTransform(world_frame_, local_costmap_.header.frame_id, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to map: %s", local_costmap_.header.frame_id.c_str(), ex.what());
            return;
        }
    

        if(local_costmap_.data.empty()){
            return;
        }

        // Clear previous obstacles
        obstacles_.clear();

        // Extract obstacles from the costmap
        for (unsigned int y = 0; y < local_costmap_.info.height; y++) {
            for (unsigned int x = 0; x < local_costmap_.info.width; x++) {
                unsigned int index = x + y * local_costmap_.info.width;
                if (local_costmap_.data[index] > 50) { // Assuming 50 as the threshold for obstacles
                    double local_x = local_costmap_.info.origin.position.x + x * local_costmap_.info.resolution;
                    double local_y = local_costmap_.info.origin.position.y + y * local_costmap_.info.resolution;
                    
                    // Transform obstacle position from local_costmap frame to map frame
                    geometry_msgs::msg::PointStamped local_point;
                    local_point.header.frame_id = local_costmap_.header.frame_id;
                    local_point.point.x = local_x;
                    local_point.point.y = local_y;
                    geometry_msgs::msg::PointStamped map_point;
                    tf2::doTransform(local_point, map_point, transformStamped);

                    obstacles_.emplace_back(map_point.point.x, map_point.point.y);
                }
            }
        }
    }

    void control_loop()
    {
        if (global_path_.empty() ){
            RCLCPP_WARN(this->get_logger(), "Waiting for path");
            return;
        }

        if (!odom_available()){
            RCLCPP_WARN(this->get_logger(), "Waiting for odometry");
            return;
        }

        if(local_costmap_.data.empty()){
            RCLCPP_WARN(this->get_logger(), "Local costmap is empty");
            return;
        }

        auto target_point = get_target_point();
        if (!target_point){
            RCLCPP_WARN(this->get_logger(), "No target point");
            return;
        }

        auto best_trajectory = dwa_algorithm(target_point.value());
        if(best_trajectory.empty()){
            RCLCPP_WARN(this->get_logger(), "No best trajectory");
            return;
        }

        if (best_trajectory.size() > 0)
        {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = best_trajectory[0];
            cmd_vel.angular.z = -best_trajectory[1];
            RCLCPP_INFO(this->get_logger(), "Linear velocity: %f, Angular velocity: %f", cmd_vel.linear.x, cmd_vel.angular.z);
            cmd_vel_pub_->publish(cmd_vel);
        }
        return;
    }

    std::optional<geometry_msgs::msg::Pose> get_target_point()
    {
        // double min_distance = std::numeric_limits<double>::max();
        std::optional<geometry_msgs::msg::Pose> target_point;
        int count = 0;
        for (auto &pose : global_path_)
        {
            double distance = calculate_distance(current_pose_, pose.pose);
            RCLCPP_INFO(this->get_logger(), "Current pose: %f, %f", current_pose_.position.x, current_pose_.position.y);
            RCLCPP_INFO(this->get_logger(), "Target pose: %f, %f", pose.pose.position.x, pose.pose.position.y);
            RCLCPP_INFO(this->get_logger(), "Distance to target: %f", distance);
            if (count == target_select_point)
            {
                target_point = pose.pose;
                RCLCPP_INFO(this->get_logger(), "Current target point: %f, %f", target_point.value().position.x, target_point.value().position.y);
                return target_point;
            }
            count++;
        }
        // count<target_select_point
        RCLCPP_INFO(this->get_logger(), "Goal reached");
        return {};
    }

    double calculate_distance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2)
    {
        return std::sqrt(std::pow(pose1.position.x - pose2.position.x, 2) + std::pow(pose1.position.y - pose2.position.y, 2));
    }

    std::vector<double> dwa_algorithm(const geometry_msgs::msg::Pose &target_point)
    {
        // 1.Sampling
        std::vector<std::vector<double>> velocities;
        for (int i = 0; i < vx_samples_; ++i)   
        {
            double vx = i * max_vel_x_ / vx_samples_;
            for (int j = 0; j < vtheta_samples_; ++j)
            {
                double vtheta = -max_vel_theta_ + j * (2 * max_vel_theta_ / vtheta_samples_);
                velocities.push_back({vx, vtheta});
                RCLCPP_INFO(this->get_logger(), "Velocity: %f, %f", vx, vtheta);
            }
        }

        // 2. Evaluation
        double best_cost = 100000000.0;
        std::vector<double> best_trajectory;
        for (const auto &velocity : velocities)
        {
            double vx = velocity[0];
            double vtheta = velocity[1];

            // Generate trajectory
            auto trajectory = generate_trajectory(vx, vtheta);
            if (trajectory.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Empty trajectory");
                return {};
            }

            // Evaluate trajectory
            double cost = evaluate_trajectory(trajectory, target_point);
            if (cost < best_cost)
            {
                best_cost = cost;
                best_trajectory = velocity;
                RCLCPP_INFO(this->get_logger(), "Best cost: %f", best_cost);
                RCLCPP_INFO(this->get_logger(), "Best velocity: %f, %f", best_trajectory[0], best_trajectory[1]);
            }
        }

        return best_trajectory;
    }

    std::vector<std::pair<double, double>> generate_trajectory(double vx, double vtheta)
    {
        std::vector<std::pair<double, double>> trajectory;
        double dt = dt_;
        double x = current_pose_.position.x;
        double y = current_pose_.position.y;
        double theta = tf2::getYaw(current_pose_.orientation);
        for (int i = 0; i < steps_; ++i)
        {
            x += vx * std::cos(theta) * dt;
            y += vx * std::sin(theta) * dt;
            theta += vtheta * dt;
            trajectory.push_back({x, y});
        }
        return trajectory;
    }

    double evaluate_trajectory(const std::vector<std::pair<double, double>> &trajectory, const geometry_msgs::msg::Pose &target_point)
    {
        double cost = 0.0;
        double heading_eval = 0.0;
        double dist_eval = 0.0;
        double vel_eval = 0.0;

        // Orientation evaluation
        double goal_x = target_point.position.x;
        double goal_y = target_point.position.y;
        double dx = goal_x - trajectory.back().first;
        double dy = goal_y - trajectory.back().second;
        double error_angle = atan2(dy, dx);
        double cost_angle = error_angle - atan2(trajectory.back().second - trajectory.front().second, trajectory.back().first - trajectory.front().first);
        heading_eval = M_PI - fabs(cost_angle);

        // Distance evaluation
        double min_distance = std::numeric_limits<double>::max();
        for (const auto &point : trajectory)
        {
            for (const auto &obs : obstacles_)
            {
                double dist = std::sqrt(std::pow(point.first - obs.first, 2) + std::pow(point.second - obs.second, 2));
                if (dist < min_distance)
                {
                    min_distance = dist;
                }
            }
        }
        if (min_distance < safe_distance_) 
        {
            dist_eval = 100000.0; // ~ abandon this trajectory
        }
        else
        {
            dist_eval = min_distance;
        }

        // Velocity evaluation
        double vx = (trajectory.back().first - trajectory.front().first) / (trajectory.size() * dt_); // dt_ is the time step
        vel_eval = vx;

        // Overall evaluation
        cost = alpha * heading_eval + beta * dist_eval  + gamma * vel_eval ;

        return cost;
    }

    // bool is_free_space(double x, double y)
    // {
    //     unsigned int mx, my;
    //     costmap_ros_->getCostmap()->worldToMap(x, y, mx, my);
    //     unsigned char cost = costmap_ros_->getCostmap()->getCost(mx, my);
    //     return cost == costmap_2d::FREE_SPACE;
    // }

    double max_vel_x_;
    double max_vel_theta_;
    int vx_samples_;
    int vtheta_samples_;
    double acc_lim_x_;
    double acc_lim_theta_;
    std::string path_topic_;
    std::string odom_topic_;
    std::string cmd_vel_topic_;
    // std::string scan_topic_;
    double goal_tolerance_;
    std::string local_costmap_frame_;
    std::string local_costmap_topic_;
    std::string world_frame_;
    double safe_distance_;
    double alpha, beta, gamma;
    double dt_;
    int target_select_point;
    int steps_;

    std::vector<geometry_msgs::msg::PoseStamped> global_path_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist current_velocity_;
    nav_msgs::msg::OccupancyGrid local_costmap_;
    std::vector<std::pair<double, double>> obstacles_;
    sensor_msgs::msg::LaserScan scan_data_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    };

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DwaLocalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
