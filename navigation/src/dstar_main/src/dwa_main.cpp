#include "dwa_main/dwa_main.h"
dwaNode::dwaNode() : rclcpp::Node("dwa_main"){

    //Initialize parameters
    string grid_topic;
    this->declare_parameter("grid_topic", "/map");
    if(!this->get_parameter("grid_topic", grid_topic)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter grid_topic");
    }

    string path_topic;
    this->declare_parameter("path_topic", "/path");
    if(!this->get_parameter("path_topic", path_topic)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter path_topic");
    }

    string cmd_vel_topic;
    this->declare_parameter("cmd_vel_topic", "/cmd_vel");
    if(!this->get_parameter("cmd_vel_topic", cmd_vel_topic)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter cmd_vel_topic");
    }

    this->declare_parameter("robot_frame", "base_link");
    if(!this->get_parameter("robot_frame", robot_frame)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter robot_frame");
    }

    this->declare_parameter("static_frame", "map");
    if(!this->get_parameter("static_frame", static_frame)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter static_frame");
    }
    //Get velocity params
    this->declare_parameter("max_axis_speed", 0.5);
    if(!this->get_parameter("max_axis_speed", max_axis_speed)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter max_axis_speed");
    }

    this->declare_parameter("min_axis_speed", 0.0);
    if(!this->get_parameter("min_axis_speed", min_axis_speed)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter min_axis_speed");
    }

    this->declare_parameter("axis_speed_resolution", 0.1);
    if(!this->get_parameter("axis_speed_resolution", axis_speed_resolution)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter axis_speed_resolution");
    }

    this->declare_parameter("min_omega_speed", -0.5);
    if(!this->get_parameter("min_omega_speed", min_omega_speed)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter min_omega_speed");
    }

    this->declare_parameter("max_omega_speed", 0.5);
    if(!this->get_parameter("max_omega_speed", max_omega_speed)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter max_omega_speed");
    }

    this->declare_parameter("omega_speed_resolution", 0.1);
    if(!this->get_parameter("omega_speed_resolution", omega_speed_resolution)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter omega_speed_resolution");
    }

    this->declare_parameter("dt", 0.005);
    if(!this->get_parameter("dt", dt)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter dt");
    }

    this->declare_parameter("sample_time", 0.1);
    if(!this->get_parameter("sample_time", sample_time)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter sample_time");
    }

    //Initialize costmap
    map_resolution = 0.05;
    map_width = 5000;
    map_height = 5000;
    map_origin_x = -map_width / 2 * map_resolution;
    map_origin_y = -map_height / 2 * map_resolution;
    costmap = new double*[map_width];
    for(int i = 0; i < map_width; i++){
        costmap[i] = new double[map_height];
        for(int j = 0; j < map_height; j++){
            costmap[i][j] = 0;
        }
    }
    
    //Initialize subscribers and publishers
    grid_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(grid_topic, 10, std::bind(&dwaNode::grid_callback, this, std::placeholders::_1));
    path_sub = this->create_subscription<nav_msgs::msg::Path>(path_topic, 10, std::bind(&dwaNode::path_callback, this, std::placeholders::_1));
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    
    //Initialize processing flag
    grid_received = path_received = main_process = false;
    
    //Initialize tf tools
    tfBuffer = new tf2_ros::Buffer(this->get_clock());
    tfListener = new tf2_ros::TransformListener(*tfBuffer);

    //Initialize the timer
    dwa_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&dwaNode::dwa_callback, this));

}
geometry_msgs::msg::TransformStamped dwaNode::get_transform(std::string target_frame, std::string source_frame){
    geometry_msgs::msg::TransformStamped transform;
    try{
        transform = tfBuffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    }
    catch(tf2::TransformException &ex){
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
    return transform;
}
geometry_msgs::msg::Point dwaNode::transformPoint(const geometry_msgs::msg::Point& point, const std::string& target_frame, const std::string& source_frame, geometry_msgs::msg::TransformStamped transformStamped) {
    geometry_msgs::msg::Point transformed_point;
    // 将输入点封装成 PointStamped 消息
    geometry_msgs::msg::PointStamped input_point;
    input_point.point = point;
    input_point.header.frame_id = source_frame;
    input_point.header.stamp = rclcpp::Clock().now();
    geometry_msgs::msg::PointStamped output_point_stamped;
    tf2::doTransform(input_point, output_point_stamped, transformStamped);
    return output_point_stamped.point;
}
void dwaNode::grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    grid_copy = *msg;
    grid_received = true;
}
void dwaNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg){
    path_copy = *msg;
    path_received = true;
}
void dwaNode::dwa_callback(){
    if(main_process == false){
        RCLCPP_INFO(this->get_logger(), "________________________________________\nStart processing");
        main_process = true;
    }
    else return;
    RCLCPP_INFO(this->get_logger(), "Main_Processing");
    if(grid_received){
        RCLCPP_INFO(this->get_logger(), "   Solving grid");
        solve_grid();
        RCLCPP_INFO(this->get_logger(), "   Solving grid complete");
    }
    if(path_received){
        RCLCPP_INFO(this->get_logger(), "   getting current position");
        Position robot_position = get_current_position();
        RCLCPP_INFO(this->get_logger(), "   getting current position: %f %f %f", robot_position.x, robot_position.y, robot_position.theta);
        RCLCPP_INFO(this->get_logger(), "   Solving path");
        Position goal = solve_path();
        RCLCPP_INFO(this->get_logger(), "   Solving path complete, goal_node pos: %f, %f", goal.x, goal.y);
        // if(abs(goal.x - robot_position.x)<=0.2&&abs(goal.y - robot_position.y)<=0.2){
        //     RCLCPP_INFO(this->get_logger(), "   Goal reached");
        //     geometry_msgs::msg::Twist cmd_vel;
        //     cmd_vel.linear.x = 0;
        //     cmd_vel.angular.z = 0;
        //     cmd_vel_pub->publish(cmd_vel);
        //     rclcpp::sleep_for(std::chrono::milliseconds(100));
        //     main_process = false;
        //     return;
        // }
        RCLCPP_INFO(this->get_logger(), "   Finding best velocity");
        Velocity best_velocity = find_best_velocity(robot_position, goal);
        RCLCPP_INFO(this->get_logger(), "   Finding best velocity complete, best_velocity: %f, %f", best_velocity.v, best_velocity.w);
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = best_velocity.v;
        cmd_vel.angular.z = best_velocity.w;
        cmd_vel_pub->publish(cmd_vel);
    }
    else{
        RCLCPP_WARN(this->get_logger(), "Waiting for path messages");
    }
        RCLCPP_INFO(this->get_logger(), "Finish processing");
    main_process = false;
}
dwaNode::Velocity dwaNode::find_best_velocity(Position robot_position, Position goal_position){
    double best_cost = 1e9;
    Velocity best_velocity;
    for(double v = min_axis_speed; v <= max_axis_speed; v += axis_speed_resolution){
        for(double w = min_omega_speed; w <= max_omega_speed; w += omega_speed_resolution){
            Velocity velocity;
            velocity.v = v;
            velocity.w = w;
            vector<Position> trajectory = get_trajectory(robot_position, velocity);
            double cost = evaluate_trajectory(trajectory, goal_position);
            RCLCPP_INFO(this->get_logger(), "       Velocity: %f, %f, Cost: %f", v, w, cost);
            if(cost < best_cost){
                best_cost = cost;
                best_velocity = velocity;
            }
        }
    }
    // {
    //     Velocity minus_velocity;
    //     minus_velocity.v = -min_axis_speed;
    //     minus_velocity.w = 0;
    //     vector<Position> trajectory = get_trajectory(robot_position, minus_velocity);
    //     double cost = 50;
    //     if(cost < best_cost){
    //         best_cost = cost;
    //         best_velocity = minus_velocity;
    //     }
    //     RCLCPP_INFO(this->get_logger(), "       Velocity: %f, %f, Cost: %f", -min_axis_speed, 0, cost);
    // }
    return best_velocity;
}
dwaNode::Position dwaNode::solve_path(){
    Position goal;
    geometry_msgs::msg::PoseStamped goal_pose = path_copy.poses[min(path_copy.poses.size() - 1, static_cast<std::size_t>(15))];
    geometry_msgs::msg::PoseStamped angle_vector;
    angle_vector.pose.position.x = path_copy.poses[1].pose.position.x - path_copy.poses[0].pose.position.x;
    angle_vector.pose.position.y = path_copy.poses[1].pose.position.y - path_copy.poses[0].pose.position.y;
    angle_vector.pose.position.z = 0;
    std::string path_frame = path_copy.header.frame_id;
    geometry_msgs::msg::TransformStamped transform = get_transform(static_frame, path_frame);
    geometry_msgs::msg::Point transformed_goal = transformPoint(goal_pose.pose.position, static_frame, path_frame, transform);
    geometry_msgs::msg::Point transformed_angle_vector = transformPoint(angle_vector.pose.position, static_frame, path_frame, transform);
    goal.x = transformed_goal.x;
    goal.y = transformed_goal.y;
    goal.theta = atan2(transformed_angle_vector.y, transformed_angle_vector.x);
    path_received = false;
    return goal;
}
void dwaNode::solve_grid(){
    double resolution = grid_copy.info.resolution;
    int width = grid_copy.info.width;
    int height = grid_copy.info.height;
    double origin_x = grid_copy.info.origin.position.x;
    double origin_y = grid_copy.info.origin.position.y;
    std::string map_frame = grid_copy.header.frame_id;
    geometry_msgs::msg::TransformStamped transform = get_transform(static_frame, map_frame);
    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
            int index = i + j * height;
            double real_x = origin_x + i * resolution;
            double real_y = origin_y + j * resolution;
            geometry_msgs::msg::Point point;
            point.x = real_x;
            point.y = real_y;
            point.z = 0;
            geometry_msgs::msg::Point transformed_point = transformPoint(point, static_frame, map_frame, transform);
            real_x = transformed_point.x;
            real_y = transformed_point.y;
            int x = (real_x - map_origin_x) / map_resolution;
            int y = (real_y - map_origin_y) / map_resolution;
            if(x < 0 || x >= map_width || y < 0 || y >= map_height){
                continue;
            }
            costmap[x][y] = grid_copy.data[index];
        }
    }
    grid_received = false;
}
dwaNode::Position dwaNode::get_current_position(){
    geometry_msgs::msg::TransformStamped transform = get_transform(static_frame, robot_frame);
    Position current_position;
    current_position.x = transform.transform.translation.x;
    current_position.y = transform.transform.translation.y;
    tf2::Quaternion quat;
    tf2::fromMsg(transform.transform.rotation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_position.theta = yaw;
    return current_position;
}

vector<dwaNode::Position> dwaNode::get_trajectory(Position robot_position, Velocity robot_velocity){
    vector<Position> trajectory;
    for(double t = 0; t < sample_time; t += dt){
        Position p = robot_position + robot_velocity * t;
        trajectory.push_back(p);
    }
    return trajectory;
}
double dwaNode::evaluate_trajectory(vector<Position> trajectory, Position goal){
    double cost = 0, final_cost = 0, min_dis = 114514;
    for(size_t i = 0; i < trajectory.size(); i++){
        Position p = trajectory[i];
        int x = (p.x-map_origin_x) / map_resolution, y = (p.y-map_origin_y) / map_resolution;    
        cost += costmap[x][y]==100?100:0;
        if(min_dis > sqrt(pow(p.x - goal.x, 2) + pow(p.y - goal.y, 2))){
            min_dis = sqrt(pow(p.x - goal.x, 2) + pow(p.y - goal.y, 2));
            final_cost = min_dis + cost;
        }
    }
    return final_cost;
}
dwaNode::~dwaNode(){
    delete tfBuffer;
    delete tfListener;
}
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dwaNode>());
    rclcpp::shutdown();
    return 0;
}