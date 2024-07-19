#include "dstar_main/dstar_main.h"
#include "dstar_main/dstar_lct.hpp"
dstarNode::dstarNode() : rclcpp::Node("dstar_main"){
    this->declare_parameter<bool>("esdf_or_grid_map", false);
    if(!this->get_parameter("esdf_or_grid_map", esdf_or_grid_map)){
        RCLCPP_ERROR(this->get_logger(), "esdf_or_grid_map not set");
        exit(1);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "esdf_or_grid_map: %d", esdf_or_grid_map);
    }
    this->declare_parameter<std::string>("global_read_in_topic", "esdf");
    if(!this->get_parameter("global_read_in_topic", global_read_in_topic)){
        RCLCPP_ERROR(this->get_logger(), "global_read_in_topic not set");
        exit(1);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "global_read_in_topic: %s", global_read_in_topic.c_str());
    }
    this->declare_parameter<std::string>("local_read_in_topic", "esdf");
    if(!this->get_parameter("local_read_in_topic", local_read_in_topic)){
        RCLCPP_ERROR(this->get_logger(), "local_read_in_topic not set");
        exit(1);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "local_read_in_topic: %s", local_read_in_topic.c_str());
    }
    this->declare_parameter<std::string>("path_topic", "path");
    if(!this->get_parameter("path_topic", path_topic)){
        RCLCPP_ERROR(this->get_logger(), "path_topic not set");
        exit(1);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "path_topic: %s", path_topic.c_str());
    }
    this->declare_parameter<std::string>("goal_topic", "goal");
    if(!this->get_parameter("goal_topic", goal_topic)){
        RCLCPP_ERROR(this->get_logger(), "goal_topic not set");
        exit(1);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "goal_topic: %s", goal_topic.c_str());
    }
    this->declare_parameter<std::string>("robot_frame", "base_link");
    if(!this->get_parameter("robot_frame", robot_frame)){
        RCLCPP_ERROR(this->get_logger(), "robot_frame not set");
        exit(1);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "robot_frame: %s", robot_frame.c_str());
    }
    this->declare_parameter<std::string>("map_frame", "map");
    if(!this->get_parameter("map_frame", map_frame)){
        RCLCPP_ERROR(this->get_logger(), "map_frame not set");
        exit(1);
    }
    else{
        RCLCPP_INFO(this->get_logger(), "map_frame: %s", map_frame.c_str());
    }
    // Create a publisher for the path
    path_pub = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
    // Create a publisher for the status
    status_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("status", 10);
    // Create a subscriber for the map
    if(!esdf_or_grid_map)
        esdf_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(global_read_in_topic, 10, std::bind(&dstarNode::esdf_callback, this, std::placeholders::_1));
    else 
        grid_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(global_read_in_topic, 10, std::bind(&dstarNode::map_callback, this, std::placeholders::_1));
    local_grid_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(local_read_in_topic, 10, std::bind(&dstarNode::map_callback, this, std::placeholders::_1));
    // Create a subscriber for the goal
    goal_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(goal_topic, 10, std::bind(&dstarNode::goal_callback, this, std::placeholders::_1));
    // Create a timer for the main process
    dstar_timer = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&dstarNode::dstar_callback, this));
    // Initialize the tfBuffer
    tfBuffer = new tf2_ros::Buffer(this->get_clock());
    tfListener = new tf2_ros::TransformListener(*tfBuffer);
    //Initialize the process flag
    main_process_flag = get_esdf_msg = get_grid_msg = false;
    //Initialize the map information
    global_width = global_height = 5000;
    global_resolution = 0.05;
    global_ori_x = global_ori_y = -global_height * global_resolution / 2;
    node_map = new Nodeptr*[global_width];
    for(int i = 0; i < global_width; i++){
        node_map[i] = new Nodeptr[global_height];
        for(int j = 0; j < global_height; j++){
            node_map[i][j] = new Node(i, j);
        }
    }
    lct = new LinkCutTree();
    //Set edge value params
    x0 = 75;
    k = 0.25;
    L = 5; 
}
void dstarNode::dstar_update(){
    RCLCPP_INFO(this->get_logger(), "dstar_update");
    int num=0;
    while(!dstar_list.empty()){
        if(lct->find_root(goal_node) == lct->find_root(start_node))break;
        // RCLCPP_INFO(this->get_logger(), "dis_to_goal: %f, rhs: %f, x: %f, y: %f", (*dstar_list.begin())->dis_to_goal, (*dstar_list.begin())->rhs, (*dstar_list.begin())->x * global_resolution + global_ori_x, (*dstar_list.begin())->y * global_resolution + global_ori_y);
        // rclcpp::sleep_for(std::chrono::milliseconds(1000));
        
        Nodeptr cur = *dstar_list.begin();
        // num++;
        // if(num>1000){
        //     RCLCPP_INFO(this->get_logger(), "dis_to_goal: %f, rhs: %f, x: %f, y: %f, lenth %d", cur->dis_to_goal, cur->rhs, cur->x * global_resolution + global_ori_x, cur->y * global_resolution + global_ori_y,dstar_list.size());
        //     // rclcpp::sleep_for(std::chrono::milliseconds(1000));
        // }
        // auto it = dstar_list.begin();
        // it++;
        // test = *it;
        // RCLCPP_INFO(this->get_logger(), "dis_to_goal_next: %f, rhs: %f", test->dis_to_goal, test->rhs);
        dstar_list.erase(cur);
        cur->dstar_list_status = OUT_LIST;
        if(cur->dis_to_goal < cur -> rhs){
            int x = cur->x, y = cur->y;
            cur -> dis_to_goal = INF + 1;
            cur -> dstar_list_status = IN_LIST;
            dstar_list.insert(cur);
            for(int i = 0; i < 8; i++){
                int nx = x + dx[i];
                int ny = y + dy[i];
                if(nx < 0 || nx >= global_width || ny < 0 || ny >= global_height)continue;
                update_node(node_map[nx][ny]);
            }
        }
        else if(cur -> dis_to_goal > cur -> rhs){
            int x = cur->x, y = cur->y;
            cur -> dis_to_goal = cur -> rhs;
            for(int i = 0; i < 8; i++){
                int nx = x + dx[i];
                int ny = y + dy[i];
                if(nx < 0 || nx >= global_width || ny < 0 || ny >= global_height)continue;
                update_node(node_map[nx][ny]);
            }
            if(cur->succ != nullptr)lct->link(cur, cur->succ);
        }
    }
    RCLCPP_INFO(this->get_logger(), "dstar_update end");
}
dstarNode::~dstarNode(){
    delete tfBuffer;
    delete tfListener;
    delete lct;
    for(int i = 0; i < global_width; i++)for(int j = 0; j < global_height; j++)delete node_map[i][j];
    for(int i = 0; i < global_width; i++)delete[] node_map[i];
    delete[] node_map;
}
void dstarNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "solve_grid_msg");
    int width = msg->info.width;
    int height = msg->info.height;
    double resolution = msg->info.resolution;
    double ori_x =msg->info.origin.position.x;
    double ori_y =msg->info.origin.position.y;
    std::string grid_frame = msg->header.frame_id;
    RCLCPP_INFO(this->get_logger(), "grid_frame: %s resolution: %f, ori_x: %f, ori_y: %f, width: %d, height: %d", grid_frame.c_str(), resolution, ori_x, ori_y, width, height);
    //get tf from grid_frame to map_frame
    geometry_msgs::msg::TransformStamped transform = get_transform(map_frame, grid_frame);
    for(int i = 0 ; i < width; i++){
        for(int j = 0; j < height; j++){
            //get coordinate in the grid_frame
            double real_x = ori_x + i * resolution;
            double real_y = ori_y + j * resolution;
            // RCLCPP_INFO(this->get_logger(), "real_x: %f, real_y: %f", real_x, real_y);
            geometry_msgs::msg::Point point;
            point.x = real_x;
            point.y = real_y;
            point.z = 0;
            //transform the point into the map_frame
            geometry_msgs::msg::Point transformed_point = transformPoint(point, map_frame, grid_frame, transform);
            // RCLCPP_INFO(this->get_logger(), "transformed_point: %f, %f", transformed_point.x, transformed_point.y);
            int x = round((transformed_point.x - global_ori_x) / global_resolution);
            int y = round((transformed_point.y - global_ori_y) / global_resolution);
            if(x < 0 || x >= global_width || y < 0 || y >= global_height){
                // RCLCPP_ERROR(this->get_logger(), "x: %f, y: %f", transformed_point.x, transformed_point.y);
                continue;
            }
            if(msg->data[i + j * width] != node_map[x][y]->obstacle_possibility && msg->data[i + j * width] != -1){
                node_map[x][y]->obstacle_possibility = msg->data[i + j * width];
                update_node(node_map[x][y]);
            }
        }
    }
    // rclcpp::sleep_for(std::chrono::milliseconds(10000));
    RCLCPP_INFO(this->get_logger(), "solve_grid_msg end");
    get_grid_msg = true;
}
void dstarNode::goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
    goal_msg_copy = *msg;
    get_goal_msg = true;
}
void dstarNode::esdf_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    esdf_msg_copy = *msg;
    get_esdf_msg = true;
}
void dstarNode::clear(){
    dstar_list.clear();
    for(int i = 0; i < global_width; i++){
        for(int j = 0; j < global_height; j++){
            node_map[i][j]->clear();
        }
    }
}
void dstarNode::solve_goal_msg(){
    RCLCPP_INFO(this->get_logger(), "solve_goal_msg");
    geometry_msgs::msg::TransformStamped transform = get_transform(map_frame, goal_msg_copy.header.frame_id);
    geometry_msgs::msg::Point point;
    point.x = goal_msg_copy.point.x;
    point.y = goal_msg_copy.point.y;
    point.z = goal_msg_copy.point.z;
    geometry_msgs::msg::Point transformed_point = transformPoint(point, map_frame, goal_msg_copy.header.frame_id, transform);
    // if(transformed_point.x - point.x > 0.1){
    //     RCLCPP_ERROR(this->get_logger(), "transform error");
    //     rclcpp::sleep_for(std::chrono::milliseconds(1000));
    // }
    int x = round((transformed_point.x - global_ori_x) / global_resolution);
    int y = round((transformed_point.y - global_ori_y) / global_resolution);
    if(x < 0 || x >= global_width || y < 0 || y >= global_height || (goal_node != nullptr && x == goal_node -> x && y == goal_node -> y))return;
    get_new_goal = true;
    clear();
    goal_node = node_map[x][y];
    RCLCPP_INFO(this->get_logger(), "goal_node: %f, %f, %d, %d", transformed_point.x, transformed_point.y, x, y);
    // rclcpp::sleep_for(std::chrono::milliseconds(10000));
    goal_node->dis_to_goal = INF + 1;
    goal_node->rhs = 0;
    goal_node->succ = nullptr;
    goal_node->dstar_list_status = IN_LIST;
    dstar_list.insert(goal_node);
    RCLCPP_INFO(this->get_logger(), "solve_goal_msg end");
}
void dstarNode::solve_grid_msg(){
}
void dstarNode::solve_esdf_msg(){
    RCLCPP_INFO(this->get_logger(), "solve_esdf_msg");
    sensor_msgs::msg::PointCloud2 transformed_msg;
    geometry_msgs::msg::TransformStamped transform = get_transform(map_frame, esdf_msg_copy.header.frame_id);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    // 将ROS消息转换为PCL点云
    pcl::fromROSMsg(esdf_msg_copy, cloud);
    for(const auto& point_ori : cloud){
        double x = point_ori.x;
        double y = point_ori.y;
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = 0;
        geometry_msgs::msg::Point transformed_point = transformPoint(point, map_frame, esdf_msg_copy.header.frame_id, transform);
        x = transformed_point.x;
        y = transformed_point.y;
        double intensity = point_ori.intensity; // 获取intensity值
        int x_index = round((x - global_ori_x) / global_resolution);
        int y_index = round((y - global_ori_y) / global_resolution);
        if(x_index < 0 || x_index >= global_width || y_index < 0 || y_index >= global_height) continue;
        if(node_map[x_index][y_index]->obstacle_possibility != intensity){
            node_map[x_index][y_index]->obstacle_possibility = intensity;
            update_node(node_map[x_index][y_index]);
        }
    }
    RCLCPP_INFO(this->get_logger(), "solve_esdf_msg end");
}
void dstarNode::solve_current_pose(){
    RCLCPP_INFO(this->get_logger(), "solve_current_pose");
    geometry_msgs::msg::TransformStamped transform = get_transform(map_frame, robot_frame);
    geometry_msgs::msg::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    geometry_msgs::msg::Point transformed_point = transformPoint(point, map_frame, robot_frame, transform);
    int x = round((transformed_point.x - global_ori_x) / global_resolution);
    int y = round((transformed_point.y - global_ori_y) / global_resolution);
    if(x < 0 || x >= global_width || y < 0 || y >= global_height)return;
    start_node = node_map[x][y];
    RCLCPP_INFO(this->get_logger(), "solve_current_pose end");
}
geometry_msgs::msg::TransformStamped dstarNode::get_transform(std::string target_frame, std::string source_frame){
    geometry_msgs::msg::TransformStamped transform;
    try{
        transform = tfBuffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    }
    catch(tf2::TransformException &ex){
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
    return transform;
}
geometry_msgs::msg::Point dstarNode::transformPoint(const geometry_msgs::msg::Point& point, const std::string& target_frame, const std::string& source_frame, geometry_msgs::msg::TransformStamped transformStamped) {
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
void dstarNode::dstar_callback(){
    if(main_process_flag)return;
    main_process_flag = true;
    RCLCPP_INFO(this->get_logger(), "dstar_callback");
    if(get_goal_msg){
        solve_goal_msg();
        get_goal_msg = false;
    }
    if(get_new_goal){
        if(get_esdf_msg){
            solve_esdf_msg();
            get_esdf_msg = false;
        }
        solve_current_pose();
        dstar_update();
        RCLCPP_INFO(this->get_logger(), "publish start");
        publish_path();
        // publish_map_status();
        RCLCPP_INFO(this->get_logger(), "publish end");
        if(abs(goal_node->x - start_node->x) + abs(goal_node->y - start_node->y) < 10){
            RCLCPP_INFO(this->get_logger(), "goal reached");
            get_new_goal = false;
            goal_node = nullptr;
        }
        // rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
    main_process_flag = false;
}
void dstarNode::update_node(Nodeptr cur){
    if(cur == goal_node)return;
    if(cur->dstar_list_status == IN_LIST){
        dstar_list.erase(cur);
        cur->dstar_list_status = OUT_LIST;
    }
    Nodeptr previous_succ = cur->succ;
    double previous_rhs = cur->rhs;
    cur->rhs = INF;
    cur->succ = nullptr;
    for(int i = 0; i < 8; i++){
        int nx = cur->x + dx[i];
        int ny = cur->y + dy[i];
        if(nx < 0 || nx >= global_width || ny < 0 || ny >= global_height)continue;
        double value = calculate_edge_value(cur)*calculate_edge_value(node_map[nx][ny]);
        // if(value - 1 > 0.1)RCLCPP_INFO(this->get_logger(), "value: %f", value);
        double new_rhs = i<4?node_map[nx][ny]->dis_to_goal + value : node_map[nx][ny]->dis_to_goal + 1.414 * value;
        if(new_rhs < cur->rhs){
            cur->rhs = new_rhs;
            cur->succ = node_map[nx][ny];
        }
    }
    if(previous_rhs == cur -> dis_to_goal && previous_succ != nullptr)lct->del(cur, previous_succ);
    if(cur->rhs == cur->dis_to_goal && cur->succ != nullptr)lct->link(cur, cur->succ);
    if(cur -> rhs != cur->dis_to_goal){    
        cur->dstar_list_status = IN_LIST;
        dstar_list.insert(cur);
    }
}
double dstarNode::calculate_edge_value(Nodeptr cur){
    return 1 + L/(1 + exp(-k * (cur->obstacle_possibility - x0)));
}
void dstarNode::publish_path(){
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame;
    path.header.stamp = rclcpp::Clock().now();
    Nodeptr cur = start_node;
    while(cur != nullptr){
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = cur->x * global_resolution + global_ori_x;
        pose.pose.position.y = cur->y * global_resolution + global_ori_y;
        pose.pose.position.z = 0;
        path.poses.push_back(pose);
        cur = cur->succ;
    }
    path_pub->publish(path);

}
void dstarNode::publish_map_status(){
    nav_msgs::msg::OccupancyGrid status;
    status.header.frame_id = map_frame;
    status.header.stamp = rclcpp::Clock().now();
    status.info.width = global_width;
    status.info.height = global_height;
    status.info.resolution = global_resolution;
    status.info.origin.position.x = global_ori_x;
    status.info.origin.position.y = global_ori_y;
    status.info.origin.position.z = 0;
    status.info.origin.orientation.x = 0;
    status.info.origin.orientation.y = 0;
    status.info.origin.orientation.z = 0;
    status.info.origin.orientation.w = 1;
    for(int i = 0; i < global_height; i++){
        for(int j = 0; j < global_width; j++){
            status.data.push_back((int)node_map[j][i]->obstacle_possibility);
        }
    }
    status_pub->publish(status);

}
int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dstarNode>();
    RCLCPP_INFO(node->get_logger(), "dstar_node start");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}