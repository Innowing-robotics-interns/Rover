#include "map2grid/map2grid.h"

geometry_msgs::msg::TransformStamped map2grid::get_transform(string target_frame, string source_frame){
    geometry_msgs::msg::TransformStamped transform;
    try{
        transform = tfBuffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    }
    catch(tf2::TransformException &ex){
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
    return transform;
}
geometry_msgs::msg::Point map2grid::transformPoint(const geometry_msgs::msg::Point& point, const string& target_frame, const string& source_frame, geometry_msgs::msg::TransformStamped transformStamped) {
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

map2grid::map2grid():rclcpp::Node("map2grid"){
    string map_topic;
    string publish_topic;
    this->declare_parameter("goal_frame", "base_link");
    this->declare_parameter("map_size", 5.0);
    this->declare_parameter("map_resolution", 0.05);
    this->declare_parameter("map_topic", "map");
    this->declare_parameter("publish_topic", "global_costmap");
    if(!this->get_parameter("map_topic", map_topic)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter map_topic");
    }
    if(!this->get_parameter("goal_frame", goal_frame)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter goal_frame");
    }
    if(!this->get_parameter("map_size", map_size)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter map_size");
    }
    if(!this->get_parameter("map_resolution", map_resolution)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter map_resolution");
    }
    if(!this->get_parameter("publish_topic", publish_topic)){
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter publish_topic");
    }
    tfBuffer = new tf2_ros::Buffer(this->get_clock());
    tfListener = new tf2_ros::TransformListener(*tfBuffer);
    map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, 10, bind(&map2grid::map_callback, this, placeholders::_1));
    occupancy_grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(publish_topic, 10);
}

map2grid::~map2grid(){
    delete tfBuffer;
    delete tfListener;
}

void map2grid::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header = msg->header;
    occupancy_grid.header.frame_id = goal_frame;
    occupancy_grid.info.resolution = map_resolution;
    occupancy_grid.info.width = map_size / map_resolution;
    occupancy_grid.info.height = map_size / map_resolution;
    occupancy_grid.info.origin.position.x = -map_size / 2;
    occupancy_grid.info.origin.position.y = -map_size / 2;
    occupancy_grid.info.origin.position.z = 0;
    occupancy_grid.info.origin.orientation.x = 0;
    occupancy_grid.info.origin.orientation.y = 0;
    occupancy_grid.info.origin.orientation.z = 0;
    occupancy_grid.info.origin.orientation.w = 1;
    occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height);
    int width = msg->info.width;
    int height = msg->info.height;
    double resolution = msg->info.resolution;
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;
    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
            if(msg->data[j * width + i] < 100)continue;
            geometry_msgs::msg::Point point;
            point.x = i * resolution + origin_x;
            point.y = j * resolution + origin_y;
            point.z = 0;
            geometry_msgs::msg::TransformStamped transformStamped = get_transform(goal_frame, msg->header.frame_id);
            geometry_msgs::msg::Point transformed_point = transformPoint(point, goal_frame, msg->header.frame_id, transformStamped);
            int x = (transformed_point.x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution;
            int y = (transformed_point.y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution;
            if(x >= 0 && x < occupancy_grid.info.width && y >= 0 && y < occupancy_grid.info.height){
                occupancy_grid.data[y * occupancy_grid.info.width + x] = 100;
            }
        }
    }
    for(int i = 0; i < occupancy_grid.info.width; i++){
        for(int j = 0; j < occupancy_grid.info.height; j++){
            if(occupancy_grid.data[j * occupancy_grid.info.width + i] == 100){
                for(int m = -20; m <= 20; m++){
                    for(int n = -20; n <= 20; n++){
                        if(i + m < 0 || i + m >= occupancy_grid.info.width || j + n < 0 || j + n >= occupancy_grid.info.height){
                            continue;
                        }
                        int new_x = i + m;
                        int new_y = j + n;
                        occupancy_grid.data[new_y * occupancy_grid.info.width + new_x] = std::max(100 - sqrt(m * m + n * n) * 5, static_cast<double>(occupancy_grid.data[new_y * occupancy_grid.info.width + new_x]));
                    }
                }
            }
        }
    }
    occupancy_grid_pub->publish(occupancy_grid);
}
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<map2grid>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}