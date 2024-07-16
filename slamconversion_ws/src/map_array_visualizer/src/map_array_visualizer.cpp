#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/OccupancyGrid.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

class MapArrayVisualizer : public rclcpp::Node {
public:
    MapArrayVisualizer() : Node("map_array_visualizer") {
        // Generic subscription to the MapArray topic
        map_array_subscriber_ = this->create_generic_subscription(
            "/map_array", 10, std::bind(&MapArrayVisualizer::mapArrayCallback, this, std::placeholders::_1));

        // Publisher for the Map (OccupancyGrid)
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/visualized_map", 10);
    }

private:
    void mapArrayCallback(const std::shared_ptr<rclcpp::SerializedMessage> msg) {
        // Note: You cannot directly access msg fields without deserializing it.
        // Deserialization requires knowing the message type, which we're avoiding here.
        // This is just a placeholder to show where you'd handle the message.

        // Example: Log receiving a message (actual handling would require deserialization)
        RCLCPP_INFO(this->get_logger(), "Received a map array message");
    }

    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr map_array_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
};