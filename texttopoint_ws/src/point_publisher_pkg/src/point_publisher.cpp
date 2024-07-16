#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std;

class PointPublisher : public rclcpp::Node {
public:
    PointPublisher() : Node("point_publisher") {
        publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("clicked_point", 10);
        publishPoint();
    }

private:
    void publishPoint() {
        string place;
        cout << "Go to: ";
        cin >> place;

        geometry_msgs::msg::PointStamped pointStamped;
        pointStamped.header.stamp = this->get_clock()->now();
        pointStamped.header.frame_id = "map"; // Adjust the frame_id as needed

        if (place == "Desks") {
            pointStamped.point.x = 0.0;
            pointStamped.point.y = 0.0;
            pointStamped.point.z = 0.0;
        } else if (place == "LED_Screen") {
            pointStamped.point.x = 8.5;
            pointStamped.point.y = -2.3;
            pointStamped.point.z = 0.0;
        } else {
            cout << "Unknown place: " << place << endl;
            return;
        }

        cout << "Now moving to " << place << " (" << pointStamped.point.x << "," << pointStamped.point.y << ")" << endl;
        publisher_->publish(pointStamped);
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
