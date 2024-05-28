#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("node_pose_publisher");

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
    }
    
    rclcpp::shutdown();
    return 0;
}
