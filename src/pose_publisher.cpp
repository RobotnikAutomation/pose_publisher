#include "rclcpp/rclcpp.hpp"

class PosePubNode : public rclcpp::Node
{
public:
    PosePubNode() : Node("node_pose_publisher") {
        RCLCPP_INFO(get_logger(),  "Connection established");
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePubNode>());
    rclcpp::shutdown();
    return 0;
}
