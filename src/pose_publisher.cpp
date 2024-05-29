#include <chrono>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PosePubNode : public rclcpp::Node
{
public:
    PosePubNode() : Node("node_pose_publisher") {
        RCLCPP_INFO(get_logger(),  "Connection established");

        // std::chrono::seconds timer_frequency(1);
        std::chrono::milliseconds timer_frequency{500};

        // Call timer_callback function every timer_frequency
        timer_ = this->create_wall_timer(
            timer_frequency, std::bind(&PosePubNode::timer_callback, this));
    }

private:
    void timer_callback() {
        RCLCPP_INFO(get_logger(),  "In timer_callback");
    }

    /* Here it is created the shared pointers to the Publisher and timer objects
    * defined above, and it is also created the variable count*/
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePubNode>());
    rclcpp::shutdown();
    return 0;
}
