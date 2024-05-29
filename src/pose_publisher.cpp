#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std::chrono_literals;

class PosePubNode : public rclcpp::Node
{
public:
    PosePubNode() : Node("node_pose_publisher")
    {
        RCLCPP_INFO(get_logger(),  "Connection established");

        // Read parameters from the parameter server
        RCLCPP_DEBUG(get_logger(), "Reading parameters from server");
        read_parameters();

        // Create pose_publisher publisher
        pose_publisher_ =
            this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_republish, 1);        
        RCLCPP_INFO(get_logger(),  "Pose_publisher created with topic: %s", topic_republish.c_str());
        
        // std::chrono::seconds timer_period(1);
        std::chrono::milliseconds timer_period{500};

        // Call timer_callback function every timer_period
        timer_ = this->create_wall_timer(
            timer_period, std::bind(&PosePubNode::timer_callback, this));
    }

    void read_parameters()
    {
        // Declare and acquire parameters
        declare_parameter<std::string>("map", map_frame);
        declare_parameter<std::string>("base_link", base_frame);
        declare_parameter<std::string>("pose", topic_republish);

        get_parameter("map", map_frame);
        get_parameter("base_link", base_frame);
        get_parameter("pose", topic_republish);
    }


private:
    void timer_callback()
    {
        RCLCPP_INFO(get_logger(),  "In timer_callback");
    }

    /* Here it is created the shared pointers to the Publisher and timer objects
    * defined above, and it is also created the variable count*/
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::string map_frame = "map";
    std::string base_frame = "base_link";
    std::string topic_republish = "pose";
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_{nullptr};
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePubNode>());
    rclcpp::shutdown();
    return 0;
}
