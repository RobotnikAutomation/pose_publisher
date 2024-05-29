#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

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

        // Create ROS2 tf listener
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
        declare_parameter<std::string>("robot", tf_prefix);
        declare_parameter<std::string>("map", map_frame);
        declare_parameter<std::string>("base_link", base_frame);
        declare_parameter<std::string>("pose", topic_republish);

        get_parameter("robot", tf_prefix);
        get_parameter("map", map_frame);
        get_parameter("base_link", base_frame);
        get_parameter("pose", topic_republish);
    }


private:
    void timer_callback()
    {
        RCLCPP_INFO(get_logger(),  "In timer_callback");

        std::string fromFrameRel = (tf_prefix + "/" + base_frame).c_str();
        std::string toFrameRel = (tf_prefix + "/" + map_frame).c_str();

        // std::string fromFrameRel = (tf_prefix + "/" + map_frame).c_str();
        // std::string toFrameRel = (tf_prefix + "/" + base_frame).c_str();

        RCLCPP_INFO(get_logger(),  "fromFrameRel topic (target_frame): %s", fromFrameRel.c_str());
        RCLCPP_INFO(get_logger(),  "toFrameRel topic (source_frame): %s",toFrameRel.c_str());

        geometry_msgs::msg::TransformStamped tf_transform;
        bool tf_ok = true;

        try {
            tf_transform = tf_buffer_->lookupTransform(
                toFrameRel, fromFrameRel,
                tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                get_logger(), "Could not transform %s to %s: %s",
                toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            tf_ok = false;
            return;
        }

        if(tf_ok)
        {
            RCLCPP_INFO(get_logger(),  "TF okey");

            geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
            pose_stamped.header.stamp = rclcpp::Clock{}.now();
            //pose_stamped.header.stamp = rclcpp::Time{};
            pose_stamped.header.frame_id = tf_prefix + "/" + map_frame;

            pose_stamped.pose.pose.position.x = tf_transform.transform.translation.x;
            pose_stamped.pose.pose.position.y = tf_transform.transform.translation.y;
            pose_stamped.pose.pose.position.z = tf_transform.transform.translation.z;

            pose_stamped.pose.pose.orientation.x = tf_transform.transform.rotation.x;
            pose_stamped.pose.pose.orientation.y = tf_transform.transform.rotation.y;
            pose_stamped.pose.pose.orientation.z = tf_transform.transform.rotation.z;
            pose_stamped.pose.pose.orientation.w = tf_transform.transform.rotation.w;

            pose_publisher_->publish(pose_stamped);
        }
    }

    /* Here it is created the shared pointers to the Publisher and timer objects
    * defined above, and it is also created the variable count*/
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::string tf_prefix = "robot";
    std::string map_frame = "map";
    std::string base_frame = "base_link";
    std::string topic_republish = "pose";
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePubNode>());
    rclcpp::shutdown();
    return 0;
}
