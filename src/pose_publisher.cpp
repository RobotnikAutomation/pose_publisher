#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>

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
        RCLCPP_DEBUG(get_logger(),  "Connection established");

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
            this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_republish, rclcpp::QoS(1).keep_last(1).reliable().transient_local());
        RCLCPP_DEBUG(get_logger(),  "Pose_publisher created with topic: %s", topic_republish.c_str());

        // Call timer_callback function every timer_period
        timer_ = this->create_wall_timer(
            timer_period, std::bind(&PosePubNode::timer_callback, this));
    }

    void read_parameters()
    {
        double frequency{50.0};
        // Declare and acquire parameters
        declare_parameter<double>("frequency", frequency);
        declare_parameter<std::string>("tf_prefix", tf_prefix);
        declare_parameter<std::string>("map_frame", map_frame);
        declare_parameter<std::string>("base_frame", base_frame);
        declare_parameter<std::string>("topic_republish", topic_republish);

        get_parameter("frequency", frequency);
        get_parameter("tf_prefix", tf_prefix);
        get_parameter("map_frame", map_frame);
        get_parameter("base_frame", base_frame);
        get_parameter("topic_republish", topic_republish);

        timer_period = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<float>(1.0 / frequency));
    }

private:
    void timer_callback()
    {
        const std::string fromFrame = (tf_prefix + "/" + base_frame).c_str();
        const std::string toFrame = (tf_prefix + "/" + map_frame).c_str();

        geometry_msgs::msg::TransformStamped tf_transform;
        bool tf_ok = true;

        // Look up for the transformation between fromFrame and toFrame frames
        try {
            tf_transform = tf_buffer_->lookupTransform(
                toFrame, fromFrame,
                tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(
                get_logger(), "Could not transform %s to %s: %s",
                toFrame.c_str(), fromFrame.c_str(), ex.what());
            tf_ok = false;
            return;
        }

        if(tf_ok)
        {
            // Publish the TF transformation
            RCLCPP_DEBUG(get_logger(), "Publishing the TF transformation");
            geometry_msgs::msg::PoseWithCovarianceStamped pose_stamped;
            pose_stamped.header.stamp = tf_transform.header.stamp;
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

    // Create the shared variables, pointers and timer objects
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::string tf_prefix{"robot"};
    std::string map_frame{"map"};
    std::string base_frame{"base_link"};
    std::string topic_republish{"pose"};
    std::chrono::microseconds timer_period{500'000};
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
