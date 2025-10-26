#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/empty.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp> 
#include <std_msgs/msg/string.hpp>
using namespace std::chrono_literals;

class InitialPoseAndRotateNode : public rclcpp::Node
{
public:
    InitialPoseAndRotateNode()
        : Node("initial_pose_and_rotate_node")
    {
        RCLCPP_INFO(this->get_logger(), "Node started, waiting for rviz2 to start...");
        std::this_thread::sleep_for(5s); 
        reading_publisher_ = this->create_publisher<std_msgs::msg::String>("/read_aloud", 10);
        
        auto initial_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        auto client = this->create_client<std_srvs::srv::Empty>("reinitialize_global_localization");
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();

        int trytimes = 0;

        std_msgs::msg::String msg;
        msg.data = "正在自动定位中";
        reading_publisher_->publish(msg);

        while (!client->wait_for_service(1s)) {
        if (trytimes++ >= 5) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call global localization after 5 attempts.");
        }
        RCLCPP_WARN(this->get_logger(), "Waiting for the service to be available...");
        sleep(1);
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Call to global localization successful.");
        std::this_thread::sleep_for(5s);  // Wait for service to start
        // Create publisher for /cmd_vel
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        rotate_timer_ = this->create_wall_timer(100ms, std::bind(&InitialPoseAndRotateNode::rotate_callback, this));
        rotate_duration_ = 37.7;  // 37.7 seconds
        start_time_ = this->now().seconds();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call global localization.");
    }

     
    }

private:
    void rotate_callback()
    {
        double current_time = this->now().seconds();
        double elapsed_time = current_time - start_time_;

        if (elapsed_time < rotate_duration_)
        {
            auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
            twist_msg->angular.z = 0.5;  // Angular velocity 0.5 rad/s
            RCLCPP_INFO(this->get_logger(), "Rotating...");
            cmd_vel_publisher_->publish(*twist_msg);
        }else if(elapsed_time < rotate_duration_ + 3){
            RCLCPP_INFO(this->get_logger(), "Stop Rotating...");
            auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
            twist_msg->angular.z = 0.0;  // Angular velocity 1 rad/s
            cmd_vel_publisher_->publish(*twist_msg);
            
        }else{
            rotate_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Rotation completed.");
            auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
            twist_msg->angular.z = 0.0;  // Angular velocity 1 rad/s
            cmd_vel_publisher_->publish(*twist_msg);
            std_msgs::msg::String msg;
            msg.data = "定位成功";
            reading_publisher_->publish(msg);
            rclcpp::shutdown();
        }
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reading_publisher_;
    rclcpp::TimerBase::SharedPtr rotate_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    double rotate_duration_;
    double start_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPoseAndRotateNode>());
    rclcpp::shutdown();
    return 0;
}
