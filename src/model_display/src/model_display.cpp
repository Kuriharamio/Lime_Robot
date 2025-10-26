#include <iostream>
#include <memory>
#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "urdf/model.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

class ModelPublisher : public rclcpp::Node
{
public:
    ModelPublisher(): Node("model_display")
    {
        std::string urdf_file = "/home/mio/FIles/ros/omni_robot/src/model_display/resource/mini_omni_robot.urdf";
        std::ifstream inf(urdf_file.c_str());
        if (!inf) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file: %s", urdf_file.c_str());
            return;
        }
        std::string urdf_str((std::istreambuf_iterator<char>(inf)), std::istreambuf_iterator<char>());
        auto urdf_msg = std::make_shared<std_msgs::msg::String>();
        urdf_msg->data = urdf_str;
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
        description_publisher_ = this->create_publisher<std_msgs::msg::String>("robot_description", qos);
        description_publisher_->publish(*urdf_msg);

        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publishStaticTransforms();
        
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_publisher_;
    urdf::Model urdf_model_;
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    void publishStaticTransforms()
    {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(createTransform("base_link", "front_wheel", {0.1, 0.0, 0.03}, {0.0, 0.0, 0.0}));
        transforms.push_back(createTransform("base_link", "left_wheel", {-0.055, 0.09, 0.03}, {0.0, 0.0, -1.04719755}));
        transforms.push_back(createTransform("base_link", "right_wheel", {-0.055, -0.09, 0.03}, {0.0, 0.0, 1.04719755}));
        transforms.push_back(createTransform("base_link", "imu_link", {0.10, 0.0, 0.10}, {0.0, 0.0, 0.0}));
        transforms.push_back(createTransform("base_link", "laser_link", {0.0, 0.0, 0.105}, {0.0, 0.0, 0.8}));
        transforms.push_back(createTransform("base_link", "camera_link",  {0.0, 0.0, 0.13}, {0.0, 0.0, 0.0}));

        static_tf_broadcaster_->sendTransform(transforms);
    }

  geometry_msgs::msg::TransformStamped createTransform(
      const std::string& parent_frame,
      const std::string& child_frame,
      const std::array<double, 3>& translation,
      const std::array<double, 3>& rotation)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = translation[0];
    transform.transform.translation.y = translation[1];
    transform.transform.translation.z = translation[2];

    tf2::Quaternion q;
    q.setRPY(rotation[0], rotation[1], rotation[2]);

    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    return transform;
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ModelPublisher>();
  rclcpp::WallRate loop_rate(10.0);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    // node->publish_tf();
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}

