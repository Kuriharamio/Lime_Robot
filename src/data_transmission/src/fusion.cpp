
        

#include <memory>
#include <chrono>
#include <iostream>
#include <string>
#include <functional>
#include <vector>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "position_interfaces/msg/position.hpp" // 确保此消息存在
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

using namespace std::chrono_literals;

class FusionNode : public rclcpp::Node
{
public:
    FusionNode()
    : Node("fusion_node"),
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
    {
        rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
        // 订阅位置消息
        position_sub_ = this->create_subscription<position_interfaces::msg::Position>(
            "/position", 10,
            std::bind(&FusionNode::position_callback, this, std::placeholders::_1));

        // 订阅点云消息
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud", sensor_qos,
            std::bind(&FusionNode::point_cloud_callback, this, std::placeholders::_1));

        // 发布地图
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

        // 初始化地图参数
        initialize_map();

        RCLCPP_INFO(this->get_logger(), "Fusion node has been started.");
    }

private:
    // 地图参数初始化
    void initialize_map()
    {
        map_msg_.header.frame_id = "map";
        map_msg_.info.resolution = 0.05; // 每个栅格0.05米
        map_msg_.info.width = 1000;      // 50米
        map_msg_.info.height = 1000;     // 50米
        map_msg_.info.origin.position.x = -25.0;
        map_msg_.info.origin.position.y = -25.0;
        map_msg_.info.origin.position.z = 0.0;
        map_msg_.info.origin.orientation.w = 1.0;
        map_msg_.data.assign(map_msg_.info.width * map_msg_.info.height, -1); // 初始化为未知
    }

    // 位姿回调函数
    void position_callback(const position_interfaces::msg::Position::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // 更新里程计信息
        current_pose_.x = msg->position_x;
        current_pose_.y = msg->position_y;
        current_pose_.theta = msg->position_theta;

        RCLCPP_DEBUG(this->get_logger(), "Updated pose: x=%.2f, y=%.2f, theta=%.2f",
                     current_pose_.x, current_pose_.y, current_pose_.theta);

        // 更新地图
        update_map();

        // 发布地图
        publish_map();

        // 发布TF变换
        publish_tf();
    }

    // 点云回调函数
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *current_cloud);

        // 降采样
        // pcl::VoxelGrid<pcl::PointXYZ> voxel;
        // voxel.setLeafSize(0.05f, 0.05f, 0.05f); // 5cm
        // voxel.setInputCloud(current_cloud);
        // voxel.filter(*current_cloud);

        // 存储当前点云用于地图更新
        latest_cloud_ = current_cloud;

        if (!previous_cloud_) {
            previous_cloud_ = current_cloud;
            return;
        }

        // ICP匹配
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(current_cloud);
        icp.setInputTarget(previous_cloud_);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);

        if (icp.hasConverged()) {
            Eigen::Matrix4f transformation = icp.getFinalTransformation();
            RCLCPP_DEBUG(this->get_logger(), "ICP converged, score: %f", icp.getFitnessScore());

            // 提取旋转和平移
            Eigen::Matrix3f rotation = transformation.block<3,3>(0,0);
            Eigen::Vector3f translation = transformation.block<3,1>(0,3);

            // 更新当前位姿（简单地将ICP结果应用到当前位姿）
            double delta_x = translation[0];
            double delta_y = translation[1];
            double delta_theta = std::atan2(rotation(1,0), rotation(0,0));

            current_pose_.x += delta_x;
            current_pose_.y += delta_y;
            current_pose_.theta += delta_theta;

            RCLCPP_DEBUG(this->get_logger(),
                         "Pose updated by ICP: Δx=%.2f, Δy=%.2f, Δtheta=%.2f",
                         delta_x, delta_y, delta_theta);

            // 更新地图、发布地图和TF
            update_map();
            publish_map();
            publish_tf();
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
        }

        // 更新上一帧的点云
        previous_cloud_ = current_cloud;
    }

    // 更新地图函数
    void update_map()
    {
        if (!latest_cloud_) {
            RCLCPP_WARN(this->get_logger(), "No point cloud data available for map update.");
            return;
        }

        // 机器人的位置和姿态
        double robot_x = current_pose_.x;
        double robot_y = current_pose_.y;
        double robot_theta = current_pose_.theta;

        // 遍历每一个点云点
        for (const auto& point : latest_cloud_->points) {
            // 仅考虑XY平面
            double point_x = point.x;
            double point_y = point.y;

            // 旋转点云到世界坐标系
            double world_x = robot_x + (point_x * std::cos(robot_theta) - point_y * std::sin(robot_theta));
            double world_y = robot_y + (point_x * std::sin(robot_theta) + point_y * std::cos(robot_theta));

            // 将世界坐标转换为地图栅格坐标
            int map_x, map_y;
            world_to_map(world_x, world_y, map_x, map_y);

            // 机器人在地图中的栅格坐标
            int robot_map_x, robot_map_y;
            world_to_map(robot_x, robot_y, robot_map_x, robot_map_y);

            // 使用Bresenham算法绘制从机器人到测量点的射线，标记经过的栅格为自由
            std::vector<std::pair<int, int>> ray = bresenham(robot_map_x, robot_map_y, map_x, map_y);
            for (size_t i = 0; i < ray.size(); ++i) {
                int free_x = ray[i].first;
                int free_y = ray[i].second;

                if (i < ray.size() - 1) {
                    // 标记为自由
                    set_map_cell(free_x, free_y, 0);
                } else {
                    // 终点标记为占据
                    set_map_cell(free_x, free_y, 100);
                }
            }
        }
    }

    // 将世界坐标转换为地图栅格坐标
    void world_to_map(double x, double y, int& map_x, int& map_y)
    {
        map_x = static_cast<int>((x - map_msg_.info.origin.position.x) / map_msg_.info.resolution);
        map_y = static_cast<int>((y - map_msg_.info.origin.position.y) / map_msg_.info.resolution);
    }

    // 设置地图栅格的值
    void set_map_cell(int x, int y, int8_t value)
    {
        if (x >=0 && x < static_cast<int>(map_msg_.info.width) &&
            y >=0 && y < static_cast<int>(map_msg_.info.height)) {
            int index = y * map_msg_.info.width + x;
            map_msg_.data[index] = value;
        }
    }

    // Bresenham算法，用于绘制从起点到终点的直线
    std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1)
    {
        std::vector<std::pair<int, int>> points;

        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);

        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;

        int err = dx - dy;

        while (true) {
            points.emplace_back(std::make_pair(x0, y0));

            if (x0 == x1 && y0 == y1)
                break;

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }

        return points;
    }

    // 发布地图
    void publish_map()
    {
        map_msg_.header.stamp = this->now();
        map_pub_->publish(map_msg_);
    }

    // 发布TF变换
    void publish_tf()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";

        transform.transform.translation.x = current_pose_.x;
        transform.transform.translation.y = current_pose_.y;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, current_pose_.theta);
        q.normalize();

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }

    struct Pose {
        double x;
        double y;
        double theta;
    };

    // 订阅者和发布者
    rclcpp::Subscription<position_interfaces::msg::Position>::SharedPtr position_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // SLAM相关变量
    Pose current_pose_ = {0.0, 0.0, 0.0};
    nav_msgs::msg::OccupancyGrid map_msg_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
    std::mutex data_mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FusionNode>());
    rclcpp::shutdown();
    return 0;
}


        
// #include <memory>
// #include <chrono>
// #include <iostream>
// #include <string>
// #include <functional>
// #include <vector>
// #include <mutex>
// #include <cmath>

// #include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/odometry.hpp"         // 使用标准的Odometry消息
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "tf2_ros/transform_broadcaster.h"
// #include "tf2/LinearMath/Quaternion.h"
// #include "geometry_msgs/msg/transform_stamped.hpp"

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <pcl/registration/icp.h>
// #include <pcl/common/transforms.h>
// #include <pcl/filters/voxel_grid.h>

// using namespace std::chrono_literals;

// class FusionNode : public rclcpp::Node
// {
// public:
//     FusionNode()
//     : Node("fusion_node"),
//       tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
//     {
//         // 订阅里程计消息
//         odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//             "/odom", 10,
//             std::bind(&FusionNode::odom_callback, this, std::placeholders::_1));

//         // 订阅点云消息
//         point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             "/cloud", 10,
//             std::bind(&FusionNode::point_cloud_callback, this, std::placeholders::_1));

//         // 发布地图
//         map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

//         // 初始化地图参数
//         initialize_map();

//         RCLCPP_INFO(this->get_logger(), "Fusion node has been started.");
//     }

// private:
//     // 地图参数初始化
//     void initialize_map()
//     {
//         map_msg_.header.frame_id = "map";
//         map_msg_.info.resolution = 0.05; // 每个栅格0.05米
//         map_msg_.info.width = 1000;      // 50米
//         map_msg_.info.height = 1000;     // 50米
//         map_msg_.info.origin.position.x = -25.0;
//         map_msg_.info.origin.position.y = -25.0;
//         map_msg_.info.origin.position.z = 0.0;
//         map_msg_.info.origin.orientation.w = 1.0;
//         map_msg_.data.assign(map_msg_.info.width * map_msg_.info.height, -1); // 初始化为未知
//     }

//     // 里程计回调函数
//     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
//     {
//         std::lock_guard<std::mutex> lock(data_mutex_);

//         // 从里程计消息中提取位姿信息
//         current_pose_.x = msg->pose.pose.position.x;
//         current_pose_.y = msg->pose.pose.position.y;
        
//         // 提取旋转四元数并转换为欧拉角
//         double qx = msg->pose.pose.orientation.x;
//         double qy = msg->pose.pose.orientation.y;
//         double qz = msg->pose.pose.orientation.z;
//         double qw = msg->pose.pose.orientation.w;

//         // 使用tf2将四元数转换为欧拉角（仅考虑Z轴旋转）
//         tf2::Quaternion q(qx, qy, qz, qw);
//         current_pose_.theta = tf2::Quaternion(q).getY();
        

//         RCLCPP_INFO(this->get_logger(), "Updated pose from odom: x=%.2f, y=%.2f, theta=%.2f",
//                      current_pose_.x, current_pose_.y, current_pose_.theta);

//         // 更新地图
//         update_map();

//         // 发布地图
//         publish_map();

//         // 发布TF变换
//         publish_tf();
//     }

//     // 点云回调函数
//     void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
//     {
//         std::lock_guard<std::mutex> lock(data_mutex_);

//         pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//         pcl::fromROSMsg(*msg, *current_cloud);

//         // 降采样
//         // pcl::VoxelGrid<pcl::PointXYZ> voxel;
//         // voxel.setLeafSize(0.01f, 0.01f, 0.01f); // 5cm
//         // voxel.setInputCloud(current_cloud);
//         // voxel.filter(*current_cloud);

//         // 存储当前点云用于地图更新
//         latest_cloud_ = current_cloud;

//         if (!previous_cloud_) {
//             previous_cloud_ = current_cloud;
//             return;
//         }

//         // ICP匹配
//         pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//         icp.setInputSource(current_cloud);
//         icp.setInputTarget(previous_cloud_);
//         pcl::PointCloud<pcl::PointXYZ> Final;
//         icp.align(Final);

//         if (icp.hasConverged()) {
//             Eigen::Matrix4f transformation = icp.getFinalTransformation();
//             RCLCPP_DEBUG(this->get_logger(), "ICP converged, score: %f", icp.getFitnessScore());

//             // 提取旋转和平移
//             Eigen::Matrix3f rotation = transformation.block<3,3>(0,0);
//             Eigen::Vector3f translation = transformation.block<3,1>(0,3);

//             // 更新当前位姿（简单地将ICP结果应用到当前位姿）
//             double delta_x = translation[0];
//             double delta_y = translation[1];
//             double delta_theta = std::atan2(rotation(1,0), rotation(0,0));

//             current_pose_.x += delta_x;
//             current_pose_.y += delta_y;
//             current_pose_.theta += delta_theta;

//             RCLCPP_DEBUG(this->get_logger(),
//                          "Pose updated by ICP: Δx=%.2f, Δy=%.2f, Δtheta=%.2f",
//                          delta_x, delta_y, delta_theta);

//             // 更新地图、发布地图和TF
//             update_map();
//             publish_map();
//             publish_tf();
//         } else {
//             RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
//         }

//         // 更新上一帧的点云
//         previous_cloud_ = current_cloud;
//     }

//     // 更新地图函数
//     void update_map()
//     {
//         if (!latest_cloud_) {
//             RCLCPP_WARN(this->get_logger(), "No point cloud data available for map update.");
//             return;
//         }

//         // 机器人的位置和姿态
//         double robot_x = current_pose_.x;
//         double robot_y = current_pose_.y;
//         double robot_theta = current_pose_.theta;

//         // 遍历每一个点云点
//         for (const auto& point : latest_cloud_->points) {
//             // 仅考虑XY平面
//             double point_x = point.x;
//             double point_y = point.y;

//             // 旋转点云到世界坐标系
//             double world_x = robot_x + (point_x * std::cos(robot_theta) - point_y * std::sin(robot_theta));
//             double world_y = robot_y + (point_x * std::sin(robot_theta) + point_y * std::cos(robot_theta));

//             // 将世界坐标转换为地图栅格坐标
//             int map_x, map_y;
//             world_to_map(world_x, world_y, map_x, map_y);

//             // 机器人在地图中的栅格坐标
//             int robot_map_x, robot_map_y;
//             world_to_map(robot_x, robot_y, robot_map_x, robot_map_y);

//             // 使用Bresenham算法绘制从机器人到测量点的射线，标记经过的栅格为自由
//             std::vector<std::pair<int, int>> ray = bresenham(robot_map_x, robot_map_y, map_x, map_y);
//             for (size_t i = 0; i < ray.size(); ++i) {
//                 int free_x = ray[i].first;
//                 int free_y = ray[i].second;

//                 if (i < ray.size() - 1) {
//                     // 标记为自由
//                     set_map_cell(free_x, free_y, 0);
//                 } else {
//                     // 终点标记为占据
//                     set_map_cell(free_x, free_y, 100);
//                 }
//             }
//         }
//     }

//     // 将世界坐标转换为地图栅格坐标
//     void world_to_map(double x, double y, int& map_x, int& map_y)
//     {
//         map_x = static_cast<int>((x - map_msg_.info.origin.position.x) / map_msg_.info.resolution);
//         map_y = static_cast<int>((y - map_msg_.info.origin.position.y) / map_msg_.info.resolution);
//     }

//     // 设置地图栅格的值
//     void set_map_cell(int x, int y, int8_t value)
//     {
//         if (x >=0 && x < static_cast<int>(map_msg_.info.width) &&
//             y >=0 && y < static_cast<int>(map_msg_.info.height)) {
//             int index = y * map_msg_.info.width + x;
//             map_msg_.data[index] = value;
//         }
//     }

//     // Bresenham算法，用于绘制从起点到终点的直线
//     std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1)
//     {
//         std::vector<std::pair<int, int>> points;

//         int dx = std::abs(x1 - x0);
//         int dy = std::abs(y1 - y0);

//         int sx = x0 < x1 ? 1 : -1;
//         int sy = y0 < y1 ? 1 : -1;

//         int err = dx - dy;

//         while (true) {
//             points.emplace_back(std::make_pair(x0, y0));

//             if (x0 == x1 && y0 == y1)
//                 break;

//             int e2 = 2 * err;
//             if (e2 > -dy) {
//                 err -= dy;
//                 x0 += sx;
//             }
//             if (e2 < dx) {
//                 err += dx;
//                 y0 += sy;
//             }
//         }

//         return points;
//     }

//     // 发布地图
//     void publish_map()
//     {
//         map_msg_.header.stamp = this->now();
//         map_pub_->publish(map_msg_);
//     }

//     // 发布TF变换
//     void publish_tf()
//     {
//         geometry_msgs::msg::TransformStamped transform;
//         transform.header.stamp = this->now();
//         transform.header.frame_id = "map";
//         transform.child_frame_id = "odom";

//         transform.transform.translation.x = current_pose_.x;
//         transform.transform.translation.y = current_pose_.y;
//         transform.transform.translation.z = 0.0;

//         tf2::Quaternion q;
//         q.setRPY(0, 0, current_pose_.theta);
//         q.normalize();

//         transform.transform.rotation.x = q.x();
//         transform.transform.rotation.y = q.y();
//         transform.transform.rotation.z = q.z();
//         transform.transform.rotation.w = q.w();

//         tf_broadcaster_->sendTransform(transform);
//     }

//     struct Pose {
//         double x;
//         double y;
//         double theta;
//     };

//     // 订阅者和发布者
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
//     std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

//     // SLAM相关变量
//     Pose current_pose_ = {0.0, 0.0, 0.0};
//     nav_msgs::msg::OccupancyGrid map_msg_;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
//     std::mutex data_mutex_;
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<FusionNode>());
//     rclcpp::shutdown();
//     return 0;
// }
