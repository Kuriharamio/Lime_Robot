#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <mutex>
#include <thread>
#include <future>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

class SlamNode : public rclcpp::Node
{
public:
    SlamNode()
        : Node("slam_node")
    {
        // Create a subscription for the PointCloud2 messages
        rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
        laser_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud", sensor_qos, std::bind(&SlamNode::laserCallback, this, std::placeholders::_1));

        // Initialize publishers for the occupancy grid map
        map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_map", 10);
        grid_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);

        // Initialize timers
        downsample_timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&SlamNode::downsampleAndPublish, this));

        convert_to_map_timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&SlamNode::convertToOccupancyGridAndPublish, this));

        relign_timer_ = this->create_wall_timer(
            std::chrono::seconds(10), std::bind(&SlamNode::realign, this));

        relocalization_timer_ = this->create_wall_timer(
            std::chrono::seconds(2), std::bind(&SlamNode::relocalize, this));

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // cloud_accumulated_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

private:
    void laserCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_current(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud_current);

        if (cloud_map_->empty()) {
            *cloud_map_ = *cloud_current;
        } else {
            geometry_msgs::msg::TransformStamped tf_odom_base;
            try
            {
                tf_odom_base = tf_buffer_->lookupTransform("odom", "imu_link", tf2::TimePointZero); 
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
                return;
            }
            Eigen::Matrix4f transform = tf2::transformToEigen(tf_odom_base.transform).matrix().cast<float>();

            pcl::transformPointCloud(*cloud_current, *cloud_current, transform);
            cloud_accumulated_.push_back(*cloud_current);
            *cloud_map_ += *cloud_current;
        }
    }

    void downsampleAndPublish()
    {
        RCLCPP_INFO(this->get_logger(), "Downsampling and publishing map...");
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud_map_);
        voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_grid.filter(*cloud_filtered);
        
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_filtered, cloud_msg);
        cloud_msg.header.frame_id = "map";
        map_publisher_->publish(cloud_msg);
        *cloud_map_ = *cloud_filtered;
        pcl::io::savePCDFile("/home/mio/FIles/ros/Lime_Robot/src/slam/map/map.pcd", *cloud_map_); 
    }

    void convertToOccupancyGridAndPublish()
    {
        RCLCPP_INFO(this->get_logger(), "Converting to occupancy grid and publishing...");

        // Convert the point cloud to an occupancy grid map
        const float resolution = 0.05; // Grid resolution in meters per cell
        const int width = 200; // Number of cells in x direction
        const int height = 200; // Number of cells in y direction

        nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
        occupancy_grid_msg.info.resolution = resolution;
        occupancy_grid_msg.info.width = width;
        occupancy_grid_msg.info.height = height;

        occupancy_grid_msg.info.origin.position.x = -width * resolution / 2;
        occupancy_grid_msg.info.origin.position.y = -height * resolution / 2;
        occupancy_grid_msg.info.origin.position.z = 0;
        occupancy_grid_msg.info.origin.orientation.w = 1.0;

        occupancy_grid_msg.data.resize(width * height, -1);

        for (const auto &point : cloud_map_->points) {
            int x_idx = static_cast<int>((point.x - occupancy_grid_msg.info.origin.position.x) / resolution);
            int y_idx = static_cast<int>((point.y - occupancy_grid_msg.info.origin.position.y) / resolution);
           
            if (x_idx >= 0 && x_idx < width && y_idx >= 0 && y_idx < height) {
                int index = y_idx * width + x_idx;
                occupancy_grid_msg.data[index] = 100; // Occupied cell
            
            }
        }

        occupancy_grid_msg.header.stamp = this->get_clock()->now();
        occupancy_grid_msg.header.frame_id = "map";
        grid_map_publisher_->publish(occupancy_grid_msg);
    }

    // void realign()
    // {
    //     RCLCPP_INFO(this->get_logger(), "Relocalizing...");
    //     // Implementation for relocalization (ICP related logic and transformation adjustments)
    //     if (cloud_accumulated_.size() < 20) return;

    //     pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     for (size_t i = 0; i < cloud_accumulated_.size(); i += cloud_accumulated_.size() / 20) {
    //         *sampled_cloud += cloud_accumulated_.at(i);
    //     }

    //     cloud_accumulated_.clear();

    //     pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
    //     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    //     tree1->setInputCloud(sampled_cloud);
    //     tree2->setInputCloud(cloud_map_);
    //     icp.setSearchMethodSource(tree1);
    //     icp.setSearchMethodTarget(tree2);
    //     icp.setInputSource(sampled_cloud);
    //     icp.setInputTarget(cloud_map_);
    //     icp.setTransformationEpsilon(1e-10);
    //     icp.setMaxCorrespondenceDistance(1);
    //     icp.setEuclideanFitnessEpsilon(0.05);
    //     icp.setMaximumIterations(35);

    //     pcl::PointCloud<pcl::PointXYZ> final_cloud;
    //     icp.align(final_cloud);

    //     pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     *combined_cloud = *cloud_map_;
    //     *combined_cloud += final_cloud;

    //     if (icp.hasConverged()) {
    //         *cloud_map_ = final_cloud;
    //     }
    // }

    void realign()
    {
        RCLCPP_INFO(this->get_logger(), "Relocalizing...");
        // Implementation for relocalization (ICP related logic and transformation adjustments)
        if (cloud_accumulated_.size() < 10) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *combined_cloud = *cloud_map_;

        for (size_t i = 0; i < cloud_accumulated_.size(); i += cloud_accumulated_.size() / 10) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            *sampled_cloud = cloud_accumulated_.at(i);
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
            tree1->setInputCloud(sampled_cloud);
            tree2->setInputCloud(combined_cloud);
            icp.setSearchMethodSource(tree1);
            icp.setSearchMethodTarget(tree2);
            icp.setInputSource(sampled_cloud);
            icp.setInputTarget(combined_cloud);
            icp.setTransformationEpsilon(1e-10);
            icp.setMaxCorrespondenceDistance(1);
            icp.setEuclideanFitnessEpsilon(0.05);
            icp.setMaximumIterations(35);
            pcl::PointCloud<pcl::PointXYZ>::Ptr align_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            icp.align(*align_cloud);
            *combined_cloud += *align_cloud;
            pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
            voxel_grid.setInputCloud(combined_cloud);
            voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f);
            voxel_grid.filter(*combined_cloud);
        }

        cloud_accumulated_.clear();

        
        *cloud_map_ = *combined_cloud;
    }

    void relocalize()
    {
        RCLCPP_INFO(this->get_logger(), "Realigning and publishing TF...");
        if (cloud_accumulated_.empty()) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *latest_cloud += cloud_accumulated_.back();

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
        tree1->setInputCloud(latest_cloud);
        tree2->setInputCloud(cloud_map_);
        icp.setSearchMethodSource(tree1);
        icp.setSearchMethodTarget(tree2);
        icp.setInputSource(latest_cloud);
        icp.setInputTarget(cloud_map_);
        icp.setTransformationEpsilon(1e-10);
        icp.setMaxCorrespondenceDistance(1);
        icp.setEuclideanFitnessEpsilon(0.05);
        icp.setMaximumIterations(35);

        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        icp.align(aligned_cloud);

          // if (icp.hasConverged()) {
        //     for (size_t i = 0; i < aligned_cloud.points.size(); ++i) {
        //         const auto& aligned_point = aligned_cloud.points[i];
        //         pcl::PointXYZ nearest_point;
        //         int nearest_index = -1;
        //         float min_distance = std::numeric_limits<float>::max();
                
        //         for (size_t j = 0; j < cloud_map_->points.size(); ++j) {
        //             const auto& map_point = cloud_map_->points[j];
        //             float distance = pcl::euclideanDistance(aligned_point, map_point);
        //             if (distance < min_distance) {
        //                 min_distance = distance;
        //                 nearest_point = map_point;
        //                 nearest_index = j;
        //             }
        //         }
        //     
        //         if (nearest_index != -1) {
        //             cloud_map_->points[nearest_index] = aligned_point;
        //         }
        //     }

        //     cloud_map_->width = static_cast<uint32_t>(cloud_map_->points.size());
        //     cloud_map_->height = 1;
        //     cloud_map_->is_dense = true;
        // } else {
        //   ]
        // }

        // Eigen::Vector2f min_pt(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        // Eigen::Vector2f max_pt(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());

        // for (const auto& point : aligned_cloud.points) {
        //     if (point.x < min_pt.x()) min_pt.x() = point.x;
        //     if (point.y < min_pt.y()) min_pt.y() = point.y;
        //     if (point.x > max_pt.x()) max_pt.x() = point.x;
        //     if (point.y > max_pt.y()) max_pt.y() = point.y;
        // }

        // // 在 cloud_map_ 中删除点，前提是 aligned_cloud 在这个位置上没有点
        // cloud_map_->points.erase(
        //     std::remove_if(cloud_map_->points.begin(), cloud_map_->points.end(),
        //         [&](const pcl::PointXYZ& point) {
        //             // 检查 aligned_cloud 在这个位置上是否存在点
        //             bool exists_in_aligned = false;
        //             for (const auto& aligned_point : aligned_cloud.points) {
        //                 if (fabs(point.x - aligned_point.x < 0.05) && fabs(point.y - aligned_point.y)<0.05) {
        //                     exists_in_aligned = true;
        //                     break; // 找到与 aligned_cloud 中的点匹配，停止检查
        //                 }
        //             }
        //             // 进行删除条件判断
        //             return (point.x >= min_pt.x() && point.x <= max_pt.x() &&
        //                     point.y >= min_pt.y() && point.y <= max_pt.y() &&
        //                     !exists_in_aligned); // 确保 aligned_cloud 中没有点
        //         }),
        //     cloud_map_->points.end() // erase会删除被标记的点
        // );

        // // 更新点云的width
        // cloud_map_->width = cloud_map_->points.size();
        // cloud_map_->height = 1; // 如果高度是1，确保是一个无组织点云

        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
        Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);

        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = this->get_clock()->now();
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = "odom";

        map_to_odom.transform.translation.x = translation(0);
        map_to_odom.transform.translation.y = translation(1);
        map_to_odom.transform.translation.z = translation(2);
        Eigen::Quaternionf quaternion(rotation);
        map_to_odom.transform.rotation.x = quaternion.x();
        map_to_odom.transform.rotation.y = quaternion.y();
        map_to_odom.transform.rotation.z = quaternion.z();
        map_to_odom.transform.rotation.w = quaternion.w();

        tf_broadcaster_->sendTransform(map_to_odom);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr laser_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_publisher_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_accumulated_;
    rclcpp::TimerBase::SharedPtr downsample_timer_;
    rclcpp::TimerBase::SharedPtr convert_to_map_timer_;
    rclcpp::TimerBase::SharedPtr relocalization_timer_;
    rclcpp::TimerBase::SharedPtr relign_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamNode>());
    rclcpp::shutdown();
    return 0;
}
