#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "serial/serial.h"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <thread> 
#include <chrono> 
#include <vector>
#include <algorithm>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/static_transform_broadcaster.h"
typedef struct 
{
  float w;
  float x;
  float y;
  float z;
}quaternion_t;

#define Header 0x55

#define Type_Time 0x50
#define Type_Acceleration_raw 0x51
#define Type_Angular_Velocity_raw 0x52
#define Type_Angle_raw 0x53
#define Type_Quaternion_raw 0x59
#define Type_OffSet 0x05

#define REG_ACC 0x34 //包含三个加速度和X角速度
#define REG_AnV 0x37 //包含三个角速度和磁场X
#define REG_Angle_raw 0x3D //包含横滚角、俯仰角、航向角、温度 
#define REG_Quat 0x51 //包含四元数

double g = 9.7887;
double Acceleration_raw[3];
double Angle_raw[3];
double Angular_Velocity_raw[3];
double Quaternion_raw[4];
double Temperture;

double v_x,v_y;

using namespace std::chrono_literals;

size_t point_num = 480;
class SerialNode : public rclcpp::Node
{
public:
    SerialNode() : Node("serial_node")
    {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu",10);
        odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>();
        imu_msg_ = std::make_shared<sensor_msgs::msg::Imu>();
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        scan_msg_ = std::make_shared<sensor_msgs::msg::LaserScan>();
        scan_msg_->header.frame_id = "laser_link";
        scan_msg_->angle_min = 0.0;
        scan_msg_->angle_max = 2 * M_PI;
        scan_msg_->angle_increment = 0.8 * (M_PI / 180.0); 
        scan_msg_->range_min = 0.2; 
        scan_msg_->range_max = 200.0;
        scan_msg_->ranges.resize(point_num);
        scan_msg_->intensities.resize(point_num);
        accumulated_points_ = 0;
        timer_ = this->create_wall_timer(10us, std::bind(&SerialNode::timer_callback, this));
        tf_timer_ = this->create_wall_timer(10ms, std::bind(&SerialNode::publish_tf, this));
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&SerialNode::command_callback, this, std::placeholders::_1));
        try
        {
            ser.setPort("/dev/ttyUSB0"); 
            ser.setBaudrate(921600); 
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        }
        catch (serial::IOException& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port ");
        }

        if (ser.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial Port initialized");
        } else {
            return;
        }

    }

private:
    void timer_callback()
    {
        if (ser.available()) {
            std::vector<uint8_t> buffer(ser.available());
            ser.read(buffer.data(), buffer.size());
            parse_data(buffer);
        }
    }
    
    void parse_data(const std::vector<uint8_t>& buffer)
    {
        packet_buffer_.insert(packet_buffer_.end(), buffer.begin(), buffer.end());

        while (packet_buffer_.size() >= 82) {
            auto it_start = std::find(packet_buffer_.begin(), packet_buffer_.end(), 255);
            if (it_start == packet_buffer_.end()) {
                packet_buffer_.clear(); // 没有找到起始字节，清空缓冲区
                break;
            }
            if (std::distance(it_start, packet_buffer_.end()) < 82) {
                break; // 不足69字节，等待更多数据
            }
            if (*(it_start + 1) == 255 && *(it_start + 13) == 0xA5 && *(it_start + 71) == 0x55) {
                
                std::vector<uint8_t> position_packet(it_start, it_start + 13);
                process_position_packet(position_packet);

                std::vector<uint8_t> scan_packet(it_start + 13, it_start + 71);
                process_scan_packet(scan_packet);

                std::vector<uint8_t> imu_packet(it_start + 71, it_start + 82);
                process_imu_packet(imu_packet);
                /*
                for(auto it : scan_packet){
                    RCLCPP_INFO(this->get_logger(),"%d",it);
                }
                */
                packet_buffer_.erase(packet_buffer_.begin(), it_start + 82); // 移除已处理的数据
            } else {
                packet_buffer_.erase(packet_buffer_.begin(), it_start + 1); // 跳过当前字节继续查找
            }
        }
    }

    void Euler2Quaternion(float roll, float pitch, float yaw, quaternion_t &q)
    {
        // 传入机器人的欧拉角 roll、pitch 和 yaw。
        // 计算欧拉角的 sin 和 cos 值，分别保存在 cr、sr、cy、sy、cp、sp 六个变量中    
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        // 计算出四元数的四个分量 q.w、q.x、q.y、q.z
        q.w = cy * cp * cr + sy * sp * sr;
        q.x = cy * cp * sr - sy * sp * cr;
        q.y = sy * cp * sr + cy * sp * cr;
        q.z = sy * cp * cr - cy * sp * sr;
    }

    void publish_tf()
    {
        
        geometry_msgs::msg::TransformStamped transform;
        double seconds = this->now().seconds();
        transform.header.stamp = rclcpp::Time(static_cast<uint64_t>(seconds * 1e9));
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = odom_msg_->pose.pose.position.x;
        transform.transform.translation.y = odom_msg_->pose.pose.position.y;
        transform.transform.translation.z = odom_msg_->pose.pose.position.z;
        transform.transform.rotation.x = odom_msg_->pose.pose.orientation.x;
        transform.transform.rotation.y = odom_msg_->pose.pose.orientation.y;
        transform.transform.rotation.z = odom_msg_->pose.pose.orientation.z;
        transform.transform.rotation.w = odom_msg_->pose.pose.orientation.w;

        // 广播坐标变换信息
        tf_broadcaster_->sendTransform(transform);
    }

    void process_position_packet(const std::vector<uint8_t>& packet) {
        double position_x, position_y, position_theta;
        position_x = position_y = position_theta = 0;
        position_x = packet[4] + packet[5] / 100.0;
        position_y = packet[7] + packet[8] / 100.0;
        position_theta = packet[10] + packet[11] / 100.0;

        if (packet[3] == 1) {
            position_x = -position_x;
        }
        if (packet[6] == 1) {
            position_y = -position_y;
        }
        if (packet[9] == 1) {
            position_theta = -position_theta;
        }
        odom_msg_->header.stamp = this->now();
        odom_msg_->header.frame_id = "odom";
        odom_msg_->child_frame_id = "base_link";
        odom_msg_->pose.pose.position.x = position_x;
        odom_msg_->pose.pose.position.y = position_y;
        odom_msg_->pose.pose.position.z = 0;
        
        quaternion_t q;
        Euler2Quaternion(0, 0, position_theta, q);

        odom_msg_->pose.pose.orientation.x = q.x;
        odom_msg_->pose.pose.orientation.y = q.y;
        odom_msg_->pose.pose.orientation.z = q.z;
        odom_msg_->pose.pose.orientation.w = q.w;

        odom_pub_->publish(*odom_msg_);
        // publish_tf();

    }

    void process_scan_packet(const std::vector<uint8_t>& scan_data) {
        if (scan_data.size() < 58)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid scan_data length");
            return;
        }

        uint16_t frame_header = (scan_data[0] << 8) | scan_data[1];
        if (frame_header != 0xA55A)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid frame header");
            return;
        }

        uint8_t frame_length = scan_data[2];
        if (frame_length != scan_data.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid frame length");
            return;
        }
        
        //uint16_t speed = (scan_data[3] << 8) | scan_data[4];
        uint16_t start_angle = (scan_data[5] << 8) | scan_data[6];
        //uint16_t end_angle = (scan_data[55] << 8) | scan_data[56];

        uint8_t crc = 0;
        for (size_t i = 0; i < 57; ++i)
        {
            crc += scan_data[i];
        }
        if (crc != scan_data[57])
        {
            //RCLCPP_ERROR(this->get_logger(), "scan CRC check failed");
            return;
        }
        //RCLCPP_INFO(this->get_logger(), "Valid data packet.");
        size_t num_points = 16; // 每个数据包有16个点

        for (size_t i = 0; i < num_points; ++i)
        {
            uint16_t distance = (scan_data[7 + i * 3] << 8) | scan_data[8 + i * 3];
            uint8_t intensity = scan_data[9 + i * 3];
            double angle = start_angle / 100.0 + i * 0.8 ;

            if (angle >= 360.0) angle -= 360.0;

            size_t index = point_num - static_cast<size_t>(angle / 0.8);

            if (index < point_num) 
            {
                scan_msg_->ranges[index] = distance / 1000.0;
                scan_msg_->intensities[index] = intensity;
                accumulated_points_ ++;
            }
        }

        if (accumulated_points_ >= point_num)
        {
            scan_msg_->header.stamp = this->now();
            scan_pub_->publish(*scan_msg_);
            accumulated_points_ = 0;

            // RCLCPP_INFO(this->get_logger(), "Finished publishing a scan packet.");          
        }
    }

    void process_imu_packet(const std::vector<uint8_t>& imu_data) {

        if (imu_data.size() < 11)
        {
            //RCLCPP_ERROR(this->get_logger(), "Invalid imu_data length");
            return;
        }
        if (imu_data[0] != Header)
        {
            //RCLCPP_ERROR(this->get_logger(), "Invalid frame header");
            return;
        }

        
        uint8_t CRC_SUM = 0;
        for(int i =0; i<10;i++){
            CRC_SUM+=imu_data[i];
        }
        if(CRC_SUM != imu_data[10]){
            //RCLCPP_ERROR(this->get_logger(), "imu CRC check failed");
            return;
        }
        
        switch(imu_data[1]){
        case Type_Acceleration_raw:
            for(int i = 0; i<3; i++){
                Acceleration_raw[i] = ((int16_t)(((uint16_t)imu_data[3+2*i]<<8)|imu_data[2+2*i]))/32768.0 * 16 * g;
            }
            
            break;
        case Type_Angular_Velocity_raw:
            for(int i = 0; i<3; i++){
                Angular_Velocity_raw[i] = ((int16_t)(((uint16_t)imu_data[3+2*i]<<8)|imu_data[2+2*i]))/32768.0 * 4000;
            }
            break;
        case Type_Quaternion_raw:
            for(int i = 0; i<4; i++){
                Quaternion_raw[i] = ((int16_t)(((uint16_t)imu_data[3+2*i]<<8)|imu_data[2+2*i]))/32768.0;
            }
            break;
        }

        imu_msg_->linear_acceleration.x = Acceleration_raw[0];
        imu_msg_->linear_acceleration.y = Acceleration_raw[1];
        imu_msg_->linear_acceleration.z = Acceleration_raw[2];

        imu_msg_->angular_velocity.x = Angular_Velocity_raw[0];
        imu_msg_->angular_velocity.y = Angular_Velocity_raw[1];
        imu_msg_->angular_velocity.z = Angular_Velocity_raw[2];

        imu_msg_->orientation.w = Quaternion_raw[0]; 
        imu_msg_->orientation.x = Quaternion_raw[1]; 
        imu_msg_->orientation.y = Quaternion_raw[2]; 
        imu_msg_->orientation.z = Quaternion_raw[3];  

        imu_msg_->header.stamp = this->now();
        imu_msg_->header.frame_id = "imu_link";
        // imu_pub_->publish(*imu_msg_);

        // RCLCPP_INFO(this->get_logger(), "Publishing IMU data");
        
    }



    void command_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", msg->linear.x, msg->linear.y,msg->angular.z);
        uint8_t velocity_data_[11];
        velocity_data_[0] = 255;
        for(int i = 1; i < 11; i++)
        {
            velocity_data_[i] = 0;
        }
        
        double temp_x = msg->linear.x;
        double temp_y = msg->linear.y;
        double temp_z = msg->angular.z;

        if(temp_x < 0)
        {
            velocity_data_[1] = 1;
            temp_x = -temp_x;
        }
        if(temp_y < 0)
        {
            velocity_data_[4] = 1;
            temp_y = -temp_y;
        }
        if(temp_z < 0)
        {
            velocity_data_[7] = 1;
            temp_z = -temp_z;
        }

        velocity_data_[2] = static_cast<uint8_t>(temp_x);
        velocity_data_[3] = static_cast<uint8_t>((temp_x * 100) - velocity_data_[2] * 100);
        velocity_data_[5] = static_cast<uint8_t>(temp_y);
        velocity_data_[6] = static_cast<uint8_t>((temp_y * 100) - velocity_data_[5] * 100);
        velocity_data_[8] = static_cast<uint8_t>(temp_z);
        velocity_data_[9] = static_cast<uint8_t>((temp_z * 100) - velocity_data_[8] * 100);

        for(int i = 0; i<10; i++){
            velocity_data_[10] += velocity_data_[i];
        }

        ser.write(velocity_data_, 11);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    serial::Serial ser;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    std::vector<uint8_t> packet_buffer_;
    std::shared_ptr<nav_msgs::msg::Odometry> odom_msg_;
    std::shared_ptr<sensor_msgs::msg::Imu> imu_msg_;
    std::shared_ptr<sensor_msgs::msg::LaserScan> scan_msg_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

    size_t accumulated_points_;
    

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialNode>());
    rclcpp::shutdown();
    return 0;
}


