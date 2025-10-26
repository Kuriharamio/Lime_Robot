#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/msg/string.hpp>
#include "wheeltec_mic_msg/msg/voice.hpp"
#include "wheeltec_mic_msg/msg/item_info.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <optional>
#include <tuple>
#include <sstream>

#define DISTANCE_THRESHOLD 0.3

enum Status
{
    NONE,
    VOICE_NAV,
    EYE_NAV,
    EYE_CLOSE,
    SPIN,
    STOP,
    BACK,
    AWAKE_NAV,
    CLOSE_TOWARD_TARGET
};

class CmdNav : public rclcpp::Node
{
public:
    CmdNav() : Node("cmd_nav_node"),
               is_navigating_(false),
               tf_buffer_(this->get_clock()),
               tf_listener_(tf_buffer_)
    {
        current_status_ = NONE;
        goal_distance_ = 0;

        visual_control_sub_ = this->create_subscription<std_msgs::msg::String>("visual_control", 10, std::bind(&CmdNav::visualControllCallback, this, std::placeholders::_1));

        voice_sub_ = this->create_subscription<wheeltec_mic_msg::msg::ItemInfo>("online", 10, std::bind(&CmdNav::voiceCallback, this, std::placeholders::_1));

        angle_sub_ = this->create_subscription<wheeltec_mic_msg::msg::Voice>("offline", 10, std::bind(&CmdNav::angleCallback, this, std::placeholders::_1));

        eye_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/object_xyz", 10, std::bind(&CmdNav::eyeCallback, this, std::placeholders::_1));

        get_pose_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&CmdNav::getCurrentPose, this));


        nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // pid_2d_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pid_2d", 10);

        target_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/target_xyz", 10);

        controller_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&CmdNav::controllerCallback, this));

        target_pub_ = this->create_publisher<std_msgs::msg::String>("/target_object_name", 10);
        
        reading_publisher_ = this->create_publisher<std_msgs::msg::String>("/read_aloud", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("point", 10);
        

        loadCoordinatesFromFile();
    }

    ~CmdNav()
    {
        for (int i = 0; i < 10; i++)
        {
            auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
            twist_msg->linear.x = 0;
            twist_msg->linear.y = 0;
            twist_msg->angular.z = 0.0;
            cmd_vel_publisher_->publish(*twist_msg);
        }

        RCLCPP_INFO(this->get_logger(), "Node Destroyed");
    }

private:
    Status current_status_;
    bool is_navigating_;

    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped cmd_pose_;
    geometry_msgs::msg::PoseStamped goal_pose;
    geometry_msgs::msg::PoseStamped target_pose;

    double goal_distance_;
    bool found_target_;

    double pos_goal_[3];
    std::string target_name_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr visual_control_sub_;
    rclcpp::Subscription<wheeltec_mic_msg::msg::ItemInfo>::SharedPtr voice_sub_;
    rclcpp::Subscription<wheeltec_mic_msg::msg::Voice>::SharedPtr angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr eye_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pid_2d_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reading_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr get_pose_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr target_timer_;

    tf2::Transform global_pose_;

    double roll_, pitch_, yaw_;
    double awake_angle;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_pub_;
    rclcpp::TimerBase::SharedPtr controller_timer_;

    int mode;

    std::map<std::string, std::tuple<double, double, double, double, double, double>> coordinates_list_;

    void publishMarker(geometry_msgs::msg::Point position, int marker_id)
    {
        visualization_msgs::msg::Marker marker_msg;
        marker_msg.header.frame_id = "map";
        marker_msg.header.stamp = this->now();
        marker_msg.ns = "points";
        marker_msg.id = marker_id;
        marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;
        marker_msg.pose.position = position;
        marker_msg.pose.orientation.w = 1.0;
        marker_msg.scale.x = 0.2;
        marker_msg.scale.y = 0.2;
        marker_msg.scale.z = 0.2;
        marker_msg.color.a = 1.0;
        if(marker_id==1){
            marker_msg.color.r = 1.0;
            marker_msg.color.g = 0.0;
            marker_msg.color.b = 0.0;
        }else{
            marker_msg.color.r = 0.0;
            marker_msg.color.g = 1.0;
            marker_msg.color.b = 0.0;
        }
        marker_msg.lifetime = rclcpp::Duration(0, 0);

        marker_pub_->publish(marker_msg);
    }
    void publishTarget()
    {
        if(found_target_){    
            geometry_msgs::msg::TransformStamped base_link_to_map;
            try
            {
                base_link_to_map = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "无法实现map到base_link的变换: %s", ex.what());
                return;
            }

            tf2::Transform tf_base_link = tf2::Transform(tf2::Quaternion(base_link_to_map.transform.rotation.x,
                                                                        base_link_to_map.transform.rotation.y,
                                                                        base_link_to_map.transform.rotation.z,
                                                                        base_link_to_map.transform.rotation.w));

            tf2::Quaternion rotation = tf_base_link.getRotation();
            rotation.normalize();
            tf_base_link.setRotation(rotation);

            double error_x = target_pose.pose.position.x - current_pose_.pose.position.x;
            double error_y = target_pose.pose.position.y - current_pose_.pose.position.y;


            tf2::Transform map_to_target;
            map_to_target.setOrigin(tf2::Vector3(error_x, error_y, 0));
            map_to_target.setRotation(tf2::Quaternion(0, 0, 0, 1));

            tf2::Transform local_target = tf_base_link * map_to_target;

            double x = local_target.getOrigin().x();
            double y = local_target.getOrigin().y();
            double z = local_target.getOrigin().z();

            std_msgs::msg::Float32MultiArray target_xyz;
            target_xyz.data.resize(3);

            // target_xyz.data[0] = -(x - 0.17);
            // target_xyz.data[1] = 0;
            // target_xyz.data[2] = pos_goal_[2] - 0.09;

            target_xyz.data[0] = 0.16;
            target_xyz.data[1] = 0.00;
            target_xyz.data[2] = 0.11;
            target_publisher_->publish(target_xyz);
                
            geometry_msgs::msg::Point p;
            p.x = target_pose.pose.position.x;
            p.y = target_pose.pose.position.y;
            p.z = target_pose.pose.position.z;
            publishMarker(p, 1);
            RCLCPP_INFO(this->get_logger(), "Target: %f, %f, %f", target_xyz.data[0], target_xyz.data[1], target_xyz.data[2]);
        }
    }
    // 将角度转换为四元数
    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) 
    {
        geometry_msgs::msg::Quaternion orientation;

        double half_yaw = yaw / 2.0; 
        orientation.x = 0.0; 
        orientation.y = 0.0; 
        orientation.z = sin(half_yaw); 
        orientation.w = cos(half_yaw); 

        return orientation;
    }
    // 将四元数转换为角度
    double calculateYaw(const geometry_msgs::msg::Quaternion orientation) 
    {
        double x = orientation.x;
        double y = orientation.y;
        double z = orientation.z;
        double w = orientation.w;

        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        return yaw; 
    }
    // 复制位置
    void CopyPose(int mode)
    {
        switch (mode)
        {
        case 1:
            goal_pose.header.frame_id = "map";
            goal_pose.header.stamp = this->now();
            goal_pose.pose.position.x = current_pose_.pose.position.x;
            goal_pose.pose.position.y = current_pose_.pose.position.y;
            goal_pose.pose.orientation = current_pose_.pose.orientation;
            break;
        case 2:
            cmd_pose_.header.frame_id = "map";
            cmd_pose_.header.stamp = this->now();
            cmd_pose_.pose.position.x = current_pose_.pose.position.x;
            cmd_pose_.pose.position.y = current_pose_.pose.position.y;
            cmd_pose_.pose.orientation = current_pose_.pose.orientation;
            break;
        case 3:
            goal_pose.header.frame_id = "map";
            goal_pose.header.stamp = this->now();
            goal_pose.pose.position.x = cmd_pose_.pose.position.x;
            goal_pose.pose.position.y = cmd_pose_.pose.position.y;
            goal_pose.pose.orientation = cmd_pose_.pose.orientation;
            break;
        default:
            break;
        }
        
    }
    // 存储坐标
    void saveCoordinateToFile(const std::string &placeName, double x, double y, double ori_x, double ori_y, double ori_z, double ori_w) 
    {
        std::ifstream infile("coordinates.txt");
        std::ofstream outfile("temp_coordinates.txt"); 
        bool found = false;

        // 检查是否存在, 如果存在则更新
        std::string line;
        if (infile.is_open()) {
            while (std::getline(infile, line)) {
                std::istringstream iss(line);
                std::string existingPlaceName;
                double existingX, existingY, existingW;
                if (iss >> existingPlaceName >> existingX >> existingY >> existingW) {
                    if (existingPlaceName == placeName) {
                        outfile << placeName << " " << x << " " << y << " " << ori_x << " " <<  ori_y << " " << ori_z << " " << ori_w  << " " << "\n"; // 替换
                        found = true;
                    } else {
                        outfile << line << "\n";
                    }
                }
            }
            infile.close();
        }

        if (!found) {
            outfile << placeName << " " << x << " " << y << " " << ori_x << " " <<  ori_y << " " << ori_z << " " << ori_w  << " " << "\n";
            RCLCPP_INFO(this->get_logger(), "Stored new coordinates for %s.", placeName.c_str());
            std_msgs::msg::String msg;
            msg.data = "已存储" + placeName + "的坐标";
            reading_publisher_->publish(msg);
        } else {
            RCLCPP_INFO(this->get_logger(), "Updated coordinates for %s.", placeName.c_str());
            std_msgs::msg::String msg;
            msg.data = "已更新" + placeName + "的坐标";
            reading_publisher_->publish(msg);
        }

        outfile.close();

        std::remove("coordinates.txt");
        std::rename("temp_coordinates.txt", "coordinates.txt");
    }
    // 加载坐标
    void loadCoordinatesFromFile() {
        std::ifstream infile("coordinates.txt");
        RCLCPP_INFO(this->get_logger(), "Loaded coordinates from file.");
        std_msgs::msg::String msg;
        msg.data = "正在加载本地坐标信息";
        reading_publisher_->publish(msg);
        std::string line;

        if (infile.is_open()) {
            while (std::getline(infile, line)) {
                std::istringstream iss(line);
                std::string placeName;
                double x, y, ori_x, ori_y, ori_z, ori_w;
                if (iss >> placeName >> x >> y >> ori_x >> ori_y >> ori_z >> ori_w) {
                    coordinates_list_[placeName] = std::make_tuple(x, y, ori_x, ori_y, ori_z, ori_w);
                }
            }
            infile.close();
        } else {
            std::cerr << "Unable to open file for reading.\n";
        }
    }
    // 获取坐标
    std::optional<std::tuple<double, double, double, double, double, double>> getCoordinate(const std::string &placeName) const
    {
        auto it = coordinates_list_.find(placeName);
        if (it != coordinates_list_.end())
        {
            return it->second;
        }
        return std::nullopt;
    }
    // 存储坐标
    void storeCoordinate(const std::string &placeName, double x, double y, double ori_x, double ori_y, double ori_z, double ori_w) {
        coordinates_list_[placeName] = std::make_tuple(x, y, ori_x, ori_y, ori_z, ori_w);
        saveCoordinateToFile(placeName, x, y, ori_x, ori_y, ori_z, ori_w); 
    }

    void visualControllCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data.c_str();
        if(command == "REACH"){
            RCLCPP_INFO(this->get_logger(), "REACH");
            std_msgs::msg::String msg;
            msg.data = "正在前往目标点";
            reading_publisher_->publish(msg);
            current_status_ = STOP;

        }else if(command == "BACK"){
            RCLCPP_INFO(this->get_logger(), "BACK");
            std_msgs::msg::String msg;
            msg.data = "正在返回起始点";
            reading_publisher_->publish(msg);
            current_status_ = BACK;

        }else if(command == "FINISH"){
            RCLCPP_INFO(this->get_logger(), "FINISH");
            std_msgs::msg::String msg;
            msg.data = "已完成任务";
            reading_publisher_->publish(msg);
            current_status_ = STOP;
        }
    }
    // 角度回调
    void angleCallback(const wheeltec_mic_msg::msg::Voice::SharedPtr msg)
    {
        awake_angle = -0.8889 * msg->awake_angle + 313.3333;
        RCLCPP_INFO(this->get_logger(), "awake_angle: %f", awake_angle);
        CopyPose(2);

        
        
        double yaw = calculateYaw(current_pose_.pose.orientation);
        yaw += awake_angle / 360.0f * 2 * M_PI;

        if(yaw >  M_PI){
            yaw -= 2 * M_PI;
        }else if(yaw < -M_PI){
            yaw += 2 * M_PI;
        }

        cmd_pose_.pose.orientation = yawToQuaternion(yaw);

        current_status_ = AWAKE_NAV;
    }
    // 语音模块回调
    void voiceCallback(const wheeltec_mic_msg::msg::ItemInfo::SharedPtr msg)
    {
        is_navigating_ = false;
        found_target_ = false;

        auto position = msg->position;
        auto orientation = msg->orientation;
        target_name_ = msg->name;
        // ros2 topic pub --once /online wheeltec_mic_msg/msg/ItemInfo "{name: "bottle", position: "客厅", orientation: ""}"
        // ros2 topic pub --once /online wheeltec_mic_msg/msg/ItemInfo "{name: "无", position: "无", orientation: "客厅"}"ros2 topic pub --once /object_xyz std_msgs/msg/Float32MultiArray '{data: [0.6, -0.6, 0]}'
        // 标定坐标
        if (target_name_ == "无" && position == "无" && orientation != "无")
        {
            storeCoordinate(orientation, current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.orientation.x, current_pose_.pose.orientation.y, current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);
            CopyPose(1);
            current_status_ = NONE;
        }
        else
        {
            if (target_name_ == "无"){
                mode = 0;
            }else{
                mode = 1;
                CopyPose(2);
            }
               
            auto coordinate = getCoordinate(position);
            if (coordinate)
            {
                auto [x, y, ori_x, ori_y, ori_z, ori_w] = *coordinate;
                std::cout << "Coordinates of PlaceA: (" << x << ", " << y << ", " << ori_x << ", " <<  ori_y << ", " << ori_z << ", " <<  ori_w <<")\n";
                goal_pose.header.stamp = this->get_clock()->now();
                goal_pose.header.frame_id = "map";

                goal_pose.pose.position.x = x;
                goal_pose.pose.position.y = y;
                goal_pose.pose.orientation.w = ori_w;
                goal_pose.pose.orientation.x = ori_x;
                goal_pose.pose.orientation.y = ori_y;
                goal_pose.pose.orientation.z = ori_z;

                current_status_ = VOICE_NAV;
            }
            else
            {
                std::cout << position << "not found.\n";
                current_status_ = NONE;
                CopyPose(1);

                std_msgs::msg::String msg;
                msg.data = "我没听说过" + position + "在哪噢，麻烦您带我过去认认地方呗";
                reading_publisher_->publish(msg);
            }
        }
    }
    // 视觉模块回调
    void eyeCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {

        if (!is_navigating_)
        {   
            int mode = msg->data[3];
            pos_goal_[0] = msg->data[0];
            pos_goal_[1] = msg->data[1];
            pos_goal_[2] = msg->data[2];



            goal_distance_ = std::hypot(pos_goal_[0], pos_goal_[1]);

            tf2::Transform robot_to_goal;

            if (goal_distance_  > DISTANCE_THRESHOLD)
            {
                robot_to_goal.setOrigin(tf2::Vector3(pos_goal_[0] - 0.2, pos_goal_[1], 0.0));
                // double angle = atan2(pos_goal_[1], pos_goal_[0]);
                // geometry_msgs::msg::Quaternion quat_goal = yawToQuaternion(angle);
                // robot_to_goal.setRotation(tf2::Quaternion(quat_goal.x, quat_goal.y, quat_goal.z, quat_goal.w));
            }else{
                robot_to_goal.setOrigin(tf2::Vector3(pos_goal_[0], pos_goal_[1], 0.0));
                // double angle = atan2(pos_goal_[1], pos_goal_[0]-0.3);
                // geometry_msgs::msg::Quaternion quat_goal = yawToQuaternion(angle);
                // robot_to_goal.setRotation(tf2::Quaternion(quat_goal.x, quat_goal.y, quat_goal.z, quat_goal.w));
            }
            
            robot_to_goal.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

            tf2::Transform global_pose = global_pose_;
            tf2::Transform global_goal = global_pose * robot_to_goal;

            goal_pose.header.frame_id = "map";
            goal_pose.pose.position.x = global_goal.getOrigin().x();
            goal_pose.pose.position.y = global_goal.getOrigin().y();
            goal_pose.pose.position.z = 0.0;
            goal_pose.pose.orientation = tf2::toMsg(global_goal.getRotation());


            robot_to_goal.setOrigin(tf2::Vector3(pos_goal_[0] , pos_goal_[1], 0.0));
             
            
            robot_to_goal.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

            global_pose = global_pose_;
            global_goal = global_pose * robot_to_goal;

            target_pose.header.frame_id = "map";
            target_pose.pose.position.x = global_goal.getOrigin().x();
            target_pose.pose.position.y = global_goal.getOrigin().y();
            target_pose.pose.position.z = pos_goal_[2];
            target_pose.pose.orientation = tf2::toMsg(global_goal.getRotation());


            if(mode == 0){
                current_status_ = STOP;
            }else if(mode == 1){
                found_target_ = true;
                RCLCPP_INFO(this->get_logger(), "发现目标，停止旋转……");

                std_msgs::msg::String msg;
                msg.data = "发现目标，停止旋转";
                reading_publisher_->publish(msg);

                //if (current_status_ == EYE_NAV || current_status_ == EYE_CLOSE || current_status_ == NONE || current_status_ == STOP  || current_status_ == SPIN)
                {
                    if (goal_distance_ > DISTANCE_THRESHOLD)
                    {
                        current_status_ = EYE_NAV;
                        // is_navigating_ = true;
                    }
                    else
                    {
                        current_status_ = EYE_CLOSE;
                        // is_navigating_ = true;
                    }
                }
            }
            
        }
    }
    // 获取当前位姿ros2 topic pub /target_xyz std_msgs/msg/Float32MultiArray "data: [0.033,0.005,0.005]"
    void getCurrentPose()
    {
        geometry_msgs::msg::TransformStamped odom_to_map;
        try
        {
            odom_to_map = tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "无法实现odom到map的变换: %s", ex.what());
            return;
        }

        geometry_msgs::msg::TransformStamped base_link_to_odom;
        try
        {
            base_link_to_odom = tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "无法实现base_link到odom的变换: %s", ex.what());
            return;
        }

        

        tf2::Transform tf_robot;
        tf2::fromMsg(base_link_to_odom.transform, tf_robot);

        tf2::Transform tf_map = tf2::Transform(tf2::Quaternion(odom_to_map.transform.rotation.x,
                                                               odom_to_map.transform.rotation.y,
                                                               odom_to_map.transform.rotation.z,
                                                               odom_to_map.transform.rotation.w),
                                               tf2::Vector3(odom_to_map.transform.translation.x,
                                                            odom_to_map.transform.translation.y,
                                                            odom_to_map.transform.translation.z));

        global_pose_ = tf_map * tf_robot;

        tf2::Matrix3x3(tf2::Quaternion(global_pose_.getRotation())).getRPY(roll_, pitch_, yaw_);

        current_pose_.header.frame_id = "map";
        // current_pose_.header.stamp = this->get_clock()->now();
        current_pose_.pose.position.x = global_pose_.getOrigin().x();
        current_pose_.pose.position.y = global_pose_.getOrigin().y();
        current_pose_.pose.position.z = 0.0;
        current_pose_.pose.orientation = tf2::toMsg(global_pose_.getRotation());

        // auto msg = std::make_shared<std_msgs::msg::Float32MultiArray>();
        // msg->data = {float(current_pose_.pose.position.x), float(current_pose_.pose.position.y)};
        // pid_2d_publisher_->publish(*msg);
    }
    // 发布目标
    void publishGoal(bool current = false)
    {   
        if (current)
        {
            CopyPose(1);
        }

        goal_pose.header.stamp = this->get_clock()->now();
        if (nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
            goal_msg.pose = goal_pose;

            auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            send_goal_options.result_callback = std::bind(&CmdNav::GoalResultCallback, this, std::placeholders::_1);

            RCLCPP_INFO(this->get_logger(), "发送目标到 x: %f, y: %f", goal_pose.pose.position.x, goal_pose.pose.position.y);
            nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
            is_navigating_ = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "导航服务不可访问");
        }
    }
    // 目标到达回调
    void GoalResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
    {   
        std_msgs::msg::String reading_msg;

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "顺利到达目标");
            reading_msg.data = "已到达目标点";
            reading_publisher_->publish(reading_msg);

            if (current_status_ == VOICE_NAV)
            {
                if (mode)
                {
                    current_status_ = SPIN;
                    is_navigating_ = false;
                    RCLCPP_INFO(this->get_logger(), "旋转以寻找目标……");
                    reading_msg.data = "旋转以寻找目标";
                    reading_publisher_->publish(reading_msg);

                    std_msgs::msg::String msg;
                    if(target_name_ == "瓶子"){
                        msg.data = "bottle";
                    }else if(target_name_ == "手机"){
                        msg.data = "cell phone";
                    }else if(target_name_ == "香蕉"){
                        msg.data = "banana";
                    }else if(target_name_ == "苹果"){
                        msg.data = "apple";
                    }else if(target_name_ == "勺子"){
                        msg.data = "spoon";
                    }else if(target_name_ == "橙子"){
                        msg.data = "orange";
                    }else{
                        msg.data = target_name_;
                    }
                    target_pub_->publish(msg);
                }
                else
                {
                    current_status_ = NONE;
                    is_navigating_ = false;
                }
            }else if (current_status_ == EYE_NAV){
                // current_status_ = CLOSE_TOWARD_TARGET;
                // publishTarget();
                double current = this->now().seconds();
                while(this->now().seconds() - current < 5){
                    // publishTarget();
                }
                while(this->now().seconds() - current < 10){
                    // publishTarget();
                }
                // publishTarget();
                current_status_ = BACK;
                is_navigating_ = false;
            }else if (current_status_ == BACK)
            {
                current_status_ = NONE;
                std_msgs::msg::String msg;
                msg.data = "person";
                target_pub_->publish(msg);
            }else if (current_status_ == AWAKE_NAV){
                current_status_ = NONE;
            }else if(current_status_ == SPIN){
                current_status_ = SPIN;
            }
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "目标被放弃");
            found_target_ = false;
            current_status_ = SPIN;
            is_navigating_ = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "目标被取消");
            found_target_ = false;
            current_status_ = SPIN;
            is_navigating_ = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "未知错误");
            found_target_ = false;
            current_status_ = SPIN;
            is_navigating_ = false;
            break;
        }
    }

    void spinToFindTarget(std::string status)
    {
        if (status == "start")
        {
            geometry_msgs::msg::Twist twist_msg;
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.z = 0.5;
            cmd_vel_publisher_->publish(twist_msg);
        }
        else if (status == "stop")
        {   
            geometry_msgs::msg::Twist twist_msg;
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.z = 0.0;
            cmd_vel_publisher_->publish(twist_msg);
            goal_pose.pose.orientation = current_pose_.pose.orientation;
            // publishGoal();
        }
    }

    void fineMoveToGoal()
    {
        static double prev_error_x = 0;
        static double prev_error_y = 0;
        static double prev_error_theta = 0;

        static double integral_x = 0;
        static double integral_y = 0;
        static double integral_theta = 0;

        double x_kp = 0.5;
        double x_ki = 0.01;
        double x_kd = 0.05;


        double y_kp = 3.5;
        double y_ki = 0.01;
        double y_kd = 0.05;


        double theta_kp = 3.0;
        double theta_ki = 0.01;
        double theta_kd = 0.1;

        static int flag = 0;
        if(current_status_ == CLOSE_TOWARD_TARGET && flag == 0){
            tf2::Transform robot_to_goal;

            robot_to_goal.setOrigin(tf2::Vector3(pos_goal_[0] - 0.2, pos_goal_[1], 0.0));
             
            robot_to_goal.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

            tf2::Transform global_pose = global_pose_;
            tf2::Transform global_goal = global_pose * robot_to_goal;

            goal_pose.header.frame_id = "map";
            goal_pose.pose.position.x = global_goal.getOrigin().x();
            goal_pose.pose.position.y = global_goal.getOrigin().y();
            goal_pose.pose.position.z = 0.0;
            goal_pose.pose.orientation = tf2::toMsg(global_goal.getRotation());
            flag = 1;
        }


        double error_x = (goal_pose.pose.position.x - current_pose_.pose.position.x);
        double error_y = (goal_pose.pose.position.y - current_pose_.pose.position.y);

        double cmd_yaw = (calculateYaw(cmd_pose_.pose.orientation));
        double current_yaw = calculateYaw(current_pose_.pose.orientation);
        double error_theta = cmd_yaw - current_yaw ;

        geometry_msgs::msg::TransformStamped map_to_base_link;
        try
        {
            map_to_base_link = tf_buffer_.lookupTransform("base_link", "map", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "无法实现map到base_link的变换: %s", ex.what());
            return;
        }

        tf2::Transform tf_base_link = tf2::Transform(tf2::Quaternion(map_to_base_link.transform.rotation.x,
                                                                     map_to_base_link.transform.rotation.y,
                                                                     map_to_base_link.transform.rotation.z,
                                                                     map_to_base_link.transform.rotation.w));

        tf2::Quaternion rotation = tf_base_link.getRotation();
        rotation.normalize();
        tf_base_link.setRotation(rotation);

        tf2::Transform map_to_error;
        map_to_error.setOrigin(tf2::Vector3(error_x, error_y, 0.0));
        map_to_error.setRotation(tf2::Quaternion(0, 0, 0, 1));

        tf2::Transform local_error = tf_base_link * map_to_error;

        error_x = local_error.getOrigin().x();
        error_y = local_error.getOrigin().y();

        
        if (fabs(error_x) > 0.15)
        {
            x_ki = 0;
        }
        else{
            x_ki = (0.15 - fabs(error_x) / 100.0) * 5;
        }

        if (fabs(error_y) > 0.1){
            y_ki = 0;
        }
        else{
            y_ki = (0.001 - fabs(error_y) / 100.0) * 15;
        }

        if (fabs(error_theta) > 0.15){
            theta_ki = 0;
        }
        else{
            theta_ki = (0.15 - fabs(error_theta) / 100.0);
        }

        double control_x = 0;
        double control_y = 0;
        double control_theta = 0;

        // 先转后直行
        if(current_status_ == EYE_CLOSE){
            // RCLCPP_INFO(this->get_logger(), "error_x: %f, error_y: %f", error_x, error_y);
            if (fabs(error_y) > 0.02)
            {
                double derivative_y = (error_y - prev_error_y) / 0.01;
                control_y = y_kp * error_y + y_ki * integral_y + y_kd * derivative_y;
                prev_error_y = error_y;
                integral_y += error_y;
                // RCLCPP_INFO(this->get_logger(), "speed_y: %f", control_y);
                if (fabs(control_y) > 0.8)
                {
                    control_y = control_y > 0 ? 0.8 : -0.8;
                }
                else if (fabs(control_y) < 0.1)
                {
                    control_y = 0;
                }
            }
            else if (fabs(error_x) > 0.02)
            {
                double derivative_x = (error_x - prev_error_x) / 0.01;
                control_x = x_kp * error_x + x_ki * integral_x + x_kd * derivative_x;
                prev_error_x = error_x;
                integral_x += error_x;
                // RCLCPP_INFO(this->get_logger(), "speed_x: %f", control_x);
                if (fabs(control_x) > 0.1)
                {
                    control_x = control_x > 0 ? 0.1 : -0.1;
                }
                else if (fabs(control_x) < 0.08)
                {
                    control_x = 0;
                }
            }
            else
            {
                control_x = 0;
                control_y = 0;
            }

            if (fabs(error_x) > 0.03 || fabs(error_y) > 0.03)
            {

                geometry_msgs::msg::Twist cmd_vel_msg;
                cmd_vel_msg.linear.x = control_x;
                cmd_vel_msg.linear.y = 0;
                cmd_vel_msg.angular.z = control_y;
                cmd_vel_publisher_->publish(cmd_vel_msg);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "已微调至目标");
                std_msgs::msg::String msg;
                msg.data = "已微调至目标";
                reading_publisher_->publish(msg);
                prev_error_x = 0;
                prev_error_y = 0;
                integral_x = 0;
                integral_y = 0;
                is_navigating_ = false;
                current_status_ = SPIN;
                RCLCPP_INFO(this->get_logger(), "停止中……");
                
            }
        }
        else if (current_status_ == AWAKE_NAV )
        {
            // RCLCPP_INFO(this->get_logger(), "error_theta: %f", error_theta);
            if (fabs(error_theta) > 0.05)
            {
                double derivative_theta = (error_theta - prev_error_theta) / 0.01;
                control_theta = theta_kp * error_theta + theta_ki * integral_theta + theta_kd * derivative_theta;
                prev_error_theta = error_theta;
                integral_theta += error_theta;
                // RCLCPP_INFO(this->get_logger(), "speed_theta: %f", control_theta);
                if (fabs(control_theta) > 1.0)
                {
                    control_theta = control_theta > 0 ? 1.0 : -1.0;
                }
                else if (fabs(control_theta) < 0.5)
                {
                    control_theta = 0;
                }

                geometry_msgs::msg::Twist cmd_vel_msg;
                cmd_vel_msg.linear.x = 0;
                cmd_vel_msg.linear.y = 0;
                cmd_vel_msg.angular.z = control_theta;
                cmd_vel_publisher_->publish(cmd_vel_msg);
            }
            else
            {
                control_theta = 0;

                RCLCPP_INFO(this->get_logger(), "已看向命令者");
                prev_error_x = 0;
                prev_error_y = 0;
                prev_error_theta = 0;

                integral_x = 0;
                integral_y = 0;
                integral_theta = 0;
                current_status_ = STOP;
                RCLCPP_INFO(this->get_logger(), "停止中……");
            }
        }
        else  if(current_status_ == CLOSE_TOWARD_TARGET){
            // RCLCPP_INFO(this->get_logger(), "error_x: %f", error_x);
           if (fabs(error_x) > 0.02)
            {
                double derivative_x = (error_x - prev_error_x) / 0.01;
                control_x = x_kp * error_x + x_ki * integral_x + x_kd * derivative_x;
                prev_error_x = error_x;
                integral_x += error_x;
                // RCLCPP_INFO(this->get_logger(), "speed_x: %f", control_x);
                if (fabs(control_x) > 0.1)
                {
                    control_x = control_x > 0 ? 0.1 : -0.1;
                }
                else if (fabs(control_x) < 0.08)
                {
                    control_x = 0;
                }
            }
            else
            {
                control_x = 0;
            }

            if (fabs(error_x) > 0.03)
            {
                geometry_msgs::msg::Twist cmd_vel_msg;
                cmd_vel_msg.linear.x = control_x;
                cmd_vel_msg.linear.y = 0;
                cmd_vel_msg.angular.z = 0;
                cmd_vel_publisher_->publish(cmd_vel_msg);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "已在导航基础上靠近目标");
                std_msgs::msg::String msg;
                msg.data = "已在导航基础上靠近目标";
                reading_publisher_->publish(msg);
                prev_error_x = 0;
                prev_error_y = 0;
                integral_x = 0;
                integral_y = 0;
                is_navigating_ = false;
                current_status_ = STOP;
                publishTarget();
                flag = 0;
                // RCLCPP_INFO(this->get_logger(), "停止中……");
                
            }
        }
        
    }

    void controllerCallback()
    {
        static int timer = 0;
        timer++;
        if (timer > 100)
        {
            timer = 0;
            // RCLCPP_INFO(this->get_logger(), "当前坐标: x:%f y:%f z:%f", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.orientation.z);
            // RCLCPP_INFO(this->get_logger(), "目标坐标: x:%f y:%f z:%f", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.orientation.z);
        }

        if (current_status_ == VOICE_NAV && !is_navigating_)
        {
            RCLCPP_INFO(this->get_logger(), "发布语音指令的导航目标……");
            publishGoal();
        }
        else if (current_status_ == BACK && !is_navigating_)
        {
            RCLCPP_INFO(this->get_logger(), "发布初始位置的目标……");
            CopyPose(3);
            publishGoal();
        }
        else if (current_status_ == AWAKE_NAV)
        {
            // RCLCPP_INFO(this->get_logger(), "看向发布命令者……");
            fineMoveToGoal();
        }
        else if (current_status_ == CLOSE_TOWARD_TARGET){
            static int flag =  0;
            if(!flag){
                RCLCPP_INFO(this->get_logger(), "初步导航完成，正在靠近目标");
                flag = 1;
            }
            fineMoveToGoal();
        }
        else if (current_status_ == EYE_NAV && !is_navigating_)
        {
            RCLCPP_INFO(this->get_logger(), "发布视觉指令的导航目标……");
            publishGoal();
        }
        else if (current_status_ == EYE_CLOSE)
        {
            RCLCPP_INFO(this->get_logger(), "微调到目标点……");
            fineMoveToGoal();
        }
        else if (current_status_ == SPIN)
        {
            if (!found_target_)
            {   
                
                static int timer_spin_ = 0;
                timer_spin_++;
                if (timer_spin_ < 100 * 4.5)
                {   spinToFindTarget("stop");
                }else{
                    if(timer_spin_ > 100 * 5){
                        timer_spin_ = 0;
                        RCLCPP_INFO(this->get_logger(), "未发现目标，转一圈……");
                    }
                    spinToFindTarget("start");
                    std_msgs::msg::String msg;
                    if(target_name_ == "瓶子"){
                        msg.data = "bottle";
                    }else if(target_name_ == "手机"){
                        msg.data = "cell phone";
                    }else if(target_name_ == "香蕉"){
                        msg.data = "banana";
                    }else if(target_name_ == "苹果"){
                        msg.data = "apple";
                    }else if(target_name_ == "勺子"){
                        msg.data = "spoon";
                    }else if(target_name_ == "橙子"){
                        msg.data = "orange";
                    }else{
                        msg.data = target_name_;
                    }
                    target_pub_->publish(msg);
                    found_target_ = false;
                }
            }
        }
        else if (current_status_ == STOP)
        {
            is_navigating_ = false;
            static int timer_stop_ = 0;
            timer_stop_++;
            if (timer_stop_ < 100 * 2)
            {
                spinToFindTarget("stop");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "已停止");
                timer_stop_ = 0;
                current_status_ = NONE;
            }
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdNav>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
