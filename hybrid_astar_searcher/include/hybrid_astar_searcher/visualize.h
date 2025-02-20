/**
 * @file visualize.h
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

// 包含ROS相关的头文件
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 添加这个头文件
// 包含自定义类型定义的头文件
#include "type_defs.h"

// 定义命名空间planning
namespace planning
{

// 定义Visualize类，用于可视化相关操作
class Visualize : public rclcpp::Node
{
private:
    // ROS节点句柄，用于与ROS系统交互
    rclcpp::Node::SharedPtr node_handle_; 
    // 发布者，用于发布探索节点的位置、车辆包围盒和路径点
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr explored_nodes_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vehicle_boxes_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_points_pub_;
    // 存储探索节点的位置信息
    geometry_msgs::msg::PoseArray nodes_pose;
    // 存储车辆包围盒和路径点的可视化标记
    visualization_msgs::msg::MarkerArray vehicle_boxes, path_points;
public:
    // 构造函数，初始化发布者
    Visualize() : Node("visualize_node") {
        // 初始化发布者，发布探索节点的位置信息到"/visualize_nodes_pose"主题
        explored_nodes_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/visualize_nodes_pose", 10);
        // 初始化发布者，发布车辆包围盒信息到"/vehicle_boxes"主题
        vehicle_boxes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vehicle_boxes", 10);
        // 初始化发布者，发布路径点信息到"/path_points"主题
        path_points_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_points", 10);
    }

    // 清空存储的节点位置和车辆包围盒信息
    void clear() {
        nodes_pose.poses.clear();
        vehicle_boxes.markers.clear();
    }

    // 发布探索节点的位置信息
    void publishExploredNodes(Vec3d node_pose) {
        // 设置消息的帧ID和时间戳
        nodes_pose.header.frame_id = "map";
        nodes_pose.header.stamp = this->now();
        // 创建一个Pose对象，用于存储节点的位置和方向
        geometry_msgs::msg::Pose pose;
        // 设置节点的x和y坐标
        pose.position.x = node_pose(0);
        pose.position.y = node_pose(1);
        // 根据节点的yaw角创建四元数，并设置节点的方向
        tf2::Quaternion q;
        q.setRPY(0, 0, node_pose(2));
        pose.orientation = tf2::toMsg(q);  // 使用 tf2::toMsg 进行转换
        // 将节点的位置信息添加到nodes_pose中
        nodes_pose.poses.push_back(pose);
        // 发布节点位置信息
        explored_nodes_pub_->publish(nodes_pose);
    }
    
    // 发布车辆包围盒信息
    void publishVehicleBoxes(Vec3d node_pose, int i) {

        // 创建一个Marker对象，用于存储车辆包围盒的信息
        visualization_msgs::msg::Marker vehicle_box;
        // 如果i为0，设置action为3（表示删除标记）
        if (i == 0) {
            // vehicle_box.action = 3;
            vehicle_box.action = visualization_msgs::msg::Marker::DELETE;
        }
        // 设置消息的帧ID和时间戳
        vehicle_box.header.frame_id = "map";
        vehicle_box.header.stamp = this->now();
        // 设置标记的ID
        vehicle_box.id = i;
        // 设置标记的类型为立方体
        vehicle_box.type = visualization_msgs::msg::Marker::CUBE;
        // 设置立方体的尺寸
        vehicle_box.scale.x = 4.933;
        vehicle_box.scale.y = 2.11;
        vehicle_box.scale.z = 1;
        // 设置立方体的颜色和不透明度
        vehicle_box.color.a = 0.05;
        vehicle_box.color.r = 0;
        vehicle_box.color.b = 1;
        vehicle_box.color.g = 0;

        // 计算车辆包围盒的中心位置
        Vec2d center = {node_pose(0) + 1.45 * std::cos(node_pose(2)),
                    node_pose(1) + 1.45 * std::sin(node_pose(2))};
        // 设置立方体的位置
        vehicle_box.pose.position.x = center(0);
        vehicle_box.pose.position.y = center(1);
        // 设置立方体的方向
        tf2::Quaternion q;
        q.setRPY(0, 0, node_pose(2));
        vehicle_box.pose.orientation = tf2::toMsg(q);
        // 将车辆包围盒信息添加到vehicle_boxes中
        vehicle_boxes.markers.push_back(vehicle_box);
        // 发布车辆包围盒信息
        vehicle_boxes_pub_->publish(vehicle_boxes);
    }

    // 发布路径点信息
    void publishPathPoint(Vec2d position, int i) {
        // 创建一个Marker对象，用于存储路径点的信息
        visualization_msgs::msg::Marker path_point;

        // 如果i为0，设置action为3（表示删除标记）
        if (i == 0) {
            path_point.action = visualization_msgs::msg::Marker::DELETE;
        }

        // 设置消息的帧ID和时间戳
        path_point.header.frame_id = "map";
        path_point.header.stamp = this->now();
        // 设置标记的ID
        path_point.id = i;
        // 设置标记的类型为球体
        path_point.type = visualization_msgs::msg::Marker::SPHERE;
        // 设置球体的尺寸
        path_point.scale.x = 0.1;
        path_point.scale.y = 0.1;
        path_point.scale.z = 0.1;

        // 设置球体的颜色和不透明度
        path_point.color.a = 1;
        path_point.color.r = 1;
        path_point.color.g = 0;
        path_point.color.b = 0;

        // 设置球体的位置
        path_point.pose.position.x = position(0);
        path_point.pose.position.y = position(1);
        path_point.pose.position.z = 0.1;

        // 设置球体的方向
        path_point.pose.orientation.w = 1.0;

        // 将路径点信息添加到path_points中
        path_points.markers.push_back(path_point);

        // 发布路径点信息
        path_points_pub_->publish(path_points);

    }
};

} // namespace planning