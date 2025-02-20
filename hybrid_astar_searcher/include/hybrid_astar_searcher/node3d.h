/**
 * @file node3d.h
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>
#include <chrono>

#include "hybrid_astar_searcher/type_defs.h" 

namespace planning {

/**
 * @brief 混合A星搜索用的到节点
 * 
 */
class Node3d
{
private:
    Vec3d pose_;  // 节点的姿态，包含x, y, theta三个值

    std::vector<double> traversed_x_, traversed_y_, traversed_theta_;  // 记录路径中经过的x, y, theta值

    int index_;  // 节点的索引
    double traj_cost_, heuristic_cost_;  // 路径代价和启发式代价
    int step_size_ = 1;  // 步长
    double steering_ = 0;  // 转向角度
    std::shared_ptr<Node3d> parent_;  // 父节点指针
    bool direction_ = true;  // 方向，true表示前进

public:
    /**
     * @brief 构造函数，初始化节点
     * @param pose 节点的初始姿态
     */
    Node3d(Vec3d pose) {
        pose_ = pose;  // 设置节点的姿态
        traj_cost_ = 0.0;  // 初始化路径代价为0
        heuristic_cost_ = 0.0;  // 初始化启发式代价为0
        traversed_x_.push_back(pose_(0));  // 记录初始x值
        traversed_y_.push_back(pose_(1));  // 记录初始y值
        traversed_theta_.push_back(pose_(2));  // 记录初始theta值
        step_size_ = 1;  // 设置步长为1
        parent_ = nullptr;  // 初始化父节点指针为空
        index_ = 0;  // 初始化索引为0
    }

    /**
     * @brief 构造函数，初始化节点
     * @param traversed_x 记录的x值
     * @param traversed_y 记录的y值
     * @param traversed_theta 记录的theta值
     */
    Node3d(std::vector<double> traversed_x, std::vector<double> traversed_y, 
    std::vector<double> traversed_theta) {

        pose_(0) = traversed_x.back();  // 设置节点的x值为记录的最后一个x值
        pose_(1) = traversed_y.back();  // 设置节点的y值为记录的最后一个y值
        pose_(2) = traversed_theta.back();  // 设置节点的theta值为记录的最后一个theta值

        traversed_x_ = traversed_x;  // 设置记录的x值
        traversed_y_ = traversed_y;  // 设置记录的y值
        traversed_theta_ = traversed_theta;  // 设置记录的theta值

        step_size_ = traversed_x.size();  // 设置步长为记录的x值的数量
        traj_cost_ = 0.0;  // 初始化路径代价为0
        heuristic_cost_ = 0.0;  // 初始化启发式代价为0
        parent_ = nullptr;  // 初始化父节点指针为空
        index_ = 0;  // 初始化索引为0
        
    }

    /**
     * @brief 设置路径代价
     * @param traj_cost 路径代价
     */
    void setTrajCost(const double traj_cost) {traj_cost_ = traj_cost;}

    /**
     * @brief 设置启发式代价
     * @param heuristic_cost 启发式代价
     */
    void setHeuristicCost(const double heuristic_cost) {heuristic_cost_ = heuristic_cost;}

    /**
     * @brief 设置父节点
     * @param parent 父节点指针
     */
    void setParent(std::shared_ptr<Node3d> parent) {parent_ = parent;}

    /**
     * @brief 设置索引
     * @param pose_map 姿态映射
     * @param map_size_x 地图x方向大小
     * @param map_size_y 地图y方向大小
     */
    void setIndex(Vec3i pose_map, const int map_size_x, const int map_size_y) {
        index_ = static_cast<int>(pose_map(2) * map_size_x * map_size_y + 
                                  pose_map(1) * map_size_x + pose_map(0));
    }

    /**
     * @brief 设置方向
     * @param direction 方向，true表示前进
     */
    void setDirection(bool direction) {direction_ = direction;}

    /**
     * @brief 设置转向角度
     * @param steering 转向角度
     */
    void setSteering(double steering) {steering_ = steering;}

    /**
     * @brief 获取路径代价
     * @return 路径代价
     */
    double getTrajCost() {return traj_cost_;}

    /**
     * @brief 获取启发式代价
     * @return 启发式代价
     */
    double getHeuristicCost() {return heuristic_cost_;}

    /**
     * @brief 获取总代价
     * @return 总代价
     */
    double getCost() {return traj_cost_ + 1 * heuristic_cost_;}

    /**
     * @brief 获取父节点
     * @return 父节点指针
     */
    std::shared_ptr<Node3d> getParent() {return parent_;}

    /**
     * @brief 获取索引
     * @return 索引
     */
    int getIndex() {return index_;}

    /**
     * @brief 获取方向
     * @return 方向，true表示前进
     */
    bool getDirection() {return direction_;}

    /**
     * @brief 获取步长
     * @return 步长
     */
    int getStepSize() {return step_size_;}

    /**
     * @brief 获取转向角度
     * @return 转向角度
     */
    double getSteering() {return steering_;}

    /**
     * @brief 获取记录的x值
     * @return 记录的x值
     */
    std::vector<double> getXs() {return traversed_x_;}

    /**
     * @brief 获取记录的y值
     * @return 记录的y值
     */
    std::vector<double> getYs() {return traversed_y_;}

    /**
     * @brief 获取记录的theta值
     * @return 记录的theta值
     */
    std::vector<double> getThetas() {return traversed_theta_;}

    /**
     * @brief 获取x值
     * @return x值
     */
    double getX() {return pose_(0);}

    /**
     * @brief 获取y值
     * @return y值
     */
    double getY() {return pose_(1);}

    /**
     * @brief 获取theta值
     * @return theta值
     */
    double getTheta() {return pose_(2);}

};
}