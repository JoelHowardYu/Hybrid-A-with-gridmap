/**
 * @file hybrid_astar.h
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-08
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

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>

#include "hybrid_astar_searcher/type_defs.h" 
#include "hybrid_astar_searcher/calculate_heuristic.h" 
#include "hybrid_astar_searcher/visualize.h"
#include "hybrid_astar_searcher/ReedsSheppPath.h"
#include "hybrid_astar_searcher/node3d.h"

// 命名空间别名，方便使用
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace planning {

/**
 * @brief 存储混合A星结果的类型
 * 
 */
struct HybridAstarResult {
    std::vector<double> x; // 路径点的x坐标
    std::vector<double> y; // 路径点的y坐标
    std::vector<double> theta; // 路径点的方向角
};

class HybridAstar
{
private:
    // 车辆参数
    double length_ = 4.933; // 车辆长度
    double width_ = 2.11; // 车辆宽度
    double min_turn_radius_ = 5.0538; // 最小转弯半径
    double max_steer_angle_ = 27; // 最大转向角
    double max_steer_angle_rate_ = 8.552; // 最大转向角速度
    double steer_ratio_ = 16; // 转向比
    double wheel_base_ = 2.845; // 轴距
    double front_edge_to_center_ = 3.89;  // 前边缘到中心的距离（后轴中心）
    double back_edge_to_center_ = 1.043; // 后边缘到中心的距离
    double left_edge_to_center_ = 1.055; // 左边缘到中心的距离
    double right_edge_to_center_ = 1.055; // 右边缘到中心的距离
   
    // 搜索参数
    double next_node_num_ = 6; // 每个节点生成的下一个节点数量
    double step_size_ = 0.7; // 步长

    // 地图参数
    double x_min_, x_max_, y_min_, y_max_, xy_resolution_, theta_resolution_; // 地图边界和分辨率
    int map_size_x_, map_size_y_; // 地图尺寸
    grid_map::GridMap map_; // 地图对象
    Visualize vis; // 可视化对象

    // 惩罚参数
    double traj_forward_penalty_ = 1.0; // 前进惩罚
    double traj_back_penalty_ = 10.0; // 后退惩罚
    double traj_gear_switch_penalty_ = 1.0; // 换挡惩罚
    double traj_steer_penalty_ = 0.0; // 转向惩罚
    double traj_steer_change_penalty_ = 0; // 转向变化惩罚

    // 节点指针
    std::shared_ptr<Node3d> start_node_; // 起始节点
    std::shared_ptr<Node3d> goal_node_; // 目标节点
    std::shared_ptr<Node3d> final_node_; // 最终节点

    // 优先队列比较函数
    struct cmp {
        bool operator() (const std::pair<int, double>& l, 
                         const std::pair<int, double>& r) {
            return l.second > r.second; // 比较两个节点的代价，返回代价较小的节点
        }
    };

    // 优先队列和集合
    std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, cmp> 
        open_pq_; // 开放列表优先队列
    std::unordered_map<int, std::shared_ptr<Node3d>> open_set_; // 开放列表
    std::unordered_map<int, std::shared_ptr<Node3d>> close_set_; // 关闭列表

    // 智能指针
    std::unique_ptr<GridSearch> grid_search_ptr_; // 网格搜索指针
    std::shared_ptr<ReedShepp> reed_shepp_generator_; // ReedShepp路径生成器指针

public:
    // 构造函数和析构函数
    HybridAstar(grid_map::GridMap map);
    ~HybridAstar();

    // 主要规划函数
    bool plan(Vec3d start_pose, Vec3d goal_pose, HybridAstarResult &result);

    // 辅助函数
    Vec3i getIndexFromPose(Vec3d pose); // 从姿态获取索引
    bool AnalyticExpansion(std::shared_ptr<Node3d> cur_node); // 分析扩展
    void mod2Pi(double &angle); // 角度归一化到[0, 2π)
    std::vector<Vec2d> calculateCarBoundingBox(Vec3d pose); // 计算车辆包围盒
    bool isLinecollision(double x0, double y0, double x1, double y1); // 检查线段是否碰撞
    bool validityCheck(std::shared_ptr<Node3d> node); // 检查节点是否有效
    bool isInMap(double x, double y); // 检查点是否在地图内
    bool isStateValid2(const ob::SpaceInformation *si, const ob::State *state); // 检查状态是否有效
    std::shared_ptr<Node3d> nextNodeGenerator(std::shared_ptr<Node3d> cur_node, int next_node_idx); // 生成下一个节点

    // 代价计算函数
    double TrajCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node); // 轨迹代价
    double calculateNodeCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node); // 节点代价

    // 结果获取函数
    bool getHybridAstarResult(HybridAstarResult &result); // 获取混合A星结果
};

} //namespace planning