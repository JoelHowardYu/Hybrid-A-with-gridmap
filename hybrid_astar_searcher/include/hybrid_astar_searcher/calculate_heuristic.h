/**
 * @file calculate_heuristic.h
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-06
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

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include "hybrid_astar_searcher/type_defs.h" 

namespace planning {

/**
 * @brief 用于A*搜索和构建离线的查询障碍物启发函数距离的表
 * 
 */
class Node2d
{
private:
    Vec2i pos_map_; // 节点在地图中的位置
    double g_, h_, f_; // g: 从起点到当前节点的代价, h: 从当前节点到目标节点的启发值, f: g + h
    int index_; // 节点在地图中的索引
    std::shared_ptr<Node2d> parent_; // 指向父节点的指针

public:
    
    Node2d(Vec2i pos_map, const int map_size_x) {
        pos_map_ = pos_map; // 设置节点在地图中的位置
        g_ = 0.0; // 初始化g值为0
        h_ = 0.0; // 初始化h值为0
        f_ = 0.0; // 初始化f值为0
        index_ = pos_map_(1) * map_size_x + pos_map_(0); // 计算节点在地图中的索引
        parent_ = nullptr; // 初始化父节点指针为空
    }

    void setG(const double g) {g_ = g; f_ = g_ + 1*h_;} // 设置g值，并更新f值
    void setH(const double h) {h_ = h; f_ = g_ + 1*h_;} // 设置h值，并更新f值
    void setF(const double f) {f_ = f;} // 直接设置f值
    void setParent(std::shared_ptr<Node2d> node) {parent_ = node;} // 设置父节点

    int getPosXInMap() {return pos_map_(0);} // 获取节点在地图中的x坐标
    int getPosYInMap() {return pos_map_(1);} // 获取节点在地图中的y坐标
    double getG() {return g_;} // 获取g值
    double getH() {return h_;} // 获取h值
    double getF() {return f_;} // 获取f值
    int getIndex() {return index_;} // 获取节点在地图中的索引
    std::shared_ptr<Node2d> getParentNode() {return parent_;} // 获取父节点指针

    
};

class GridSearch {
private:
    double x_min_, x_max_, y_min_, y_max_, resolution_; // 地图的边界和分辨率
    int map_size_x_, map_size_y_; // 地图的尺寸
    grid_map::GridMap map_; // 地图对象

    std::shared_ptr<Node2d> start_node_; // 起点节点
    std::shared_ptr<Node2d> goal_node_; // 目标节点
    std::shared_ptr<Node2d> final_node_; // 最终节点
    double heu_astar_; // A*算法的启发值

    struct cmp {
        bool operator() (const std::pair<int, double>& l, 
                         const std::pair<int, double>& r) {
            return l.second > r.second; // 用于优先队列的比较函数，按f值从小到大排序                
        }
    };
    std::unordered_map<int, std::shared_ptr<Node2d>> dp_map_; // 用于存储启发函数的离线查询表

    

public:
    GridSearch(grid_map::GridMap map); // 构造函数，初始化地图

    Vec2i getIndexFromPosition(Vec2d position); // 根据位置获取索引
    Vec2d getPositionFromIndex(Vec2i index); // 根据索引获取位置

    std::vector<std::shared_ptr<Node2d>> getNeighborNodes(std::shared_ptr<Node2d> cur_node); // 获取当前节点的邻居节点
    double EuclidDistance(const double x1, const double y1,
                          const double x2, const double y2); // 计算两点之间的欧几里得距离
    
    bool isInGridMap(std::shared_ptr<Node2d> node); // 判断节点是否在地图范围内
    bool isOnObstacle(std::shared_ptr<Node2d> node); // 判断节点是否在障碍物上

    std::vector<Vec2d> getAstartPath(); // 获取A*算法的路径

    bool calculateHeuByAstar(Vec2d start_pos, Vec2d goal_pos); // 使用A*算法计算启发值

    bool generateDpMap(Vec2d goal_pos); // 生成启发函数的离线查询表
    double lookupInDpMap(Vec2d start_pos); // 在离线查询表中查找启发值


};

} //namespace planning