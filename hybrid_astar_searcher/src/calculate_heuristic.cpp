/**
 * @file calculate_heuristic.cpp
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hybrid_astar_searcher/calculate_heuristic.h"

using namespace std;

namespace planning {

    // 构造函数，初始化GridSearch对象
    GridSearch::GridSearch(grid_map::GridMap map) {
        map_ = map; // 将传入的地图赋值给成员变量map_
        resolution_ = map.getResolution(); // 获取地图的分辨率
        x_min_ = map.getStartIndex()(0) * resolution_ - 25; // 计算地图的最小x坐标
        x_max_ = x_min_ + map.getLength().x(); // 计算地图的最大x坐标
        y_min_ = map.getStartIndex()(1) * resolution_ -25; // 计算地图的最小y坐标
        y_max_ = y_min_ + map.getLength().y(); // 计算地图的最大y坐标
        
        map_size_x_ = map.getSize()(0); // 获取地图的x方向大小
        map_size_y_ = map.getSize()(1); // 获取地图的y方向大小
        
        cout << "x_min and x_max:" << x_min_ << " " << x_max_ << endl; // 输出x方向的最小和最大坐标
        cout << "y_min and y_max:" << y_min_ << " " << y_max_ << endl; // 输出y方向的最小和最大坐标

        cout << "map_size_x_" << map_size_x_ << endl; // 输出地图的x方向大小
        cout << "getStartIndex()(0)" << map.getStartIndex()(0) << endl; // 输出地图的起始索引的x值


    }

    /**
     * @brief 通过A星的距离作为启发式函数，一般不用，因为每次扩展节点都用一次A星累积起来太多了
     * 
     * @param start_pos 起始位置
     * @param goal_pos 目标位置
     * @return true 如果找到路径
     * @return false 如果未找到路径
     */
    bool GridSearch::calculateHeuByAstar(Vec2d start_pos, Vec2d goal_pos) {
        cout << "A star search" << endl; // 输出提示信息
        priority_queue<pair<int, double>, vector<pair<int, double>>, cmp> open_pq; // 定义优先队列用于存储开放列表中的节点
        unordered_map<int, shared_ptr<Node2d>> open_set; // 定义开放集合，存储已扩展但未处理的节点
        unordered_map<int, shared_ptr<Node2d>> close_set; // 定义关闭集合，存储已处理的节点

        Vec2i start_idx = getIndexFromPosition(start_pos); // 将起始位置转换为索引
        Vec2i goal_idx = getIndexFromPosition(goal_pos); // 将目标位置转换为索引
        //cout << "start_idx:" << start_idx << endl;

        shared_ptr<Node2d> start_node = make_shared<Node2d>(start_idx, map_size_x_); // 创建起始节点
        shared_ptr<Node2d> goal_node = make_shared<Node2d>(goal_idx, map_size_x_); // 创建目标节点

        open_pq.emplace(start_node->getIndex(), start_node->getF()); // 将起始节点加入优先队列
        open_set.emplace(start_node->getIndex(), start_node); // 将起始节点加入开放集合

        cout << "即将进入循环" << endl; // 输出提示信息
        int explored_node_num = 0; // 初始化探索节点数
        while (!open_pq.empty()) { // 当开放列表不为空时，继续搜索
            int cur_index = open_pq.top().first; // 获取优先队列中F值最小的节点的索引
            open_pq.pop(); // 弹出该节点
            shared_ptr<Node2d> cur_node = open_set[cur_index]; // 获取当前节点

            if (cur_node->getIndex() == goal_node->getIndex()) { // 如果当前节点是目标节点
                final_node_ = cur_node; // 设置最终节点
                break; // 跳出循环
            }
            

            close_set.emplace(cur_index, cur_node); // 将当前节点加入关闭集合
            vector<shared_ptr<Node2d>> neighbor_nodes = move(getNeighborNodes(cur_node)); // 获取当前节点的邻居节点
            //cout << "得到邻居节点" << endl;
            for (auto &neighbor_node : neighbor_nodes) { // 遍历邻居节点

                //检测是否在地图以内
                if (!isInGridMap(neighbor_node)) { // 如果邻居节点不在地图范围内
                    continue; // 跳过该节点
                }
                //cout << "在地图以内" << endl;

                //是否在障碍物上
                if (isOnObstacle(neighbor_node)) { // 如果邻居节点在障碍物上
                    continue; // 跳过该节点
                }
                //cout << "不在障碍物上" << endl;

                //是否在close set 里
                if (close_set.find(neighbor_node->getIndex()) != close_set.end()) { // 如果邻居节点已经在关闭集合中
                    continue; // 跳过该节点
                }

                if (open_set.find(neighbor_node->getIndex()) == open_set.end()) { // 如果邻居节点不在开放集合中
                
                    ++explored_node_num; // 增加探索节点数
                    double heuristic = 
                    EuclidDistance(neighbor_node->getPosXInMap(),neighbor_node->getPosYInMap(),
                                goal_node->getPosXInMap(), goal_node->getPosYInMap()); // 计算启发式值
                    neighbor_node->setH(heuristic); // 设置邻居节点的启发式值
                    neighbor_node->setParent(cur_node); // 设置邻居节点的父节点
                    open_set.emplace(neighbor_node->getIndex(), neighbor_node); // 将邻居节点加入开放集合
                    open_pq.emplace(neighbor_node->getIndex(), neighbor_node->getF()); // 将邻居节点加入优先队列
                } else { // 如果邻居节点已经在开放集合中
                    if (open_set[neighbor_node->getIndex()]->getG() > neighbor_node->getG()) { // 如果邻居节点的G值更小
                        open_set[neighbor_node->getIndex()]->setG(neighbor_node->getG()); // 更新邻居节点的G值
                        open_set[neighbor_node->getIndex()]->setParent(cur_node); // 更新邻居节点的父节点
                    }
                }

            }

        }

        if (final_node_ == nullptr) { // 如果没有找到路径
            cout << "没找到一条路径" << endl; // 输出提示信息
            return false; // 返回false
        }
        heu_astar_ = final_node_->getG() * resolution_; // 计算A星启发式值
        cout << "Astar Heuristic:" << heu_astar_ << endl; // 输出A星启发式值
        cout << "探索的节点数是：" << explored_node_num << endl; // 输出探索的节点数
        return true; // 返回true

    }
    /**
     * @brief 先生成一个到终点的距离表，每次扩展都直接查表即可，但是构建这个表也要花很多时间
     * 
     * @param goal_pos 目标位置
     * @return true 如果生成成功
     * @return false 如果生成失败
     */
    bool GridSearch::generateDpMap(Vec2d goal_pos) {
        priority_queue<pair<int, double>, vector<pair<int, double>>, cmp> open_pq; // 定义优先队列用于存储开放列表中的节点
        unordered_map<int, shared_ptr<Node2d>> open_set; // 定义开放集合，存储已扩展但未处理的节点

        dp_map_.clear(); // 清空距离表

        Vec2i goal_idx = getIndexFromPosition(goal_pos); // 将目标位置转换为索引

        shared_ptr<Node2d> goal_node = make_shared<Node2d>(goal_idx, map_size_x_); // 创建目标节点

        open_set.emplace(goal_node->getIndex(), goal_node); // 将目标节点加入开放集合
        open_pq.emplace(goal_node->getIndex(), goal_node->getG()); // 将目标节点加入优先队列

        int explored_node_num = 0; // 初始化探索节点数
        while (!open_pq.empty()) { // 当开放列表不为空时，继续搜索
            int cur_index = open_pq.top().first; // 获取优先队列中G值最小的节点的索引
            open_pq.pop(); // 弹出该节点
            shared_ptr<Node2d> cur_node = open_set[cur_index]; // 获取当前节点

            dp_map_.emplace(cur_index, cur_node); // 将当前节点加入距离表

            vector<shared_ptr<Node2d>> neighbor_nodes = move(getNeighborNodes(cur_node)); // 获取当前节点的邻居节点
            //cout << "得到邻居节点" << endl;
            for (auto &neighbor_node : neighbor_nodes) { // 遍历邻居节点

                //检测是否在地图以内
                if (!isInGridMap(neighbor_node)) { // 如果邻居节点不在地图范围内
                    continue; // 跳过该节点
                }
                //cout << "在地图以内" << endl;

                //是否在障碍物上
                if (isOnObstacle(neighbor_node)) { // 如果邻居节点在障碍物上
                    continue; // 跳过该节点
                }
                //cout << "不在障碍物上" << endl;

                if (dp_map_.find(neighbor_node->getIndex()) != dp_map_.end()) { // 如果邻居节点已经在距离表中
                    continue; // 跳过该节点
                }

                if (open_set.find(neighbor_node->getIndex()) == open_set.end()) { // 如果邻居节点不在开放集合中
                    ++explored_node_num; // 增加探索节点数
                    neighbor_node->setParent(cur_node); // 设置邻居节点的父节点
                    open_set.emplace(neighbor_node->getIndex(), neighbor_node); // 将邻居节点加入开放集合
                    open_pq.emplace(neighbor_node->getIndex(), neighbor_node->getG()); // 将邻居节点加入优先队列
                } else { // 如果邻居节点已经在开放集合中
                    if (open_set[neighbor_node->getIndex()]->getG() > neighbor_node->getG()) { // 如果邻居节点的G值更小
                        open_set[neighbor_node->getIndex()]->setG(neighbor_node->getG()); // 更新邻居节点的G值
                        open_set[neighbor_node->getIndex()]->setParent(cur_node); // 更新邻居节点的父节点
                    }
                }

            }

        }

        cout << "搜索的节点数是：" << explored_node_num << endl; // 输出搜索的节点数
        return true; // 返回true
    }
    /**
     * @brief 查询到终点的距离
     * 
     * @param start_pos 起始位置
     * @return double 到终点的距离，如果未找到则返回无穷大
     */
    double GridSearch::lookupInDpMap(Vec2d start_pos) {
        Vec2i start_idx = getIndexFromPosition(start_pos); // 将起始位置转换为索引
        shared_ptr<Node2d> start_node = make_shared<Node2d>(start_idx, map_size_x_); // 创建起始节点

        if (dp_map_.find(start_node->getIndex()) != dp_map_.end()) { // 如果起始节点在距离表中
            return dp_map_[start_node->getIndex()]->getG() * resolution_; // 返回起始节点到终点的距离
        } else {
            return numeric_limits<double>::infinity(); // 返回无穷大
        }
    }

    /**
     * @brief 获取当前节点的邻居节点
     * 
     * @param cur_node 当前节点
     * @return vector<shared_ptr<Node2d>> 邻居节点列表
     */
    vector<shared_ptr<Node2d>> GridSearch::getNeighborNodes(shared_ptr<Node2d> cur_node) {
        int cur_node_x = cur_node->getPosXInMap(); // 获取当前节点的x坐标
        int cur_node_y = cur_node->getPosYInMap(); // 获取当前节点的y坐标
        int cur_node_g = cur_node->getG(); // 获取当前节点的G值
        double diagonal_distance = sqrt(2.0); // 对角线距离
        vector<shared_ptr<Node2d>> neighbors; // 定义邻居节点列表
        for (int i = -1; i <= 1; ++i) { // 遍历x方向的邻居
            for (int j = -1; j <= 1; ++j) { // 遍历y方向的邻居
                Vec2i neighbor_idx{cur_node_x + i, cur_node_y + j}; // 计算邻居节点的索引
                shared_ptr<Node2d> neightbor = make_shared<Node2d>(neighbor_idx, map_size_x_); // 创建邻居节点
                if (i ==0 && j == 0) continue; // 如果邻居节点是当前节点，跳过
                if (sqrt(i * i + j * j) > 1) { // 如果邻居节点在对角线上
                    neightbor->setG(cur_node_g + diagonal_distance); // 设置邻居节点的G值为当前节点的G值加上对角线距离
                } else {
                    neightbor->setG(cur_node_g + 1.0); // 设置邻居节点的G值为当前节点的G值加上1
                }
                neighbors.emplace_back(neightbor); // 将邻居节点加入邻居节点列表
            }
        }
        return neighbors; // 返回邻居节点列表
    }

    /**
     * @brief 判断节点是否在地图范围内
     * 
     * @param node 节点
     * @return true 如果在地图范围内
     * @return false 如果不在地图范围内
     */
    bool GridSearch::isInGridMap(shared_ptr<Node2d> node) {
        int index_x = node->getPosXInMap(); // 获取节点的x坐标
        int index_y = node->getPosYInMap(); // 获取节点的y坐标
        // cout << "到这了吗" << endl;
        // cout << "index_x: " << index_x << endl;
        // cout << "index_y: " << index_y << endl;
        // cout << "map_size_y_" << map_size_y_ << endl;

        if (index_x < 0 || index_x >= map_size_x_ || index_y < 0 || index_y >= map_size_y_) { // 如果节点不在地图范围内
            return false; // 返回false
        }
        
        return true; // 返回true
    }

    /**
     * @brief 判断节点是否在障碍物上
     * 
     * @param node 节点
     * @return true 如果在障碍物上
     * @return false 如果不在障碍物上
     */
    bool GridSearch::isOnObstacle(shared_ptr<Node2d> node) {
        int index_x = node->getPosXInMap(); // 获取节点的x坐标
        int index_y = node->getPosYInMap(); // 获取节点的y坐标
        Vec2d pos = getPositionFromIndex({index_x, index_y}); // 将节点的索引转换为位置
        // cout << "index_x: " << pos(0) << endl;
        // cout << "index_y: " << pos(1) << endl;
        if (map_.atPosition("elevation", pos) > 0) { // 如果节点在障碍物上
            return true; // 返回true
        }
        return false; // 返回false
    }

    /**
     * @brief 从终点回溯整条路径
     * 
     * @return vector<Vec2d> 路径列表
     */
    vector<Vec2d> GridSearch::getAstartPath() {
        // 获取最终节点
        shared_ptr<Node2d> cur_node = final_node_;
        // 创建一个用于存储路径节点的向量
        vector<shared_ptr<Node2d>> vec;
        // 创建一个用于存储最终路径的向量
        vector<Vec2d> res;
        // 输出调试信息：开始回溯
        //cout << "回溯 " << endl;

        // 当当前节点有父节点时，继续回溯
        while (cur_node->getParentNode() != nullptr) {
            // 将当前节点添加到路径向量中
            vec.emplace_back(cur_node);
            // 更新当前节点为其父节点
            cur_node = cur_node->getParentNode();
        }
        // 反转路径向量，使其顺序正确
        reverse(vec.begin(), vec.end());
        // 输出调试信息：路径向量的大小
        //cout << "vec 大小：" << vec.size() << endl;

        // 遍历路径向量中的每个节点
        for (auto &node : vec) {
            // 将节点的地图位置转换为实际位置，并添加到最终路径中
            res.push_back({node->getPosXInMap() * resolution_ + x_min_,
                        node->getPosYInMap() * resolution_ + y_min_ });
            // 输出调试信息：当前节点的X坐标
            //cout << "cur_node->getPosXInMap():" << node->getPosXInMap() << endl;
        }
        // 返回最终路径
        return res;
    }

    // 计算两点之间的欧几里得距离
    double GridSearch::EuclidDistance(const double x1, const double y1,
                                    const double x2, const double y2) {
        // 返回两点之间的欧几里得距离
        return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    // 根据位置获取其在网格中的索引
    Vec2i GridSearch::getIndexFromPosition(Vec2d position) {
        // 创建一个用于存储索引的向量
        Vec2i index;
        // 计算X轴的索引
        index(0) = static_cast<int>((position(0) - x_min_) / resolution_);
        // 计算Y轴的索引
        index(1) = static_cast<int>((position(1) - y_min_) / resolution_);
        // 返回索引
        return index;
    }

    // 根据网格中的索引获取其对应的位置
    Vec2d GridSearch::getPositionFromIndex(Vec2i index) {
        // 创建一个用于存储位置的向量
        Vec2d pos;
        // 计算X轴的位置
        pos(0) = x_min_ + (index(0) + 0.5) * resolution_;
        // 计算Y轴的位置
        pos(1) = y_min_ + (index(1) + 0.5) * resolution_;
        // 返回位置
        return pos;
    }
} // namespace planning

