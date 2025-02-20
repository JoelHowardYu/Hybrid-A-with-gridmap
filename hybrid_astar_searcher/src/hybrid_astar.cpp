/**
 * @file hybrid_astar.cpp
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hybrid_astar_searcher/hybrid_astar.h"

using namespace std;

namespace planning
{

// 构造函数，初始化HybridAstar对象
HybridAstar::HybridAstar(grid_map::GridMap map) {
    map_ = map; // 保存地图
    xy_resolution_ = map.getResolution(); // 获取地图的分辨率
    theta_resolution_ = 0.1; // 设置角度分辨率
    x_min_ = map.getStartIndex()(0) * xy_resolution_ - 25; // 计算地图x轴最小值
    x_max_ = x_min_ + map.getLength().x(); // 计算地图x轴最大值
    y_min_ = map.getStartIndex()(1) * xy_resolution_ -25; // 计算地图y轴最小值
    y_max_ = y_min_ + map.getLength().y(); // 计算地图y轴最大值
    
    map_size_x_ = map.getSize()(0); // 获取地图x轴大小
    map_size_y_ = map.getSize()(1); // 获取地图y轴大小
    
    cout << "map_size_x_" << map_size_x_ << endl; // 输出地图x轴大小
    cout << "getStartIndex()(0)" << map.getStartIndex()(0) << endl; // 输出地图起始索引
    grid_search_ptr_ = make_unique<GridSearch>(map_); // 创建GridSearch对象
    reed_shepp_generator_ = std::make_shared<ReedShepp>(0.3, 0.7); // 创建ReedShepp对象
}

// 析构函数
HybridAstar::~HybridAstar() {}

/**
 * @brief 混合A星搜索
 * 
 * @param start_pose 起点姿态
 * @param goal_pose 终点姿态
 * @param result 结果
 * @return true 搜索成功
 * @return false 搜索失败
 */
bool HybridAstar::plan(Vec3d start_pose, Vec3d goal_pose, HybridAstarResult &result) {
    open_set_.clear(); // 清空开放集合
    close_set_.clear(); // 清空关闭集合
    open_pq_ = decltype(open_pq_)(); // 清空优先队列
    final_node_ = nullptr; // 清空最终节点
    auto hybrid_astar_start = chrono::high_resolution_clock::now(); // 记录开始时间
    
    // 将起点和终点转换为地图索引
    Vec3i start_pose_map = getIndexFromPose(start_pose);
    Vec3i goal_pose_map = getIndexFromPose(goal_pose);

    // 创建起点和终点节点
    start_node_.reset(new Node3d({start_pose(0)}, {start_pose(1)}, {start_pose(2)}));
    goal_node_.reset(new Node3d({goal_pose(0)}, {goal_pose(1)}, {goal_pose(2)}));

    // 检查起点是否合理
    if (!validityCheck(start_node_)) {
        cout << "起点不合理" << endl;
        return false;
    }

    // 检查终点是否合理
    if (!validityCheck(goal_node_)) {
        cout << "终点不合理" << endl;
        return false;
    }

    // 生成Heuristic表
    auto dp_map_start = chrono::high_resolution_clock::now();
    grid_search_ptr_->generateDpMap({goal_pose(0), goal_pose(1)});
    auto dp_map_end = chrono::high_resolution_clock::now();
    chrono::duration<double> dp_map_use_time = dp_map_end - dp_map_start;
    
    // 设置起点节点的索引
    start_node_->setIndex(start_pose_map, map_size_x_, map_size_y_);
    open_set_.emplace(start_node_->getIndex(), start_node_); // 将起点节点加入开放集合
    open_pq_.emplace(start_node_->getIndex(), start_node_->getCost()); // 将起点节点加入优先队列

    int explored_node_num = 0; // 探索的节点数量
    while (!open_pq_.empty()) { // 当优先队列不为空时
        int cur_index = open_pq_.top().first; // 获取当前节点的索引
        cout << "cur_index" << cur_index << endl;
        open_pq_.pop(); // 弹出当前节点

        shared_ptr<Node3d> cur_node = open_set_[cur_index]; // 获取当前节点
        
        // 尝试用RS曲线直接连接到终点
        if (AnalyticExpansion(cur_node)) {
            cout << "直接连接成功" << endl;
            break;
        }

        close_set_.emplace(cur_index, cur_node); // 将当前节点加入关闭集合

        // 扩展当前节点的所有可能的下一个节点
        for (int i = 0; i < next_node_num_; ++i) {
            shared_ptr<Node3d> next_node = nextNodeGenerator(cur_node, i);
            if (next_node == nullptr) { // 如果节点超出边界
                continue;
            }            

            // 判断下一个节点是否在关闭集合中
            Vec3i index111 = getIndexFromPose({next_node->getX(), next_node->getY(), next_node->getTheta()});
            next_node->setIndex(index111, map_size_x_, map_size_y_);
            if (close_set_.find(next_node->getIndex()) != close_set_.end()) {
                continue;
            }
            
            // 检查下一个节点是否合理
            if (!validityCheck(next_node)) {
                continue;
            }

            // 如果下一个节点不在开放集合中
            if (open_set_.find(next_node->getIndex()) == open_set_.end()) {
                explored_node_num++;
                
                calculateNodeCost(cur_node, next_node); // 计算下一个节点的代价
                
                open_pq_.emplace(next_node->getIndex(), next_node->getCost()); // 将下一个节点加入优先队列
                open_set_.emplace(next_node->getIndex(), next_node); // 将下一个节点加入开放集合
            }
        }
    }

    // 如果最终节点为空，表示搜索失败
    if (final_node_ == nullptr) {
        cout << "搜索失败" << endl;
        return false;
    }

    // 获取搜索结果
    if (!getHybridAstarResult(result)) {
        cout << "未得到解" << endl;
    }
    cout << "探索的节点个数：" << explored_node_num << endl;
    cout << "dp map use time:" << dp_map_use_time.count() * 1000 << "ms" << endl;
    auto hybrid_astar_end = chrono::high_resolution_clock::now();
    chrono::duration<double> hybrid_astar_use_time = hybrid_astar_end - hybrid_astar_start;
    cout << "hybrid astar use time:" << hybrid_astar_use_time.count() * 1000 << "ms" << endl;
    return true;
}

/**
 * @brief 获取混合A星搜索结果
 * 
 * @param result 结果
 * @return true 成功
 * @return false 失败
 */
bool HybridAstar::getHybridAstarResult(HybridAstarResult &result) {
    shared_ptr<Node3d> cur_node = final_node_;
    vector<double> res_x;
    vector<double> res_y;
    vector<double> res_theta;

    // 从最终节点回溯到起点
    while (cur_node->getParent() != nullptr) {
        vector<double> x = cur_node->getXs();
        vector<double> y = cur_node->getYs();
        vector<double> theta = cur_node->getThetas();
        reverse(x.begin(), x.end());
        reverse(y.begin(), y.end());
        reverse(theta.begin(), theta.end());

        x.pop_back();
        y.pop_back();
        theta.pop_back();

        res_x.insert(res_x.end(), x.begin(), x.end());
        res_y.insert(res_y.end(), y.begin(), y.end());
        res_theta.insert(res_theta.end(), theta.begin(), theta.end());
        cur_node = cur_node->getParent();
    }
    res_x.push_back(cur_node->getX());
    res_y.push_back(cur_node->getY());
    res_theta.push_back(cur_node->getTheta());
    reverse(res_x.begin(), res_x.end());
    reverse(res_y.begin(), res_y.end());
    reverse(res_theta.begin(), res_theta.end());

    result.x = res_x;
    result.y = res_y;
    result.theta = res_theta;
    return true;
}

/**
 * @brief 生成下一个节点
 * 
 * @param cur_node 当前节点
 * @param next_node_idx 下一个节点的索引
 * @return shared_ptr<Node3d> 下一个节点
 */
shared_ptr<Node3d> HybridAstar::nextNodeGenerator(shared_ptr<Node3d> cur_node, int next_node_idx) {
    double steering = 0.0;
    double traveled_distance = 0.0;
    
    double delta_steering = 2 * max_steer_angle_ / (next_node_num_ / 2 - 1);

    // 计算正向或反向移动的距离和转向角度
    if (next_node_idx < next_node_num_ / 2) {
        steering = -max_steer_angle_ + delta_steering * next_node_idx;
        traveled_distance = step_size_;
    } else {
        steering = -max_steer_angle_ + delta_steering * (next_node_idx - next_node_num_ / 2);
        traveled_distance = -step_size_;
    }

    double steering1 = steering / 180 * M_PI; 

    // 存储节点的中间点
    double arc = sqrt(2) * xy_resolution_; // 可以设置想走多长，这个是栅格对角
    arc = 2.1; // 可以设置想走多长
    vector<double> traversed_x;
    vector<double> traversed_y;
    vector<double> traversed_theta;
    double cur_x = cur_node->getX();
    double cur_y = cur_node->getY();
    double cur_theta = cur_node->getTheta();

    traversed_x.push_back(cur_x);  // 把上一个节点也加进去了
    traversed_y.push_back(cur_y);
    traversed_theta.push_back(cur_theta);

    for (int i = 0; i < arc / step_size_; ++i) {
        double next_x = cur_x + traveled_distance * cos(cur_theta);
        double next_y = cur_y + traveled_distance * sin(cur_theta);
        double next_theta = cur_theta + traveled_distance * tan(steering1) / wheel_base_;
        mod2Pi(next_theta);
        
        traversed_x.push_back(next_x);
        traversed_y.push_back(next_y);
        traversed_theta.push_back(next_theta);

        cur_x = next_x;
        cur_y = next_y;
        cur_theta = next_theta;
    }

    // 如果超出边界，返回nullptr
    if (traversed_x.back() < x_min_ || traversed_x.back() > x_max_ ||
        traversed_y.back() < y_min_ || traversed_y.back() > y_max_) {
        return nullptr;
    }
    
    shared_ptr<Node3d> next_node = make_shared<Node3d>(traversed_x, traversed_y, traversed_theta);
    next_node->setParent(cur_node);
    next_node->setDirection(traveled_distance > 0.0);
    next_node->setSteering(steering);
    return next_node;
}

/**
 * @brief 计算轨迹代价
 * 
 * @param cur_node 当前节点
 * @param next_node 下一个节点
 * @return double 轨迹代价
 */
double HybridAstar::TrajCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node) {
    double piecewise_cost = 0.0;

    if (next_node->getDirection()) {
        piecewise_cost += (next_node->getStepSize() - 1) * step_size_ * traj_forward_penalty_;
    } else {
        piecewise_cost += (next_node->getStepSize() - 1) * step_size_ * traj_back_penalty_;
    }

    if (cur_node->getDirection() != next_node->getDirection()) {
        piecewise_cost += traj_gear_switch_penalty_;
    }

    piecewise_cost += traj_steer_change_penalty_ * std::abs(
        next_node->getSteering() - cur_node->getSteering()
    );

    piecewise_cost += traj_steer_penalty_ * std::abs(next_node->getSteering());

    return piecewise_cost;
}

/**
 * @brief 计算节点代价
 * 
 * @param cur_node 当前节点
 * @param next_node 下一个节点
 * @return double 节点代价
 */
double HybridAstar::calculateNodeCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node) {
    next_node->setTrajCost(cur_node->getTrajCost() + TrajCost(cur_node, next_node));
    double holo_heu_cost = 0.0;
    holo_heu_cost = grid_search_ptr_->lookupInDpMap({next_node->getX(), next_node->getY()});
    cout << "holo heu cost:" << holo_heu_cost << endl;

    auto non_holo_heu_start = chrono::system_clock::now();
    ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>(3));
    ob::SE2StateSpace::StateType * rsStart = (ob::SE2StateSpace::StateType*)space->allocState();
    ob::SE2StateSpace::StateType* rsEnd = (ob::SE2StateSpace::StateType*)space->allocState();
    rsStart->setXY(next_node->getX(), next_node->getY());
    rsStart->setYaw(next_node->getTheta());
    rsEnd->setXY(goal_node_->getX(), goal_node_->getY());
    rsEnd->setYaw(goal_node_->getTheta());
    double RS_heu_cost = space->distance(rsStart, rsEnd);
    auto non_holo_heu_end = chrono::system_clock::now();
    chrono::duration<double> non_holo_use_time = non_holo_heu_end - non_holo_heu_start;
    cout << "nonholo use time:" << non_holo_use_time.count() * 1000 << "ms" << endl;

    cout << "RS heu cost: " << RS_heu_cost << endl;
    next_node->setHeuristicCost(max(RS_heu_cost, holo_heu_cost));
}

/**
 * @brief 计算车辆边界框
 * 
 * @param pose 车辆姿态
 * @return vector<Vec2d> 车辆边界框的顶点
 */
vector<Vec2d> HybridAstar::calculateCarBoundingBox(Vec3d pose) {
    vector<Vec2d> vertices;
    double shift_distance = length_ / 2 - back_edge_to_center_;
    Vec2d center = {pose(0) + shift_distance * std::cos(pose(2)),
                    pose(1) + shift_distance * std::sin(pose(2))};
    const double dx1 = std::cos(pose(2)) * length_ / 2;
    const double dy1 = std::sin(pose(2)) * length_ / 2;
    const double dx2 = std::sin(pose(2)) * width_ / 2;
    const double dy2 = -std::cos(pose(2)) * width_ / 2;
    vertices.emplace_back(center(0) + dx1 + dx2, center(1) + dy1 + dy2);
    vertices.emplace_back(center(0) + dx1 - dx2, center(1) + dy1 - dy2);
    vertices.emplace_back(center(0) - dx1 - dx2, center(1) - dy1 - dy2);
    vertices.emplace_back(center(0) - dx1 + dx2, center(1) - dy1 + dy2); //顺时针
    return vertices;
}

/**
 * @brief 查询一条线上的点是否在障碍物里
 * 
 * @param x0 
 * @param y0 
 * @param x1 
 * @param y1 
 * @return true 
 * @return false 
 */
bool HybridAstar::isLinecollision(double x0, double y0, double x1, double y1) {
    //int check_point_num = static_cast<int>(max(abs(x1 - x0),abs(y1 - y0)) / xy_resolution_) + 1;
    int check_point_num = static_cast<int>(max(abs(x1 - x0),abs(y1 - y0)) / 1) + 1;

    double delta_x = (x1 - x0) / check_point_num;
    double delta_y = (y1 - y0) / check_point_num;

    double cur_x = x0;
    double cur_y = y0;
    for (int i = 0; i < check_point_num; ++i) {
        if (!isInMap(cur_x, cur_y)) {
            return true;
        }

        Vec3i idx = getIndexFromPose({cur_x, cur_y, 0});
        if (map_.atPosition("elevation", {cur_x, cur_y}) > 0) {
            return true;
        }

        cur_x += delta_x;
        cur_y += delta_y;
    }
    return false;
}

/**
 * @brief 验证节点是否可行，需要得到车的轮廓，然后用上边的判断四条边是否穿过障碍物
 * 
 * @param node 
 * @return true 
 * @return false 
 */
bool HybridAstar::validityCheck(std::shared_ptr<Node3d> node) {
    
    int node_step_size = node->getStepSize();
    const auto traversed_x = node->getXs();
    const auto traversed_y = node->getYs();
    const auto traversed_theta = node->getThetas();

    int start_index = 0;
    if (node_step_size > 1) {  //不止一个节点，有中间节点
        start_index = 1;   //第一个是上一个节点的x y theta
    }

    //遍历节点集合里的点
    for (int i = start_index; i < node_step_size; ++i) {
        //有超出地图边界的，节点抛弃
        if (!isInMap(traversed_x[i], traversed_y[i])) {
            return false;
        }
        //生成车的四个顶点
        auto vertices = calculateCarBoundingBox({traversed_x[i], traversed_y[i], traversed_theta[i]});
        //遍历四条边是否碰撞
        for (int i = 0; i < vertices.size(); ++i) {
            if (isLinecollision(vertices[i].x(), vertices[i].y(), 
                                vertices[(i+1) % 4].x(), vertices[(i+1) % 4].y())){
                return false;                    
            }
        }
    }
    return true;
}

/**
 * @brief 将姿态转换为索引
 * 
 * @param pose 姿态
 * @return Vec3i 索引
 */
Vec3i HybridAstar::getIndexFromPose(Vec3d pose) {
    Vec3i index;
    index(0) = static_cast<int>((pose(0) - x_min_) / xy_resolution_);
    index(1) = static_cast<int>((pose(1) - y_min_) / xy_resolution_);
    mod2Pi(pose(2));
    index(2) = static_cast<int>((pose(2)) / theta_resolution_);
    return index;
}

/**
 * @brief 直接用RS曲线连接到终点
 * 
 * @param current_node 
 * @return true 
 * @return false 
 */
bool HybridAstar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
    std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
        std::make_shared<ReedSheppPath>();
    if (!reed_shepp_generator_->ShortestRSP(current_node, goal_node_,
                                            *reeds_shepp_to_check)) {
        return false;
    }

    std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_check->x, reeds_shepp_to_check->y, reeds_shepp_to_check->phi));
    
    if (!validityCheck(node)) {
        return false;
    }

    std::shared_ptr<Node3d> goal_node = std::shared_ptr<Node3d>(new Node3d(
    reeds_shepp_to_check->x, reeds_shepp_to_check->y, reeds_shepp_to_check->phi));
    goal_node->setParent(current_node);
    close_set_.emplace(goal_node->getIndex(), goal_node);
    final_node_ = goal_node;
    return true;
}

/**
 * @brief 将角度限制在[-π, π]之间
 * 
 * @param angle 角度
 */
void HybridAstar::mod2Pi(double &angle) {
    if (angle >  M_PI) {
        angle -= 2 *M_PI;
    } 

    if (angle < - M_PI) {
        angle += 2 * M_PI;
    }
}

/**
 * @brief 判断点是否在地图内
 * 
 * @param x 
 * @param y 
 * @return true 
 * @return false 
 */
bool HybridAstar::isInMap(double x, double y) {
    if (x < x_min_ || x > x_max_ ||
        y < y_min_ || y > y_max_) {
        return false;
    }
    return true;
}

} // namespace planning