#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/iterators/PolygonIterator.hpp>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <boost/program_options.hpp>
#include <ompl/config.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>

#include "hybrid_astar_searcher/calculate_heuristic.h"
#include "hybrid_astar_searcher/hybrid_astar.h"

// 命名空间别名定义
namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace planning;
using std::vector;

// ROS路径可视化发布器
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_vis_pub;

// 定义状态类型为ompl::base::SE2StateSpace::StateType
typedef ompl::base::SE2StateSpace::StateType State;
using namespace grid_map;
// 定义网格地图对象
GridMap map_;
// 定义地图边界和分辨率变量
double x_min_, x_max_, y_min_, y_max_, xy_resolution_, theta_resolution_;
// 定义A*搜索指针
std::unique_ptr<GridSearch> astar_ptr;

// 网格地图回调函数
void Gridmap_Callback(const grid_map_msgs::msg::GridMap::SharedPtr msg){
    // 如果地图中已经存在"elevation"层，则直接返回
    if (map_.exists("elevation")) return;
    std::cout << "receive map" << std::endl; 
    // 从ROS消息中转换为网格地图
    GridMapRosConverter::fromMessage(*msg, map_);
    std::cout << "FrameId:" << map_.getFrameId() << std::endl;
    std::cout << "map :" << map_.getLength().x() << std::endl;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Created map with size %f x %f m (%i x %i cells).",
    map_.getLength().x(), map_.getLength().y(),
    map_.getSize()(0), map_.getSize()(1));
    // 重置A*搜索指针
    astar_ptr.reset(new GridSearch(map_));

    // 设置地图分辨率和角度分辨率
    xy_resolution_ = map_.getResolution();
    theta_resolution_ = 0.1;
    // 计算地图边界
    x_min_ = map_.getStartIndex()(0) * xy_resolution_ - 25;
    x_max_ = x_min_ + map_.getLength().x();
    y_min_ = map_.getStartIndex()(1) * xy_resolution_ -25;
    y_max_ = y_min_ + map_.getLength().y();
}   

// 状态有效性检查函数1
bool isStateValid1(const ob::SpaceInformation *si, const ob::State *state) {
    const auto *s = state->as<ob::SE2StateSpace::StateType>();
    double x = s->getX(), y = s->getY();

    // 检查状态是否在边界内，并且不在小正方形区域内
    return si->satisfiesBounds(s) && (x < -8 || x > -3 || y < 0 || y > 5);
}

// 计算车辆边界框的顶点
vector<Vec2d> calculateCarBoundingBox(Vec3d pose) {
    vector<Vec2d> vertices;
    const double dx1 = std::cos(pose(2)) * 4.9 / 2;
    const double dy1 = std::sin(pose(2)) * 4.9 / 2;
    const double dx2 = std::sin(pose(2)) * 2.6 / 2;
    const double dy2 = -std::cos(pose(2)) * 2.6 / 2;
    vertices.emplace_back(pose(0) + dx1 + dx2, pose(1) + dy1 + dy2);
    vertices.emplace_back(pose(0) + dx1 - dx2, pose(1) + dy1 - dy2);
    vertices.emplace_back(pose(0) - dx1 - dx2, pose(1) - dy1 - dy2);
    vertices.emplace_back(pose(0) - dx1 + dx2, pose(1) - dy1 + dy2); //顺时针
    return vertices;
}

// 角度归一化到[-π, π]
void mod2Pi(double &angle) {
    if (angle >  M_PI) {
        angle -= 2 *M_PI;
    } 

    if (angle < - M_PI) {
        angle += 2 * M_PI;
    }
}

// 检查点是否在地图内
bool isInMap(double x, double y) {
    if (x < x_min_ || x > x_max_ ||
        y < y_min_ || y > y_max_) {
        return false;
    }
    return true;
}

// 从姿态获取索引
Vec3i getIndexFromPose(Vec3d pose) {
    Vec3i index;
    index(0) = static_cast<int>((pose(0) - x_min_) / xy_resolution_);
    index(1) = static_cast<int>((pose(1) - y_min_) / xy_resolution_);
    mod2Pi(pose(2));
    index(2) = static_cast<int>((pose(2)) / theta_resolution_);
    return index;
}

// 检查线段是否碰撞
bool isLinecollision(double x0, double y0, double x1, double y1) {
    int check_point_num = static_cast<int>(std::max(abs(x1 - x0),abs(y1 - y0)) / 2) + 1;

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

// 有效性检查函数
bool validityCheck(std::shared_ptr<Node3d> node) {
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

// 状态有效性检查函数2
bool isStateValid2(const std::shared_ptr<ompl::base::SpaceInformation> si, const ob::State *state) {
    const auto *s = state->as<ob::SE2StateSpace::StateType>();
    double x = s->getX(), y = s->getY(), theta = s->getYaw();
    vector<double> vec_x = {x};
    vector<double> vec_y = {y};
    vector<double> vec_theta = {theta};
    
    std::shared_ptr<Node3d> node = std::make_shared<Node3d>(vec_x, vec_y, vec_theta);

    if (!validityCheck(node)) {
        return false;
    }
    return true;
}

// 可视化路径
void visPath(std::vector<Eigen::Vector2d> path) {
    auto nav_path = std::make_shared<nav_msgs::msg::Path>();
    nav_path->header.frame_id = "map";
    nav_path->header.stamp = rclcpp::Clock().now();
    geometry_msgs::msg::PoseStamped pos;
    for (const auto pose : path) {
        pos.pose.position.x = pose[0];
        pos.pose.position.y = pose[1];
        nav_path->poses.push_back(pos);
    }
    path_vis_pub->publish(*nav_path);
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ompl_test");

    auto gridmap_sub = node->create_subscription<grid_map_msgs::msg::GridMap>("/grid_map", 1, Gridmap_Callback);
    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("rs_path_marker", 10);

    path_vis_pub = node->create_publisher<nav_msgs::msg::Path>("/path", 1);
    ob::StateSpacePtr space(std::make_shared<ob::DubinsStateSpace>(3));

    std::cout << "设置bound" << std::endl;
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -25);  //参数0-x 1-y
    bounds.setLow(1, -25);
    bounds.setHigh(0, 25);
    bounds.setHigh(1, 25);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);
    std::cout << "设置bound结束" << std::endl;

    og::SimpleSetup ss(space);

    //设置状态有效性检测
    ob::SpaceInformationPtr si(std::make_shared<ob::SpaceInformation>(space));

    ob::OptimizationObjectivePtr optimizationObjective(new ob::PathLengthOptimizationObjective(si));
    ss.setOptimizationObjective(optimizationObjective);

    auto isStateValid = isStateValid2;
    ss.setStateValidityChecker([isStateValid, si](const ob::State *state)
    {
        return isStateValid(si, state);
    });
    ob::ScopedState<> start(space);
    start[0] = 5;
    start[1] = 0;
    start[2] = 3.14;
                
    ob::ScopedState<> goal(space);
    goal[0] = -19;
    goal[1] = -18;
    goal[2] = 2.2;
                
    ss.setStartAndGoalStates(start, goal);
    std::cout << "起点终点设置完成" << std::endl;

    std::vector<ob::State*> path_states;

    rclcpp::Rate rate(10);
    while(rclcpp::ok()) {
        if (map_.exists("elevation")) {
            std::cout << "求解开始" << std::endl;
            
            ob::PlannerStatus solved = ss.solve(1);
            ss.simplifySolution();
            og::PathGeometric solution_path =  ss.getSolutionPath();
            solution_path.interpolate(100);   

            path_states = solution_path.getStates();

            //画图
            auto marker = std::make_shared<visualization_msgs::msg::Marker>();
            marker->header.frame_id = "map"; // Set the frame ID to match your RVIZ frame
            marker->header.stamp = rclcpp::Clock().now();
            marker->ns = "rs_path";
            marker->action = visualization_msgs::msg::Marker::ADD;
            marker->pose.orientation.w = 1.0;
            marker->id = 0;
            marker->type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker->scale.x = 0.5; // Line width

            // Extract and convert OMPL path points to marker points
            for (size_t i = 0; i < path_states.size(); ++i)
            {
                auto values = path_states[i]->as<ompl::base::SE2StateSpace::StateType>();
                geometry_msgs::msg::Point point;
                point.x = values->getX();
                point.y = values->getY();
                point.z = 0.0;
                marker->points.push_back(point);
            }
            
            // Set marker color and opacity
            marker->color.a = 1.0; // Fully opaque
            marker->color.r = 1.0; // Red color
            marker->color.g = 0.0;
            marker->color.b = 0.0;
            marker_pub->publish(*marker);
        }

        rclcpp::spin_some(node);
        rate.sleep();
    }
        
    rclcpp::shutdown();
    
    return 0;
}