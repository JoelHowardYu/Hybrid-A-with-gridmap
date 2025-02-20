/**
 * @file hybrid_astar_test.cpp
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <rclcpp/rclcpp.hpp> // 包含ROS 2的核心库
#include <nav_msgs/msg/path.hpp> // 包含路径消息类型
#include <geometry_msgs/msg/pose_stamped.hpp> // 包含带时间戳的位姿消息类型
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp> // 包含带协方差的位姿消息类型
#include <visualization_msgs/msg/marker.hpp> // 包含可视化标记消息类型
#include <visualization_msgs/msg/marker_array.hpp> // 包含可视化标记数组消息类型
#include <tf2_ros/transform_listener.h> // 包含TF2变换监听器

#include <grid_map_ros/grid_map_ros.hpp> // 包含网格地图ROS接口
#include <grid_map_msgs/msg/grid_map.hpp> // 包含网格地图消息类型
#include <grid_map_core/Polygon.hpp> // 包含网格地图多边形类
#include <grid_map_core/iterators/PolygonIterator.hpp> // 包含网格地图多边形迭代器

#include <ompl/base/spaces/ReedsSheppStateSpace.h> // 包含OMPL的ReedsShepp状态空间
#include <ompl/geometric/SimpleSetup.h> // 包含OMPL的几何简单设置
#include <ompl/base/ScopedState.h> // 包含OMPL的作用域状态
#include <boost/program_options.hpp> // 包含Boost程序选项库
#include <ompl/config.h> // 包含OMPL的配置

#include "hybrid_astar_searcher/calculate_heuristic.h" // 包含启发式计算头文件
#include "hybrid_astar_searcher/hybrid_astar.h" // 包含混合A*头文件
#include "hybrid_astar_searcher/dynamicvoronoi.h" // 包含动态Voronoi图头文件
#include "hybrid_astar_searcher/smooth.h" // 包含路径平滑头文件

#include <tf2/utils.h> // 包含TF2的实用工具
#include "decomp_util/ellipsoid_decomp.h"  // 包含分解工具库中的椭球分解类
#include "decomp_ros_utils/data_ros_utils.h"  // 包含分解ROS工具库
namespace ob = ompl::base; // 命名空间别名
namespace og = ompl::geometric; // 命名空间别名
typedef ompl::base::SE2StateSpace::StateType State; // 状态类型别名
using namespace planning; // 命名空间别名
using namespace std; // 命名空间别名
using namespace grid_map; // 命名空间别名

rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_vis_pub, optimized_traj_pub; // 路径可视化和优化轨迹发布者
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub, voronoi_pub; // 标记和Voronoi图发布者
rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr sfc_pub; 

nav_msgs::msg::Path path; // 路径消息
geometry_msgs::msg::PoseStamped pose; // 位姿消息

int N = 100; // 路径点数量
double dt = 0.01; // 时间步长
Eigen::Vector3d start, goal; // 起始点和目标点
bool has_start = false, has_goal = false; // 是否接收到起始点和目标点的标志
Vec3d start_pose, goal_pose; // 起始点和目标点的位姿

GridMap gmap; // 网格地图
std::unique_ptr<HybridAstar> hybrid_astar_ptr; // 混合A*指针
HybridAstarResult result; // 混合A*结果
visualization_msgs::msg::Marker voronoi; // Voronoi图标记

std::unique_ptr<Smoother> smoother_ptr; // 平滑器指针
DynamicVoronoi voronoiDiagram; // Voronoi图实例

/**
 * @brief 网格地图回调函数
 * 
 * @param msg 网格地图消息
 */
void Gridmap_Callback(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
    if (gmap.exists("elevation")) return; // 如果地图中已经存在"elevation"层，则返回
    std::cout << "receive map" << std::endl; // 输出接收地图信息
    GridMapRosConverter::fromMessage(*msg, gmap); // 从消息中转换为网格地图
    std::cout << "FrameId:" << gmap.getFrameId() << std::endl; // 输出地图的FrameId
    std::cout << "map :" << gmap.getLength().x() << std::endl; // 输出地图的长度
    RCLCPP_INFO(rclcpp::get_logger("Gridmap_Callback"), "Created map with size %f x %f m (%i x %i cells).",
        gmap.getLength().x(), gmap.getLength().y(),
        gmap.getSize()(0), gmap.getSize()(1)); // 输出地图的大小信息
    hybrid_astar_ptr.reset(new HybridAstar(gmap)); // 重置混合A*指针

    int size_x = gmap.getSize()(0); // 获取地图的x方向大小
    int size_y = gmap.getSize()(1); // 获取地图的y方向大小
    //cout << "size :" << size_x << " " << size_y << endl;
    
    // 初始化一个二进制地图，用于构建Voronoi图
    bool **bin_map;
    bin_map = new bool *[size_x];
    for (int i = 0; i < size_x; ++i) {
        bin_map[i] = new bool[size_y];
        for (int j = 0; j < size_y; ++j) {
            Vec2d pos;
            pos(0) = -25 + (i + 0.5) * gmap.getResolution();
            pos(1) = -25 + (j + 0.5) * gmap.getResolution();
            //cout << "gmap.getResolution():" <<gmap.getResolution() << endl;
            //cout << map.atPosition("elevation", pos);
            //cout << pos(0) << "x" << pos(1) << endl;
            if (gmap.atPosition("elevation", pos) > 0) {
                bin_map[i][j] = true;
            } else {
                bin_map[i][j] = false;
            }
        }
    }

    voronoiDiagram.initializeMap(size_x, size_y, bin_map); // 初始化Voronoi图
    
    voronoiDiagram.update();
    voronoiDiagram.prune();
   
    

    voronoiDiagram.visualize("../result.pgm");
    //cout << "voronoi图可视化" << std::endl;

    
    voronoi.header.frame_id = "map";
    voronoi.header.stamp = msg->header.stamp;
    voronoi.ns = "voronoi";
    voronoi.id = 0;
    voronoi.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    voronoi.action = visualization_msgs::msg::Marker::ADD;

    voronoi.color.b = 1.0;
    voronoi.color.a = 1.0;

    voronoi.scale.x = 0.1;
    voronoi.scale.y = 0.1;
    voronoi.scale.z = 0.1;
    geometry_msgs::msg::Point p;
    for (int i = size_y-1; i >= 0; --i) {
        for (int j = 0; j < size_x; ++j) {
            if (voronoiDiagram.isVoronoi(i, j)) {
                Vec2d pos;
                pos(0) = -25 + (i + 0.5) * gmap.getResolution();
                pos(1) = -25 + (j + 0.5) * gmap.getResolution();
                p.x = pos(0);
                p.y = pos(1);
                p.z = 0.05;
                voronoi.points.push_back(p);
            }
        }
    }
    

}   

/**
 * @brief 可视化路径
 * 
 * @param path 路径
 */
void visPath(std::vector<Eigen::Vector3d> path);

std::vector<Eigen::Matrix<double, 6, -1>> polyhTypeConverter(
    vec_E<Polyhedron<3>> vs) {
  std::vector<Eigen::Matrix<double, 6, -1>> polys; // 定义一个存储多面体的向量
  polys.reserve(vs.size()); // 预留空间，避免多次内存分配
  for (const auto& v : vs) { // 遍历输入的多面体向量
    Eigen::MatrixXd poly; // 定义一个多面体矩阵
    poly.resize(6, v.hyperplanes().size()); // 调整矩阵大小，使其能够容纳所有超平面
    int i = 0; // 初始化列索引
    for (const auto& p : v.hyperplanes()) { // 遍历多面体的每个超平面
      poly.col(i).tail<3>() = p.p_; // 设置点的坐标，存储在矩阵的最后3行
      poly.col(i).head<3>() = p.n_; // 设置法向量的坐标，存储在矩阵的前3行
      i++; // 增加列索引
    }
    polys.push_back(poly); // 将多面体矩阵添加到向量中
  }
  return polys; // 返回多面体向量
}

void visualizeCorridors(const std::vector<Eigen::Matrix<double, 6, -1>>& hPolys) {
    decomp_ros_msgs::msg::PolyhedronArray poly_msg;
    for (size_t i = 0; i < hPolys.size(); ++i) {
        const Eigen::MatrixXd& hpoly = hPolys[i];
        decomp_ros_msgs::msg::Polyhedron msg;
        for (size_t j = 0; j < hpoly.cols(); ++j) {
            geometry_msgs::msg::Point pt, n;
            pt.x = hpoly(3, j);
            pt.y = hpoly(4, j);
            pt.z = hpoly(5, j);
            n.x = hpoly(0, j);
            n.y = hpoly(1, j);
            n.z = hpoly(2, j);
            msg.points.push_back(pt);
            msg.normals.push_back(n);
        }
        poly_msg.polyhedrons.push_back(msg);
    }
    RCLCPP_INFO(rclcpp::get_logger("visualizeCorridors"), "publish polyhedron");
    poly_msg.header.frame_id = "map";
    poly_msg.header.stamp = rclcpp::Clock().now();
    sfc_pub->publish(poly_msg);
}


/**
 * @brief 主循环函数，给初始点和目标点就生成一条路径
 * 
 */
void run() {
    //RCLCPP_INFO(rclcpp::get_logger("run"), "run()");
    if (has_start && has_goal) { // 如果接收到起始点和目标点
        RCLCPP_INFO(rclcpp::get_logger("run"), "has start and goal"); // 输出信息
        voronoi_pub->publish(voronoi); // 发布Voronoi图
        auto hybrid_astar_start = std::chrono::high_resolution_clock::now(); // 记录混合A*开始时间
        Visualize vis; // 可视化对象
        if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) { // 执行混合A*规划
            RCLCPP_INFO(rclcpp::get_logger("run"), "search success"); // 输出成功信息
            auto hybrid_astar_end = std::chrono::high_resolution_clock::now(); // 记录混合A*结束时间
            std::chrono::duration<double> hybrid_astar_use_time = hybrid_astar_end - hybrid_astar_start; // 计算混合A*用时
            cout << "hybrid astar use time:" << hybrid_astar_use_time.count() * 1000 << "ms" << std::endl; // 输出混合A*用时
            vector<Vec3d> path2smooth; // 待平滑路径
            for (int i = 0; i < result.x.size(); ++i) {
                path2smooth.push_back({result.x[i], result.y[i], result.theta[i]});
            }
            
            //lbfgs平滑
            auto smooth_path_start = std::chrono::high_resolution_clock::now(); // 记录平滑开始时间
            smoother_ptr->optimize(voronoiDiagram, path2smooth); // 执行路径平滑优化
            //smoother_ptr->smoothPath(voronoiDiagram, path2smooth); // 这个是梯度下降版的
            
            vector<Vec3d> smooth_path; // 平滑后的路径
            smoother_ptr->getSmoothPath(smooth_path); // 获取平滑后的路径
            auto smooth_path_end = std::chrono::high_resolution_clock::now(); // 记录平滑结束时间
            std::chrono::duration<double> smooth_path_use_time = smooth_path_end - smooth_path_start; // 计算平滑用时
            cout << "smooth path use time:" << smooth_path_use_time.count() * 1000 << "ms" << std::endl; // 输出平滑用时

            // 设置障碍物
            vec_Vec3f observations;
            for (grid_map::GridMapIterator it(gmap); !it.isPastEnd(); ++it) {
                if (gmap.at("elevation", *it) > 2) {
                    Vec3f obs;
                    grid_map::Position position;
                    gmap.getPosition(*it, position);
                    obs[0] = position.x();
                    obs[1] = position.y();
                    obs[2] = 0.0;
                    observations.push_back(obs);
                }
            }

            // 转换平滑后的路径点
            vec_Vec3f waypointsf;
            for (const auto& pose : smooth_path) {
                Vec3f wp;
                wp[0] = pose[0];
                wp[1] = pose[1];
                wp[2] = pose[2];
                waypointsf.push_back(wp);
            }

            // 设置椭球分解对象的参数
            EllipsoidDecomp3D decomp_util;
            decomp_util.set_obs(observations);
            decomp_util.set_local_bbox(Vec3f(1, 2, 1));

            // 膨胀走廊
            decomp_util.dilate(waypointsf);

            // 获取走廊
            std::vector<Eigen::Matrix<double, 6, -1>> corridor = polyhTypeConverter(decomp_util.get_polyhedrons());

            // 可视化走廊
            visualizeCorridors(corridor);

            // std::vector<Eigen::Matrix<double, 6, -1>> corridor;  // 声明一个走廊向量
            // EllipsoidDecomp3D decomp_util;  // 声明一个椭球分解对象
            // vec_Vec3f observations;  // 声明一个观察点向量
            // decomp_util.set_obs(observations);  // 设置障碍物
            // decomp_util.set_local_bbox(Vec3f(1, 2, 1));  // 设置局部包围盒
            // vec_Vec3f waypointsf;  // 声明一个航点向量（浮点型）
            // decomp_util.dilate(waypointsf);  // 膨胀走廊

            // corridor = polyhTypeConverter(decomp_util.get_polyhedrons());  // 获取走廊
            // /* 清理缓冲区 */
            // std::ostringstream oss;
            // oss << "corridor size: " << corridor.size();  // 构建完整的字符串
            // RCLCPP_INFO(rclcpp::get_logger("run"), oss.str().c_str());  // 打印走廊大小
            // visualizeCorridors(corridor);  // 可视化走廊      




            // 可视化混合A星搜索的一些东西，其实可以写成一个函数，放在这太臃肿了
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map"; // 设置标记的FrameId
            marker.header.stamp = rclcpp::Clock().now(); // 设置标记的时间戳
            marker.ns = "rs_path"; // 设置命名空间
            marker.id = 1; // 设置ID
            marker.action = visualization_msgs::msg::Marker::ADD; // 设置动作
            marker.pose.orientation.w = 1.0; // 设置姿态
            
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // 设置标记类型
            marker.scale.x = 0.05; // 设置线宽

            // 设置标记颜色和透明度
            marker.color.a = 1.0; // 完全不透明
            marker.color.r = 1.0; // 红色
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            
            nav_msgs::msg::Path optimized_path; // 优化后的路径消息
            optimized_path.header.frame_id = "map"; // 设置FrameId
            optimized_path.header.stamp = rclcpp::Clock().now(); // 设置时间戳

            for (size_t i = 0; i < result.x.size(); ++i)
            {
                geometry_msgs::msg::Point point;
                point.x = result.x[i];
                point.y = result.y[i];
                point.z = 0.1;
                marker.points.push_back(point);
                vis.publishPathPoint({result.x[i], result.y[i]}, i);

                vis.publishVehicleBoxes({smooth_path[i](0), smooth_path[i](1), smooth_path[i](2)}, i);

                // 添加点到优化后的路径
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position = point;
                optimized_path.poses.push_back(pose);
            }
            marker_pub->publish(marker); // 发布标记
            optimized_traj_pub->publish(optimized_path); // 发布优化后的路径
            // 可视化平滑后的路径
            visPath(smooth_path);

            has_start = false; // 重置起始点标志
            has_goal = false; // 重置目标点标志
            
            
        } else {
            RCLCPP_WARN(rclcpp::get_logger("run"), "search fail"); // 输出失败信息
        }}
    // } else {
    //     RCLCPP_INFO(rclcpp::get_logger("run"), "waiting start or goal");
    // }

}

/**
 * @brief 可视化平滑后的路径
 * 
 * @param path 路径
 */
void visPath(std::vector<Eigen::Vector3d> path) {
    nav_msgs::msg::Path nav_path; // 路径消息
    nav_path.header.frame_id = "map"; // 设置FrameId
    nav_path.header.stamp = rclcpp::Clock().now(); // 设置时间戳
    geometry_msgs::msg::PoseStamped pos; // 位姿消息
    for (const auto pose : path) {
        pos.pose.position.x = pose[0];
        pos.pose.position.y = pose[1];
        pos.pose.position.z = 0.05;
        //std::cout << "x,y:" << pose[0] << " " << pose[1] << std::endl; 
        nav_path.poses.push_back(pos);
    }
    path_vis_pub->publish(nav_path); // 发布路径

}

/**
 * @brief 接收起始点位姿
 * 
 * @param msg 位姿消息
 */
void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {

    start_pose[0] = msg->pose.pose.position.x; // 获取起始点x坐标
    start_pose[1] = msg->pose.pose.position.y; // 获取起始点y坐标
    start_pose[2] = tf2::getYaw(msg->pose.pose.orientation); // 获取起始点yaw角
    has_start = true; // 设置起始点标志
    RCLCPP_INFO(rclcpp::get_logger("startCallback"), "receive start position"); // 输出接收起始点信息

}

/**
 * @brief 接收目标点位姿
 * 
 * @param msg 位姿消息
 */
void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

    goal_pose[0] = msg->pose.position.x; // 获取目标点x坐标
    goal_pose[1] = msg->pose.position.y; // 获取目标点y坐标
    goal_pose[2] = tf2::getYaw(msg->pose.orientation); // 获取目标点yaw角
    has_goal = true; // 设置目标点标志
    RCLCPP_INFO(rclcpp::get_logger("goalCallback"), "receive goal position"); // 输出接收目标点信息
}

/**
 * @brief 主函数
 * 
 * @param argc 参数数量
 * @param argv 参数数组
 * @return int 返回值
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hybrid_astar_test");

    auto gridmap_sub = node->create_subscription<grid_map_msgs::msg::GridMap>("/grid_map", 1, Gridmap_Callback);
    marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("rs_path_marker", 10);
    path_vis_pub = node->create_publisher<nav_msgs::msg::Path>("/path", 1);
    optimized_traj_pub = node->create_publisher<nav_msgs::msg::Path>("/optimized_path", 1);
    voronoi_pub = node->create_publisher<visualization_msgs::msg::Marker>("voronoi", 10);
    sfc_pub = node->create_publisher<decomp_ros_msgs::msg::PolyhedronArray>("safe_corridor", 1);

    auto start_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1, startCallback);
    auto goal_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1, goalCallback);

    smoother_ptr.reset(new Smoother());

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        run();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}