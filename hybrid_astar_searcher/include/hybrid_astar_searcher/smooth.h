/**
 * @file smooth.h
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

// 包含数学库，用于数学运算
#include <cmath>
// 包含向量容器，用于存储路径点
#include <vector>
// 包含输入输出流，用于调试输出
#include <iostream>
// 包含动态Voronoi图的头文件
#include "hybrid_astar_searcher/dynamicvoronoi.h"
// 包含混合A*搜索器的头文件
#include "hybrid_astar_searcher/hybrid_astar.h"
// 包含类型定义的头文件
#include "hybrid_astar_searcher/type_defs.h"
// 包含L-BFGS优化算法的头文件
#include "hybrid_astar_searcher/lbfgs.hpp"
// 包含浮点数相关常量的头文件
#include <cfloat>
// 包含格式化输出的头文件
#include <iomanip>

// 定义命名空间planning
namespace planning
{
// 定义平滑器类
class Smoother
{
private:
    // 最大曲率，用于限制路径的曲率
    double max_kappa_ = 1.0 / 5;
    // 最大清除距离，用于限制路径与障碍物的距离
    double max_clearance_ = 1.4;
    // 最大Voronoi距离，与最大清除距离相同
    double max_voronoi_ = max_clearance_;

    // 平滑参数，用于调整平滑效果
    double alpha_ = 0.01;
    // 障碍物权重，用于调整障碍物对路径的影响
    double w_obs_ = 1;
    // Voronoi权重，用于调整Voronoi图对路径的影响
    double w_vor_ = 0;
    // 曲率权重，用于调整路径曲率的影响
    double w_cur_ = 10;
    // 平滑权重，用于调整路径平滑度的影响
    double w_smo_ = 10;

    // 动态Voronoi图对象
    DynamicVoronoi voronoi_;
    // 地图宽度
    int width_;
    // 地图高度
    int height_;

    // 原始路径，存储路径点
    std::vector<Vec3d> path_;
    // 平滑后的路径，存储平滑后的路径点
    std::vector<Vec3d> smooth_path_;

    // L-BFGS优化算法的参数
    lbfgs::lbfgs_parameter_t lbfgs_params;
    // 代价函数，用于计算路径的代价
    static double costFunction(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g) ;

public:
    // 构造函数
    Smoother(/* args */);
    // 析构函数
    ~Smoother();
    // 优化函数，用于优化路径
    double optimize(DynamicVoronoi &voronoi, std::vector<Vec3d> &path);

    // 平滑路径函数，用于平滑路径
    void smoothPath(DynamicVoronoi &voronoi, std::vector<Vec3d> &path);
    // 获取平滑后的路径
    void getSmoothPath(std::vector<Vec3d> &smooth_path) {smooth_path = smooth_path_;}
    // 计算障碍物项，用于路径优化
    Vec2d calObstacleTerm(Vec2d x);
    // 计算平滑项，用于路径优化
    Vec2d calSmoothTerm(Vec2d x_p2, Vec2d x_p, Vec2d x_c, Vec2d x_n, Vec2d x_n2);

    // 判断点是否在地图内
    bool isInMap(Vec2d x);

};

} // namespace planning