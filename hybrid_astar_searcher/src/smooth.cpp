/**
 * @file smooth.cpp
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "hybrid_astar_searcher/smooth.h"

namespace planning
{

// Smoother类的构造函数，初始化对象
Smoother::Smoother(/* args */)
{
}

// Smoother类的析构函数，释放对象
Smoother::~Smoother()
{
}

// 计算成本函数的函数，用于优化路径平滑度
double Smoother::costFunction(void *ptr, 
                                const Eigen::VectorXd &x,  // 优化变量
                                Eigen::VectorXd &g)  {  // 梯度
    auto instance = reinterpret_cast<Smoother *>(ptr);  // 将指针转换为Smoother实例
    std::vector<Vec3d> smooth_path = instance->smooth_path_;  // 获取平滑路径
    const int points_num = smooth_path.size() - 4;  // 计算路径点的数量
    Eigen::Matrix2Xd opt_points;  // 定义优化点矩阵
    opt_points.resize(2, smooth_path.size());  // 调整矩阵大小
    opt_points(0,0) = smooth_path[0](0);  // 设置第一个点的x坐标
    opt_points(1,0) = smooth_path[0](1);  // 设置第一个点的y坐标
    opt_points(0,1) = smooth_path[1](0);  // 设置第二个点的x坐标
    opt_points(1,1) = smooth_path[1](1);  // 设置第二个点的y坐标
    //std::cout << "0000" << std::endl;
    
    // 将优化变量x中的值赋给opt_points矩阵
    opt_points.block(0,2,1,points_num) = x.head(points_num).transpose();
    opt_points.block(1,2,1,points_num) = x.tail(points_num).transpose();
    //std::cout << "1111" << std::endl;
    // 设置最后两个点的坐标
    opt_points.col(smooth_path.size()-2)(0) = smooth_path[smooth_path.size()-2](0);
    opt_points.col(smooth_path.size()-2)(1) = smooth_path[smooth_path.size()-2](1);
    opt_points.col(smooth_path.size()-1)(0) = smooth_path[smooth_path.size()-1](0);
    opt_points.col(smooth_path.size()-1)(1) = smooth_path[smooth_path.size()-1](1);
    //std::cout << "opt_points" << opt_points << std::endl;
    

    Eigen::Matrix2Xd grad;  // 定义梯度矩阵
    grad.resize(2, points_num);  // 调整梯度矩阵大小
    grad.setZero();  // 初始化梯度矩阵为零
    double cost = 0.0;  // 初始化成本为零

    
    double max_clearance = instance->max_clearance_;  // 获取最大清除距离
    
    //std::cout << "calculate collision cost" << std::endl;
    // 计算碰撞成本
    double collision_cost = 0.0;
    Eigen::Matrix2Xd collision_grad;
    collision_grad.resize(2, points_num);
    collision_grad.setZero();

    for (int i = 2; i < opt_points.cols()-2; ++i) {
        Vec2i x_i;
        x_i(0) = static_cast<int>((opt_points(0, i) - (-25)) / 1);  // 计算x坐标索引
        x_i(1) = static_cast<int>((opt_points(1, i) - (-25)) / 1);  // 计算y坐标索引

        // 计算到最近障碍物的距离，单位为米，1是分辨率
        double dist2obs = 1 * instance->voronoi_.getDistance(x_i(0), x_i(1));
        //std::cout << "dist2obs:" << dist2obs << std::endl;

        Vec2d vec_o2x(x_i(0) - instance->voronoi_.data[x_i(0)][x_i(1)].obstX,
                  x_i(1) - instance->voronoi_.data[x_i(0)][x_i(1)].obstY);
        //std::cout << "vec_o2x:" << vec_o2x << std::endl;
        

        if (dist2obs - max_clearance < 0) {
            collision_cost += instance->w_obs_ * pow((dist2obs - max_clearance), 2);  // 计算碰撞成本
            Vec2d gradient;
            gradient = instance->w_obs_ * 2 * (dist2obs - max_clearance) / dist2obs * vec_o2x;  // 计算碰撞梯度
            collision_grad(0, i-2) = gradient(0);
            collision_grad(1, i-2) = gradient(1);
        } else {
            collision_cost += 0;
            collision_grad(0, i-2) = 0;
            collision_grad(1, i-2) = 0;
        }
        
    }
    cost += collision_cost;  // 累加碰撞成本
    grad += collision_grad;  // 累加碰撞梯度

    //std::cout << "calculate smooth cost" << std::endl;
    // 计算平滑成本
    double smooth_cost = 0.0;
    Eigen::Matrix2Xd smooth_grad;
    smooth_grad.resize(2, points_num);
    smooth_grad.setZero();
    //std::cout << opt_points.cols()-1 << std::endl;
    for (int i = 2; i < opt_points.cols()-2; ++i)  {
        Vec2d x_p2(opt_points(0, i-2), opt_points(1, i-2));
        Vec2d x_p(opt_points(0, i-1), opt_points(1, i-1));
        Vec2d x_c(opt_points(0, i), opt_points(1, i));
        Vec2d x_n(opt_points(0, i+1), opt_points(1, i+1));
        Vec2d x_n2(opt_points(0, i+2), opt_points(1, i+2));

        
        
        Vec2d err = x_p + x_n - 2* x_c;  // 计算平滑误差

        smooth_cost += instance->w_smo_ * err.transpose() * err;  // 计算平滑成本
        //std::cout << smooth_cost << std::endl;
        
        //smooth_grad.col(i-1) = ((-4) * x_p + 8 * x_c - 4 * x_n);
        smooth_grad.col(i-2) = instance->w_smo_ * 2 * (x_p2 - 4 * x_p + 6 * x_c - 4 * x_n + x_n2);  // 计算平滑梯度

        
    }
    //std::cout << "smooth_grad" << smooth_grad << std::endl;
    cost += smooth_cost;  // 累加平滑成本
    grad +=  smooth_grad;  // 累加平滑梯度
    //std::cout << "grad" << grad << std::endl;

    // 计算曲率成本
    double curvature_cost = 0.0;
    Eigen::Matrix2Xd curvature_grad;
    curvature_grad.resize(2, points_num);
    curvature_grad.setZero();
    //std::cout << opt_points.cols()-1 << std::endl;
    for (int i = 2; i < opt_points.cols()-2; ++i)  {
        Vec2d x_p2(opt_points(0, i-2), opt_points(1, i-2));
        Vec2d x_p(opt_points(0, i-1), opt_points(1, i-1));
        Vec2d x_c(opt_points(0, i), opt_points(1, i));
        Vec2d x_n(opt_points(0, i+1), opt_points(1, i+1));
        Vec2d x_n2(opt_points(0, i+2), opt_points(1, i+2));

        // 四段线
        Vec2d delta_x_p = x_p - x_p2;
        Vec2d delta_x_c = x_c - x_p;
        Vec2d delta_x_n = x_n - x_c;
        Vec2d delta_x_n2 = x_n2 - x_n;

        if (delta_x_p.norm() > 0 && delta_x_c.norm() > 0 && delta_x_n.norm() > 0 && delta_x_n2.norm() > 0) {
            // 取[-1,1]防止出现nan
            double delta_phi_p = std::acos(std::min(std::max(delta_x_p.dot(delta_x_c) / delta_x_p.norm() / delta_x_c.norm(), -1.0), 1.0));
            double delta_phi_c = std::acos(std::min(std::max(delta_x_c.dot(delta_x_n) / delta_x_c.norm() / delta_x_n.norm(), -1.0), 1.0));
            double delta_phi_n = std::acos(std::min(std::max(delta_x_n.dot(delta_x_n2) / delta_x_n.norm() / delta_x_n2.norm(), -1.0), 1.0));
            //std::cout << delta_x_p.dot(delta_x_c) / delta_x_p.norm() / delta_x_c.norm() << std::endl;
            //std::cout << "delta_phi_p:" << delta_phi_p << std::endl;

            double kappa_p = delta_phi_p / delta_x_p.norm();
            double kappa_c = delta_phi_c / delta_x_c.norm();
            double kappa_n = delta_phi_n / delta_x_n.norm();

            if (kappa_c > instance->max_kappa_ && kappa_p > 0 && kappa_n > 0) {
                auto compute_d_delta_phi = [](const double delta_phi) {
                    return -1.0 / std::sqrt(1.0 - std::pow(std::cos(delta_phi),2));
                };

                auto compute_orthogonal_complement = [](Vec2d x0, Vec2d x1) {
                    return x0 - x1 * x0.dot(x1) / std::pow(x1.norm(), 2);
                };

                double d_delta_phi_p = compute_d_delta_phi(delta_phi_p);
                Vec2d d_cos_delta_phi_p = compute_orthogonal_complement(delta_x_p, delta_x_c) 
                                          /  delta_x_p.norm() / delta_x_c.norm();
                Vec2d d_kappa_p = 1.0 / delta_x_p.norm() * d_delta_phi_p *  d_cos_delta_phi_p;
                Vec2d k_p = 2.0 * (kappa_p - instance->max_kappa_) * d_kappa_p;
                // std::cout <<  std::pow(std::cos(delta_phi_p),2) << std::endl;
                //std::cout << "d_delta_phi_p:" << d_delta_phi_p << std::endl;
                //std::cout << "d_cos_delta_phi_p:" << d_cos_delta_phi_p << std::endl;
                //std::cout << "d_kappa_p:" << d_kappa_p << std::endl;
               
                //std::cout << "kp:" << k_p << std::endl;


                double d_delta_phi_c = compute_d_delta_phi(delta_phi_c);
                Vec2d d_cos_delta_phi_c = compute_orthogonal_complement(delta_x_n, delta_x_c) 
                                          /  delta_x_c.norm() / delta_x_n.norm()
                                          -compute_orthogonal_complement(delta_x_c, delta_x_n)
                                          / delta_x_c.norm() / delta_x_n.norm();

                Vec2d d_kappa_c = 1.0 / delta_x_c.norm() * d_delta_phi_c *  d_cos_delta_phi_c 
                                  -delta_phi_c / std::pow(delta_x_c.norm(), 3) * delta_x_c;
                Vec2d k_c = 2.0 * (kappa_c - instance->max_kappa_) * d_kappa_c;

                //std::cout << "d_cos_delta_phi_c:" << d_cos_delta_phi_c << std::endl;
                //std::cout << "k_c:" << k_c << std::endl;

                double d_delta_phi_n = compute_d_delta_phi(delta_phi_n);
                Vec2d d_cos_delta_phi_n = -compute_orthogonal_complement(delta_x_n2, delta_x_n) 
                                          /  delta_x_n.norm() / delta_x_n2.norm();
                Vec2d d_kappa_n = 1.0 / delta_x_n.norm() * d_delta_phi_n *  d_cos_delta_phi_n 
                                  +delta_phi_n / std::pow(delta_x_n.norm(), 3) * delta_x_n;
                Vec2d k_n = 2.0 * (kappa_n - instance->max_kappa_) * d_kappa_n;
                //std::cout << "d_cos_delta_phi_n:" << d_cos_delta_phi_n << std::endl;
                //std::cout << "kn:" << k_n << std::endl;

                
                curvature_cost += instance->w_cur_ * std::pow(kappa_c - instance->max_kappa_, 2);  // 计算曲率成本

                curvature_grad.col(i-2) = instance->w_cur_ * (  0.25*k_p +  0.5*k_c + 0.25*k_n );  // 计算曲率梯度

            } else {
                curvature_cost += 0;
                curvature_grad.col(i-2) = Vec2d(0, 0);
            }

            
        }

    }
    //std::cout << "curvature_grad" << curvature_grad << std::endl;
    cost += curvature_cost;  // 累加曲率成本
    grad += curvature_grad;  // 累加曲率梯度


    // // Voronoi代价计算
    // double voronoi_cost = 0.0; // 初始化Voronoi代价为0.0
    // Eigen::Matrix2Xd voronoi_grad; // 定义一个2行points_num列的矩阵，用于存储Voronoi梯度
    // voronoi_grad.resize(2, points_num); // 调整矩阵大小为2行points_num列
    // voronoi_grad.setZero(); // 将矩阵所有元素初始化为0
    // // 遍历优化点集中的每一个点，从第2个点到倒数第2个点
    // for (int i = 2; i < opt_points.cols()-2; ++i)  {

    //     Vec2i x_i; // 定义一个2维整数向量，用于存储当前点的坐标
    //     // 将当前点的x坐标转换为整数索引
    //     x_i(0) = static_cast<int>((opt_points(0, i) - (-25)) / 0.1);
    //     // 将当前点的y坐标转换为整数索引
    //     x_i(1) = static_cast<int>((opt_points(1, i) - (-25)) / 0.1);

    //     // 计算当前点到最近障碍物的距离，单位为米
    //     double dist2obs = 0.1 * instance->voronoi_.getDistance(x_i(0), x_i(1));
    //     // 输出当前点到最近障碍物的距离
    //     // std::cout << "dist2obs:" << dist2obs << std::endl;

    //     // 计算从最近障碍物到当前点的向量
    //     Vec2d vec_o2x(x_i(0) - instance->voronoi_.data[x_i(0)][x_i(1)].obstX,
    //                 x_i(1) - instance->voronoi_.data[x_i(0)][x_i(1)].obstY);
        
    //     // 定义当前点的Voronoi区域坐标
    //     int x_v = x_i(0), y_v = x_i(1);
    //     // 定义一个向量，用于存储Voronoi区域内的点
    //     std::vector<Vec2i> voronoi_points;
    //     // 遍历当前点周围的30x30区域，寻找Voronoi区域内的点
    //     for (int i = x_v - 30; i < (x_v + 30); ++i) {
    //         for (int j = y_v - 30; j < (y_v + 30); ++j) {
    //             // 如果当前点在Voronoi区域内
    //             if (instance->voronoi_.isVoronoi(x_v, y_v)) {
    //                 // 将当前点加入Voronoi区域内的点集合
    //                 voronoi_points.push_back({x_v, y_v});
    //             }
    //         }
    //     }
    //     double dist2edge; // 定义一个变量，用于存储当前点到Voronoi边缘的距离
    //     Vec2d vec_e2x; // 定义一个2维向量，用于存储从Voronoi边缘到当前点的向量
    //     // 如果Voronoi区域内的点集合为空
    //     if (voronoi_points.empty()) {
    //         // 设置当前点到Voronoi边缘的距离为3
    //         dist2edge = 3;
    //         // 设置从Voronoi边缘到当前点的向量为从最近障碍物到当前点的向量的负方向
    //         vec_e2x  = -vec_o2x;
    //     } else {
    //         // 初始化最小距离索引和最小距离
    //         int min_idx = 0;
    //         double min_dist = 10;
    //         // 遍历Voronoi区域内的点集合
    //         for (int i = 0; i < voronoi_points.size(); ++i) {
    //             // 计算当前点到Voronoi区域内某点的距离
    //             double dist = 0.1 * (x_i - voronoi_points[i]).norm();
    //             // 如果当前距离小于最小距离
    //             if (dist < min_dist) {
    //                 // 更新最小距离和最小距离索引
    //                 min_dist = dist;
    //                 min_idx = i;
    //             }
    //         }
    //         // 设置当前点到Voronoi边缘的距离为最小距离
    //         dist2edge = min_dist;
    //         // 设置从Voronoi边缘到当前点的向量为从Voronoi区域内某点到当前点的向量
    //         vec_e2x(x_i(0) - voronoi_points[min_idx](0),
    //                 x_i(1) - voronoi_points[min_idx](1));
    //     }

    //     // 获取alpha值
    //     double alpha = instance->alpha_;

    //     // 如果当前点到最近障碍物的距离小于最大安全距离
    //     if (dist2obs - max_clearance < 0) {
    //         // 输出提示信息
    //         std::cout << "求gradient:" << std::endl;

    //         // 计算Voronoi代价
    //         voronoi_cost += instance->w_vor_ * alpha /(alpha + dist2obs)
    //                         * dist2edge / (dist2edge + dist2obs)
    //                         * pow(dist2obs - max_clearance, 2) / pow(max_clearance, 2);
            
    //         // 输出提示信息
    //         std::cout << "求gradient:" << std::endl;
            
    //         // 定义一个2维向量，用于存储梯度
    //         Vec2d gradient;
    //         // 计算梯度
    //         gradient = instance->w_vor_ * 
    //                 (alpha /(alpha + dist2obs)
    //                 * dist2edge / (dist2edge + dist2obs)
    //                 * (dist2obs - max_clearance) / pow(max_clearance, 2)
    //                 * ((max_clearance - dist2obs)/(alpha + dist2obs)
    //                     -(dist2obs - max_clearance) / (dist2obs + dist2edge) + 2)
    //                 * vec_o2x / dist2obs
                    
    //                 + 
                    
    //                     alpha /(alpha + dist2obs) 
    //                 * dist2obs / pow(dist2edge + dist2obs, 2)
    //                 * pow(dist2obs - max_clearance, 2) / pow(max_clearance, 2)
    //                 * vec_e2x / dist2edge
    //                 );
                    
    //         // 将梯度存储到Voronoi梯度矩阵中
    //         voronoi_grad(0, i-2) = gradient(0);
    //         voronoi_grad(1, i-2) = gradient(1);
    //     } else {
    //         // 如果当前点到最近障碍物的距离大于等于最大安全距离，则Voronoi代价为0
    //         voronoi_cost += 0;
    //         // 将Voronoi梯度矩阵中对应位置的元素设置为0
    //         voronoi_grad(0, i-2) = 0;
    //         voronoi_grad(1, i-2) = 0;
    //     }
        
    // }
    // // 输出平滑梯度
    // // std::cout << "smooth_grad" << smooth_grad << std::endl;
    // // 将Voronoi代价加到总代价中
    // cost += voronoi_cost;
    // // 将Voronoi梯度加到总梯度中
    // grad +=  voronoi_grad;



    g.setZero();
    g.head(points_num) = grad.row(0).transpose();
    g.tail(points_num) = grad.row(1).transpose();
    //std::cout << "g" << g << std::endl;



    // std::cout << std::setprecision(10)
    // std::cout << "------------------------" << "\n";
    // std::cout << "Function Value: " << cost << "\n";
    // std::cout << "Gradient Inf Norm: " << g.cwiseAbs().maxCoeff() << "\n";
    // std::cout << "------------------------" << "\n";

    return cost;

}

double Smoother::optimize(DynamicVoronoi &voronoi, std::vector<Vec3d> &path) {
    // 将传入的路径赋值给类的成员变量 smooth_path_
    smooth_path_ = path;
    // 将传入的动态Voronoi图赋值给类的成员变量 voronoi_
    voronoi_ = voronoi;

    // 计算路径中需要优化的点的数量（去掉首尾各两个点）
    int points_num = smooth_path_.size() - 4;
    // 创建一个Eigen向量x，用于存储需要优化的点的坐标
    Eigen::VectorXd x(2 * points_num);
    // 遍历路径中需要优化的点，将它们的坐标存储到向量x中
    for (int i = 2; i < smooth_path_.size()-2; ++i) {
        x(i-2) = smooth_path_[i](0); // 存储x坐标
        x(i-2 + points_num) = smooth_path_[i](1); // 存储y坐标
    }
    
    // 初始化最小代价为0.0
    double minCost = 0.0;
    // 设置L-BFGS算法的参数
    lbfgs_params.mem_size = 256; // 内存大小
    lbfgs_params.past = 3; // 历史记录长度
    lbfgs_params.min_step = 1.0e-32; // 最小步长
    lbfgs_params.g_epsilon = 0.0; // 梯度阈值
    lbfgs_params.delta = 1.0e-5; // 收敛阈值

    // 调用L-BFGS优化算法，优化路径
    int ret = lbfgs::lbfgs_optimize(x,
                                    minCost,
                                    &Smoother::costFunction, // 代价函数
                                    nullptr,
                                    nullptr,
                                    this,
                                    lbfgs_params);
    
    // 将优化后的点坐标更新到路径中
    for (int i = 2; i < smooth_path_.size() -2; ++i) {
            smooth_path_[i](0) = x(i-2); // 更新x坐标
            smooth_path_[i](1) = x(i-2 + points_num); // 更新y坐标
            // 计算并更新路径中点的方向角
            smooth_path_[i-1](2) = std::atan2(smooth_path_[i](1) - smooth_path_[i-1](1),
                                         smooth_path_[i](0) - smooth_path_[i-1](0));
    }
    // 更新最后一个点的方向角
    smooth_path_[smooth_path_.size() -3](2) = std::atan2(smooth_path_[smooth_path_.size() -2](1) - smooth_path_[smooth_path_.size() -3](1),
                                         smooth_path_[smooth_path_.size() -2](0) - smooth_path_[smooth_path_.size() -3](0));
    // 如果优化成功
    if (ret >= 0) {
        std::cout << "Optimization success" << std::endl;
    } else {
        // 如果优化失败，将最小代价设为无穷大，并输出错误信息
        minCost = INFINITY;
        std::cout << "Optimization Failed: "
                    << lbfgs::lbfgs_strerror(ret)
                    << std::endl;
    }

    // 返回最小代价
    return minCost;
}

void Smoother::smoothPath(DynamicVoronoi &voronoi, std::vector<Vec3d> &path) {
    // 将传入的路径赋值给类的成员变量 path_ 和 smooth_path_
    path_ = path;
    smooth_path_ = path;
    // 将传入的动态Voronoi图赋值给类的成员变量 voronoi_
    voronoi_ = voronoi;
    // 获取Voronoi图的宽度和高度
    width_ = voronoi_.getSizeX();
    height_ = voronoi_.getSizeY();
    
    // 初始化迭代次数
    int iter = 0;
    // 设置最大迭代次数
    int max_iter = 1000;

    // 计算权重总和
    double weight_sum = w_obs_ + w_cur_ + w_smo_ + w_vor_;

    // 开始迭代优化路径
    while (iter < max_iter) {
        
        // 遍历路径中需要优化的点
        for (int i = 2; i < path_.size() - 2; ++i) {
            // 获取当前点的相邻点
            Vec2d x_p2(smooth_path_[i-2](0), smooth_path_[i-2](1));
            Vec2d x_p(smooth_path_[i-1](0), smooth_path_[i-1](1));
            Vec2d x_c(smooth_path_[i](0), smooth_path_[i](1));
            Vec2d x_n(smooth_path_[i+1](0), smooth_path_[i+1](1));
            Vec2d x_n2(smooth_path_[i+2](0), smooth_path_[i+2](1));

            // 初始化修正向量为零向量
            Vec2d correction = Vec2d::Zero();

            // 计算障碍物项的修正向量
            correction = correction + calObstacleTerm(x_c);
            
            // 检查修正后的点是否在地图范围内
            if (!isInMap(x_c - alpha_ * correction / weight_sum)) continue;

            // 计算平滑项的修正向量
            correction = correction + calSmoothTerm(x_p2, x_p, x_c, x_n, x_n2);
            
            // 检查修正后的点是否在地图范围内
            if (!isInMap(x_c - alpha_ * correction / weight_sum)) continue;

            // 更新当前点的坐标
            x_c = x_c - alpha_ * correction / weight_sum;
            
            // 将更新后的坐标赋值给路径
            smooth_path_[i](0) = x_c(0);
            smooth_path_[i](1) = x_c(1);

            // 计算当前点与前一个点的坐标差
            Vec2d delta_x = x_c - x_p;
            // 如果当前点不是第一个点，则更新前一个点的方向角
            if (i > 1) {
                smooth_path_[i-1](2) = std::atan2(delta_x(1), delta_x(0));
            }
            
        }

        // 迭代次数加1
        ++iter;
        
    }
    
    // 输出最终的迭代次数
    std::cout << iter << std::endl;
}

Vec2d Smoother::calObstacleTerm(Vec2d x) {
    // 初始化梯度向量
    Vec2d gradient;

    // 将点的坐标转换为整数索引
    Vec2i x_i;
    x_i(0) = static_cast<int>((x(0) - (-25)) / 0.1);
    x_i(1) = static_cast<int>((x(1) - (-25)) / 0.1);

    // 计算点到最近障碍物的距离（单位：米）
    double dist2obs = 0.1 * voronoi_.getDistance(x_i(0), x_i(1));

    // 计算从障碍物到点的向量
    Vec2d vec_o2x(x_i(0) - voronoi_.data[x_i(0)][x_i(1)].obstX,
                  x_i(1) - voronoi_.data[x_i(0)][x_i(1)].obstY);
    
    // 如果距离小于最大清除距离，则计算梯度
    if (dist2obs  < max_clearance_) {
        gradient = w_obs_ * 2 * (dist2obs - max_clearance_) / dist2obs * vec_o2x;
    } else {
        // 否则，梯度为零向量
        gradient = Vec2d::Zero();
    }
    // 返回梯度
    return gradient;
}

Vec2d Smoother::calSmoothTerm(Vec2d x_p2, Vec2d x_p, Vec2d x_c, Vec2d x_n, Vec2d x_n2) {
    // 初始化梯度向量
    Vec2d gradient;
    // 计算平滑项的梯度
    gradient = w_smo_ * (x_p2 - 4 * x_p + 6 * x_c - 4 * x_n + x_n2);
    
    // 返回梯度
    return gradient;
}

bool Smoother::isInMap(Vec2d x) {
    // 检查点是否在地图范围内
    if (x(0) < -25 || x(1) < -25 || x(0) >= 25 || x(1) >= 25) {
        return false;
    }
    return true;
}

} // namespace planning


