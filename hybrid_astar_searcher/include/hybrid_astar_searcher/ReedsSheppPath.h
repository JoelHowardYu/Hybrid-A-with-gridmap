#pragma once

#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "hybrid_astar_searcher/node3d.h"
#include "math.h"

namespace planning
{

// 定义ReedShepp路径结构体
struct ReedSheppPath {
  std::vector<double> segs_lengths; // 路径中每个段的长度
  std::vector<char> segs_types; // 路径中每个段的类型
  double total_length = 0.0; // 路径的总长度
  std::vector<double> x; // 路径中每个点的x坐标
  std::vector<double> y; // 路径中每个点的y坐标
  std::vector<double> phi; // 路径中每个点的方向角
  // true表示向前行驶，false表示向后行驶
  std::vector<bool> gear;
};

// 定义ReedShepp路径参数结构体
struct RSPParam {
  bool flag = false; // 标志位，表示是否有效
  double t = 0.0; // 参数t
  double u = 0.0; // 参数u
  double v = 0.0; // 参数v
};

// ReedShepp类，用于生成ReedShepp路径
class ReedShepp {
 public:
  // 构造函数，初始化最大曲率和步长
  ReedShepp(const double max_kappa, const double step_size);
  
  // 从所有可能的ReedShepp路径中选择最短的路径
  bool ShortestRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   ReedSheppPath& optimal_path);

 protected:
  // 生成所有可能的ReedShepp路径并进行插值
  bool GenerateRSPs(const std::shared_ptr<Node3d> start_node,
                    const std::shared_ptr<Node3d> end_node,
                    std::vector<ReedSheppPath>* all_possible_paths);
  
  // 设置运动基元的通用轮廓
  bool GenerateRSP(const std::shared_ptr<Node3d> start_node,
                   const std::shared_ptr<Node3d> end_node,
                   std::vector<ReedSheppPath>* all_possible_paths);
  
  // 设置运动基元的通用轮廓，并行实现
  bool GenerateRSPPar(const std::shared_ptr<Node3d> start_node,
                      const std::shared_ptr<Node3d> end_node,
                      std::vector<ReedSheppPath>* all_possible_paths);
  
  // 设置每个运动基元的局部精确配置
  bool GenerateLocalConfigurations(const std::shared_ptr<Node3d> start_node,
                                   const std::shared_ptr<Node3d> end_node,
                                   ReedSheppPath* shortest_path);
  
  // 在GenerateLocalConfiguration中使用的插值函数
  void Interpolation(const int index, const double pd, const char m,
                     const double ox, const double oy, const double ophi,
                     std::vector<double>* px, std::vector<double>* py,
                     std::vector<double>* pphi, std::vector<bool>* pgear);
  
  // 运动基元组合设置函数
  bool SetRSP(const int size, const double* lengths, const char* types,
              std::vector<ReedSheppPath>* all_possible_paths);
  
  // SetRSP的并行版本
  bool SetRSPPar(const int size, const double* lengths,
                 const std::string& types,
                 std::vector<ReedSheppPath>* all_possible_paths, const int idx);
  
  // ReedShepp路径中使用的六种不同的运动基元组合
  bool SCS(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CSC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CCC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CCCC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSCC(const double x, const double y, const double phi,
             std::vector<ReedSheppPath>* all_possible_paths);
  
  // 不同运动基元组合的不同选项
  void LSL(const double x, const double y, const double phi, RSPParam* param);
  void LSR(const double x, const double y, const double phi, RSPParam* param);
  void LRL(const double x, const double y, const double phi, RSPParam* param);
  void SLS(const double x, const double y, const double phi, RSPParam* param);
  void LRLRn(const double x, const double y, const double phi, RSPParam* param);
  void LRLRp(const double x, const double y, const double phi, RSPParam* param);
  void LRSR(const double x, const double y, const double phi, RSPParam* param);
  void LRSL(const double x, const double y, const double phi, RSPParam* param);
  void LRSLR(const double x, const double y, const double phi, RSPParam* param);
  
  // 计算tau和omega的函数
  std::pair<double, double> calc_tau_omega(const double u, const double v,
                                           const double xi, const double eta,
                                           const double phi);

  double max_kappa_; // 最大曲率
  double step_size_; // 步长
};
}