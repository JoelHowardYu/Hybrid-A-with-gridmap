#ifndef _VOROPOINT_H_
#define _VOROPOINT_H_

// 定义宏 INTPOINT 为 IntPoint
#define INTPOINT IntPoint

#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>

// 声明Cube结构体
struct Cube;

struct Cube
{     
      // 注释掉的原代码，表示立方体的8个顶点
      //Eigen::Vector3d p1, p2, p3, p4, p5, p6, p7, p8;   // the 8 vertex of a cube 
      
      // 使用Eigen::MatrixXd表示立方体的8个顶点
      Eigen::MatrixXd vertex;
      // 立方体的中心点
      Eigen::Vector3d center; // the center of the cube
      // 表示立方体是否有效，即是否应该被删除
      bool valid;    // indicates whether this cube should be deleted

      // 分配给立方体的时间
      double t; // time allocated to this cube
      // 表示立方体的边界框
      std::vector< std::pair<double, double> > box;
/*
           立方体的顶点示意图
           P4------------P3 
           /|           /|              ^
          / |          / |              | z
        P1--|---------P2 |              |
         |  P8--------|--p7             |
         | /          | /               /--------> y
         |/           |/               /  
        P5------------P6              / x
*/                                                                                 

      // 构造函数，使用8个顶点和中心点创建一个立方体
      Cube( Eigen::MatrixXd vertex_, Eigen::Vector3d center_)
      {
            vertex = vertex_;
            center = center_;
            valid = true;
            t = 0.0;
            box.resize(3);
      }

      // 设置立方体的顶点，用于创建一个内切于球的立方体
      void setVertex( Eigen::MatrixXd vertex_, double resolution_)
      {     
            vertex = vertex_;
            // 调整顶点的坐标以适应内切立方体的形状
            vertex(0,1) -= resolution_ / 2.0;
            vertex(3,1) -= resolution_ / 2.0;
            vertex(4,1) -= resolution_ / 2.0;
            vertex(7,1) -= resolution_ / 2.0;

            vertex(1,1) += resolution_ / 2.0;
            vertex(2,1) += resolution_ / 2.0;
            vertex(5,1) += resolution_ / 2.0;
            vertex(6,1) += resolution_ / 2.0;

            vertex(0,0) += resolution_ / 2.0;
            vertex(1,0) += resolution_ / 2.0;
            vertex(4,0) += resolution_ / 2.0;
            vertex(5,0) += resolution_ / 2.0;

            vertex(2,0) -= resolution_ / 2.0;
            vertex(3,0) -= resolution_ / 2.0;
            vertex(6,0) -= resolution_ / 2.0;
            vertex(7,0) -= resolution_ / 2.0;

            vertex(0,2) += resolution_ / 2.0;
            vertex(1,2) += resolution_ / 2.0;
            vertex(2,2) += resolution_ / 2.0;
            vertex(3,2) += resolution_ / 2.0;

            vertex(4,2) -= resolution_ / 2.0;
            vertex(5,2) -= resolution_ / 2.0;
            vertex(6,2) -= resolution_ / 2.0;
            vertex(7,2) -= resolution_ / 2.0;
            
            // 设置边界框
            setBox();
      }
      
      // 设置立方体的边界框
      void setBox()
      {
            box.clear();
            box.resize(3);
            // 设置x轴方向的边界
            box[0] = std::make_pair( vertex(3, 0), vertex(0, 0) );
            // 设置y轴方向的边界
            box[1] = std::make_pair( vertex(0, 1), vertex(1, 1) );
            // 设置z轴方向的边界
            box[2] = std::make_pair( vertex(4, 2), vertex(1, 2) );
      }

      // 打印立方体的中心点和顶点信息
      void printBox()
      {
            std::cout<<"center of the cube: \n"<<center<<std::endl;
            std::cout<<"vertex of the cube: \n"<<vertex<<std::endl;
      }

      // 默认构造函数
      Cube()
      {  
         center = Eigen::VectorXd::Zero(3);
         vertex = Eigen::MatrixXd::Zero(8, 3);

         valid = true;
         t = 0.0;
         box.resize(3);
      }

      // 析构函数
      ~Cube(){}
};


/*! 
 * 一个轻量级的整数点类，包含 x 和 y 两个字段
 */
class IntPoint {
 public:
  // 默认构造函数，初始化 x 和 y 为 0
  IntPoint() : x(0), y(0) {}
  
  // 带参数的构造函数，初始化 x 和 y 为传入的值
  IntPoint(int _x, int _y) : x(_x), y(_y) {}
  
  // 成员变量 x 和 y，表示点的坐标
  int x, y;
};

#endif