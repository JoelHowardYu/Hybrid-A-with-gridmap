#pragma once  // 确保头文件只被包含一次

#include <Eigen/Core>  // 包含Eigen库的核心功能

namespace planning {  // 定义一个名为planning的命名空间
    typedef Eigen::Vector2d Vec2d;  // 定义一个类型别名Vec2d，表示二维双精度浮点数向量
    typedef Eigen::Vector3d Vec3d;  // 定义一个类型别名Vec3d，表示三维双精度浮点数向量
    typedef Eigen::VectorXd VecXd;  // 定义一个类型别名VecXd，表示可变维度的双精度浮点数向量
    typedef Eigen::Vector2i Vec2i;  // 定义一个类型别名Vec2i，表示二维整数向量
    typedef Eigen::Vector3i Vec3i;  // 定义一个类型别名Vec3i，表示三维整数向量

    typedef Eigen::Array2i Arr2i;  // 定义一个类型别名Arr2i，表示二维整数数组
    typedef Eigen::Array3i Arr3i;  // 定义一个类型别名Arr3i，表示三维整数数组

    typedef Eigen::Matrix2d Mat2d;  // 定义一个类型别名Mat2d，表示2x2双精度浮点数矩阵
    typedef Eigen::Matrix3d Mat3d;  // 定义一个类型别名Mat3d，表示3x3双精度浮点数矩阵
    typedef Eigen::MatrixXd MatXd;  // 定义一个类型别名MatXd，表示可变维度的双精度浮点数矩阵
}