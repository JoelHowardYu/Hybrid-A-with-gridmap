#ifndef _DYNAMICVORONOI_H_
#define _DYNAMICVORONOI_H_

// 包含必要的头文件
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>

// 包含自定义的BucketQueue头文件
#include "hybrid_astar_searcher/bucketedqueue.h"

// 定义一个名为DynamicVoronoi的类，用于计算和更新距离图和Voronoi图
class DynamicVoronoi {

 public:

  // 构造函数
  DynamicVoronoi();
  // 析构函数
  ~DynamicVoronoi();

  // 初始化一个空的地图
  void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap = true);
  // 使用给定的二值地图进行初始化（false==空闲，true==占用）
  void initializeMap(int _sizeX, int _sizeY, bool** _gridMap);

  // 在指定的单元格坐标处添加障碍物
  void occupyCell(int x, int y);
  // 在指定的单元格坐标处移除障碍物
  void clearCell(int x, int y);
  // 移除旧的动态障碍物并添加新的障碍物
  void exchangeObstacles(const std::vector<IntPoint> &newObstacles);

  // 更新距离图和Voronoi图以反映变化
  void update(bool updateRealDist = true);
  // 修剪Voronoi图
  void prune();

  // 返回指定位置的障碍物距离
  float getDistance(int x, int y) const;
  // 返回指定单元格是否是（修剪后的）Voronoi图的一部分
  bool isVoronoi(int x, int y) const;
  // 检查指定位置是否被占用
  bool isOccupied(int x, int y) const;
  // 将当前的距离图和Voronoi图写入ppm文件
  void visualize(const char* filename = "result.ppm");

  // 返回工作空间/地图的水平大小
  unsigned int getSizeX() const {return sizeX;}
  // 返回工作空间/地图的垂直大小
  unsigned int getSizeY() const {return sizeY;}

  // 原本是私有成员，为了访问obstX和obstY改为公有
 public:
  // 定义一个结构体dataCell，用于存储单元格的数据
  struct dataCell {
    float dist; // 距离
    char voronoi; // Voronoi图状态
    char queueing; // 队列状态
    int obstX; // 障碍物X坐标
    int obstY; // 障碍物Y坐标
    bool needsRaise; // 是否需要提升
    int sqdist; // 平方距离
  };

  // 定义一些枚举类型
  typedef enum {voronoiKeep = -4, freeQueued = -3, voronoiRetry = -2, voronoiPrune = -1, free = 0, occupied = 1} State;
  typedef enum {fwNotQueued = 1, fwQueued = 2, fwProcessed = 3, bwQueued = 4, bwProcessed = 1} QueueingState;
  typedef enum {invalidObstData = SHRT_MAX / 2} ObstDataState;
  typedef enum {pruned, keep, retry} markerMatchResult;

  // 方法声明
  void setObstacle(int x, int y);
  void removeObstacle(int x, int y);
  inline void checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc);
  void recheckVoro();
  void commitAndColorize(bool updateRealDist = true);
  inline void reviveVoroNeighbors(int& x, int& y);

  inline bool isOccupied(int& x, int& y, dataCell& c);
  inline markerMatchResult markerMatch(int x, int y);

  // 队列
  BucketPrioQueue open; // 开放队列
  std::queue<INTPOINT> pruneQueue; // 修剪队列

  std::vector<INTPOINT> removeList; // 移除列表
  std::vector<INTPOINT> addList; // 添加列表
  std::vector<INTPOINT> lastObstacles; // 最后的障碍物列表

  // 地图
  int sizeY; // 地图的垂直大小
  int sizeX; // 地图的水平大小
  dataCell** data; // 数据数组
  bool** gridMap; // 网格地图

  // 参数
  int padding; // 填充
  double doubleThreshold; // 双精度阈值

  double sqrt2; // 根号2

  // 获取数据的函数（注释掉）
  // dataCell** getData(){ return data; }
};

#endif