#include "hybrid_astar_searcher/dynamicvoronoi.h"

#include <math.h>
#include <iostream>

// 构造函数，初始化类成员变量
DynamicVoronoi::DynamicVoronoi() {
  sqrt2 = sqrt(2.0); // 计算sqrt(2)并存储
  data = NULL; // 初始化数据指针为空
  gridMap = NULL; // 初始化网格地图指针为空
}

// 析构函数，释放内存
DynamicVoronoi::~DynamicVoronoi() {
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x]; // 释放每一行的数据
    delete[] data; // 释放数据数组
  }
  if (gridMap) {
    for (int x=0; x<sizeX; x++) delete[] gridMap[x]; // 释放每一行的网格地图
    delete[] gridMap; // 释放网格地图数组
  }
}

// 初始化空的地图
void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY, bool initGridMap) {
  sizeX = _sizeX; // 设置地图的宽度
  sizeY = _sizeY; // 设置地图的高度
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x]; // 如果已有数据，先释放
    delete[] data;
  }
  data = new dataCell*[sizeX]; // 分配新的数据数组
  for (int x=0; x<sizeX; x++) data[x] = new dataCell[sizeY]; // 为每一行分配数据

  if (initGridMap) { // 如果需要初始化网格地图
    if (gridMap) {
      for (int x=0; x<sizeX; x++) delete[] gridMap[x]; // 如果已有网格地图，先释放
      delete[] gridMap;
    }
    gridMap = new bool*[sizeX]; // 分配新的网格地图数组
    for (int x=0; x<sizeX; x++) gridMap[x] = new bool[sizeY]; // 为每一行分配网格地图
  }
  
  dataCell c; // 创建一个默认的数据单元
  c.dist = INFINITY; // 设置距离为无穷大
  c.sqdist = INT_MAX; // 设置平方距离为最大整数值
  c.obstX = invalidObstData; // 设置障碍物的X坐标为无效值
  c.obstY = invalidObstData; // 设置障碍物的Y坐标为无效值
  c.voronoi = free; // 设置Voronoi状态为自由
  c.queueing = fwNotQueued; // 设置队列状态为未排队
  c.needsRaise = false; // 设置需要提升标志为假

  for (int x=0; x<sizeX; x++)
    for (int y=0; y<sizeY; y++) data[x][y] = c; // 初始化所有数据单元

  if (initGridMap) { // 如果需要初始化网格地图
    for (int x=0; x<sizeX; x++) 
      for (int y=0; y<sizeY; y++) gridMap[x][y] = 0; // 初始化所有网格地图单元为0
  }
}

// 初始化地图并加载障碍物信息
void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap) {
  gridMap = _gridMap; // 设置网格地图
  initializeEmpty(_sizeX, _sizeY, false); // 初始化空的地图

  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      if (gridMap[x][y]) { // 如果该位置是障碍物
        dataCell c = data[x][y]; // 获取当前数据单元
        if (!isOccupied(x,y,c)) { // 如果该位置没有被占用
          
          bool isSurrounded = true; // 假设该障碍物被包围
          for (int dx=-1; dx<=1; dx++) { // 检查周围8个位置
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) continue; // 如果超出边界，跳过
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue; // 如果是当前位置，跳过
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1) continue; // 如果超出边界，跳过

              if (!gridMap[nx][ny]) { // 如果周围有非障碍物
                isSurrounded = false; // 该障碍物未被完全包围
                break;
              }
            }
          }
          if (isSurrounded) { // 如果该障碍物被完全包围
            c.obstX = x; // 设置障碍物的X坐标
            c.obstY = y; // 设置障碍物的Y坐标
            c.sqdist = 0; // 设置平方距离为0
            c.dist=0; // 设置距离为0
            c.voronoi=occupied; // 设置Voronoi状态为占用
            c.queueing = fwProcessed; // 设置队列状态为已处理
            data[x][y] = c; // 更新数据单元
          } else setObstacle(x,y); // 否则设置为障碍物
        }
      }
    }
  }
}

// 占用一个单元格
void DynamicVoronoi::occupyCell(int x, int y) {
  gridMap[x][y] = 1; // 设置该位置为障碍物
  setObstacle(x,y); // 设置为障碍物
}

// 清除一个单元格
void DynamicVoronoi::clearCell(int x, int y) {
  gridMap[x][y] = 0; // 清除该位置的障碍物
  removeObstacle(x,y); // 移除障碍物
}

// 设置障碍物
void DynamicVoronoi::setObstacle(int x, int y) {
  dataCell c = data[x][y]; // 获取当前数据单元
  if(isOccupied(x,y,c)) return; // 如果该位置已被占用，直接返回
  
  addList.push_back(INTPOINT(x,y)); // 将该位置添加到添加列表
  c.obstX = x; // 设置障碍物的X坐标
  c.obstY = y; // 设置障碍物的Y坐标
  data[x][y] = c; // 更新数据单元
}

// 移除障碍物
void DynamicVoronoi::removeObstacle(int x, int y) {
  dataCell c = data[x][y]; // 获取当前数据单元
  if(isOccupied(x,y,c) == false) return; // 如果该位置未被占用，直接返回

  removeList.push_back(INTPOINT(x,y)); // 将该位置添加到移除列表
  c.obstX = invalidObstData; // 设置障碍物的X坐标为无效值
  c.obstY  = invalidObstData; // 设置障碍物的Y坐标为无效值    
  c.queueing = bwQueued; // 设置队列状态为反向排队
  data[x][y] = c; // 更新数据单元
}

// 交换障碍物
void DynamicVoronoi::exchangeObstacles(const std::vector<INTPOINT>& points) {

  for (unsigned int i=0; i<lastObstacles.size(); i++) { // 遍历上一次的障碍物列表
    int x = lastObstacles[i].x;
    int y = lastObstacles[i].y;

    bool v = gridMap[x][y]; // 获取该位置的状态
    if (v) continue; // 如果该位置仍是障碍物，跳过
    removeObstacle(x,y); // 移除该位置的障碍物
  }  

  lastObstacles.clear(); // 清空上一次的障碍物列表
  lastObstacles.reserve(points.size()); // 预留空间

  for (unsigned int i=0; i<points.size(); i++) { // 遍历新的障碍物列表
    int x = points[i].x;
    int y = points[i].y;
    bool v = gridMap[x][y]; // 获取该位置的状态
    if (v) continue; // 如果该位置已是障碍物，跳过
    setObstacle(x,y); // 设置该位置为障碍物
    lastObstacles.push_back(points[i]); // 将该位置添加到上一次的障碍物列表
  }  
}

// 更新Voronoi图
void DynamicVoronoi::update(bool updateRealDist) {

  commitAndColorize(updateRealDist); // 提交并着色

  while (!open.empty()) { // 当开放列表不为空时
    INTPOINT p = open.pop(); // 弹出开放列表中的一个点
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y]; // 获取当前数据单元

    if(c.queueing==fwProcessed) continue; // 如果该位置已处理，跳过

    if (c.needsRaise) { // 如果需要提升
      // RAISE
      for (int dx=-1; dx<=1; dx++) { // 检查周围8个位置
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue; // 如果超出边界，跳过
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue; // 如果是当前位置，跳过
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue; // 如果超出边界，跳过
          dataCell nc = data[nx][ny]; // 获取相邻位置的数据单元
          if (nc.obstX!=invalidObstData && !nc.needsRaise) { // 如果相邻位置有障碍物且不需要提升
            if(!isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])) { // 如果相邻位置未被占用
              open.push(nc.sqdist, INTPOINT(nx,ny)); // 将相邻位置加入开放列表
              nc.queueing = fwQueued; // 设置相邻位置的队列状态为正向排队
              nc.needsRaise = true; // 设置相邻位置需要提升
              nc.obstX = invalidObstData; // 设置相邻位置的障碍物X坐标为无效值
              nc.obstY = invalidObstData; // 设置相邻位置的障碍物Y坐标为无效值
              if (updateRealDist) nc.dist = INFINITY; // 如果需要更新实际距离，设置为无穷大
              nc.sqdist = INT_MAX; // 设置相邻位置的平方距离为最大整数值
              data[nx][ny] = nc; // 更新相邻位置的数据单元
            } else {
              if(nc.queueing != fwQueued){ // 如果相邻位置未排队
                open.push(nc.sqdist, INTPOINT(nx,ny)); // 将相邻位置加入开放列表
                nc.queueing = fwQueued; // 设置相邻位置的队列状态为正向排队
                data[nx][ny] = nc; // 更新相邻位置的数据单元
              }
            }      
          }
        }
      }
      c.needsRaise = false; // 设置当前位置不需要提升
      c.queueing = bwProcessed; // 设置当前位置的队列状态为反向已处理
      data[x][y] = c; // 更新当前位置的数据单元
    }
    else if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,data[c.obstX][c.obstY])) { // 如果当前位置有障碍物且被占用

      // LOWER
      c.queueing = fwProcessed; // 设置当前位置的队列状态为正向已处理
      c.voronoi = occupied; // 设置当前位置的Voronoi状态为占用

      for (int dx=-1; dx<=1; dx++) { // 检查周围8个位置
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue; // 如果超出边界，跳过
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue; // 如果是当前位置，跳过
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue; // 如果超出边界，跳过
          dataCell nc = data[nx][ny]; // 获取相邻位置的数据单元
          if(!nc.needsRaise) { // 如果相邻位置不需要提升
            int distx = nx-c.obstX; // 计算相邻位置到障碍物的X距离
            int disty = ny-c.obstY; // 计算相邻位置到障碍物的Y距离
            int newSqDistance = distx*distx + disty*disty; // 计算新的平方距离
            bool overwrite =  (newSqDistance < nc.sqdist); // 判断是否需要覆盖
            if(!overwrite && newSqDistance==nc.sqdist) { // 如果新的平方距离等于旧的平方距离
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])==false) overwrite = true; // 如果相邻位置没有障碍物或未被占用，需要覆盖
            }
            if (overwrite) { // 如果需要覆盖
              open.push(newSqDistance, INTPOINT(nx,ny)); // 将相邻位置加入开放列表
              nc.queueing = fwQueued; // 设置相邻位置的队列状态为正向排队
              if (updateRealDist) { // 如果需要更新实际距离
                nc.dist = sqrt((double) newSqDistance); // 计算实际距离
              }
              nc.sqdist = newSqDistance; // 设置相邻位置的平方距离
              nc.obstX = c.obstX; // 设置相邻位置的障碍物X坐标
              nc.obstY = c.obstY; // 设置相邻位置的障碍物Y坐标
            } else { 
              checkVoro(x,y,nx,ny,c,nc); // 检查Voronoi图
            }
            data[nx][ny] = nc; // 更新相邻位置的数据单元
          }
        }
      }
    }
    data[x][y] = c; // 更新当前位置的数据单元
  }
}

// 获取某个位置的距离
float DynamicVoronoi::getDistance( int x, int y ) const {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[x][y].dist; // 如果位置在地图范围内，返回该位置的距离
  else return -INFINITY; // 否则返回负无穷大
}

// 判断某个位置是否在Voronoi图中
bool DynamicVoronoi::isVoronoi( int x, int y ) const {
  dataCell c = data[x][y]; // 获取当前数据单元
  
  return (c.voronoi==free || c.voronoi==voronoiKeep); // 如果Voronoi状态为自由或保留，返回真
}

// 提交并着色
void DynamicVoronoi::commitAndColorize(bool updateRealDist) {
  // ADD NEW OBSTACLES
  for (unsigned int i=0; i<addList.size(); i++) { // 遍历添加列表
    INTPOINT p = addList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y]; // 获取当前数据单元

    if(c.queueing != fwQueued){ // 如果该位置未排队
      if (updateRealDist) c.dist = 0; // 如果需要更新实际距离，设置为0
      c.sqdist = 0; // 设置平方距离为0
      c.obstX = x; // 设置障碍物的X坐标
      c.obstY = y; // 设置障碍物的Y坐标
      c.queueing = fwQueued; // 设置队列状态为正向排队
      c.voronoi = occupied; // 设置Voronoi状态为占用
      data[x][y] = c; // 更新数据单元
      open.push(0, INTPOINT(x,y)); // 将该位置加入开放列表
    }
  }// REMOVE OLD OBSTACLES
  for (unsigned int i=0; i<removeList.size(); i++) { // 遍历移除列表
    INTPOINT p = removeList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y]; // 获取当前数据单元

    if (isOccupied(x,y,c)==true) continue; // 如果该位置仍是障碍物，跳过
    open.push(0, INTPOINT(x,y)); // 将该位置加入开放列表
    if (updateRealDist) c.dist  = INFINITY; // 如果需要更新实际距离，设置为无穷大
    c.sqdist = INT_MAX; // 设置平方距离为最大整数值
    c.needsRaise = true; // 设置需要提升标志为真
    data[x][y] = c; // 更新数据单元
  }
  removeList.clear(); // 清空移除列表
  addList.clear(); // 清空添加列表
}

// 检查Voronoi图
void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc) {

  if ((c.sqdist>1 || nc.sqdist>1) && nc.obstX!=invalidObstData) { // 如果当前位置或相邻位置的平方距离大于1且相邻位置有障碍物
    if (abs(c.obstX-nc.obstX) > 1 || abs(c.obstY-nc.obstY) > 1) { // 如果当前位置和相邻位置的障碍物距离大于1
      //compute dist from x,y to obstacle of nx,ny	 
      int dxy_x = x-nc.obstX; // 计算当前位置到相邻位置障碍物的X距离
      int dxy_y = y-nc.obstY; // 计算当前位置到相邻位置障碍物的Y距离
      int sqdxy = dxy_x*dxy_x + dxy_y*dxy_y; // 计算平方距离
      int stability_xy = sqdxy - c.sqdist; // 计算稳定性
      if (sqdxy - c.sqdist<0) return; // 如果稳定性小于0，返回

      //compute dist from nx,ny to obstacle of x,y
      int dnxy_x = nx - c.obstX; // 计算相邻位置到当前位置障碍物的X距离
      int dnxy_y = ny - c.obstY; // 计算相邻位置到当前位置障碍物的Y距离
      int sqdnxy = dnxy_x*dnxy_x + dnxy_y*dnxy_y; // 计算平方距离
      int stability_nxy = sqdnxy - nc.sqdist; // 计算稳定性
      if (sqdnxy - nc.sqdist <0) return; // 如果稳定性小于0，返回

      //which cell is added to the Voronoi diagram?
      if(stability_xy <= stability_nxy && c.sqdist>2) { // 如果当前位置的稳定性小于等于相邻位置的稳定性且当前位置的平方距离大于2
        if (c.voronoi != free) { // 如果当前位置的Voronoi状态不是自由
          c.voronoi = free; // 设置当前位置的Voronoi状态为自由
          reviveVoroNeighbors(x,y); // 恢复当前位置的Voronoi邻居
          pruneQueue.push(INTPOINT(x,y)); // 将当前位置加入修剪队列
        }
      }
      if(stability_nxy <= stability_xy && nc.sqdist>2) { // 如果相邻位置的稳定性小于等于当前位置的稳定性且相邻位置的平方距离大于2
        if (nc.voronoi != free) { // 如果相邻位置的Voronoi状态不是自由
          nc.voronoi = free; // 设置相邻位置的Voronoi状态为自由
          reviveVoroNeighbors(nx,ny); // 恢复相邻位置的Voronoi邻居
          pruneQueue.push(INTPOINT(nx,ny)); // 将相邻位置加入修剪队列
        }
      }
    }
  }
}

// 恢复Voronoi邻居
void DynamicVoronoi::reviveVoroNeighbors(int &x, int &y) {
  for (int dx=-1; dx<=1; dx++) { // 检查周围8个位置
    int nx = x+dx;
    if (nx<=0 || nx>=sizeX-1) continue; // 如果超出边界，跳过
    for (int dy=-1; dy<=1; dy++) {
      if (dx==0 && dy==0) continue; // 如果是当前位置，跳过
      int ny = y+dy;
      if (ny<=0 || ny>=sizeY-1) continue; // 如果超出边界，跳过
      dataCell nc = data[nx][ny]; // 获取相邻位置的数据单元
      if (nc.sqdist != INT_MAX && !nc.needsRaise && (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) { // 如果相邻位置的平方距离不是最大值且不需要提升且Voronoi状态为保留或修剪
        nc.voronoi = free; // 设置相邻位置的Voronoi状态为自由
        data[nx][ny] = nc; // 更新相邻位置的数据单元
        pruneQueue.push(INTPOINT(nx,ny)); // 将相邻位置加入修剪队列
      }
    }
  }
}

// 判断某个位置是否被占用
bool DynamicVoronoi::isOccupied(int x, int y) const {
  dataCell c = data[x][y]; // 获取当前数据单元
  return (c.obstX==x && c.obstY==y); // 如果障碍物的X和Y坐标与当前位置相同，返回真
}

// 判断某个位置是否被占用
bool DynamicVoronoi::isOccupied(int &x, int &y, dataCell &c) { 
  return (c.obstX==x && c.obstY==y); // 如果障碍物的X和Y坐标与当前位置相同，返回真
}

// 可视化Voronoi图
void DynamicVoronoi::visualize(const char *filename) {
  // write pgm files
  std::cout << "keshihua" << std::endl; // 输出“可视化”
  FILE* F = fopen(filename, "w"); // 打开文件
  if (!F) {
    std::cerr << "could not open 'result.pgm' for writing!\n"; // 如果文件打开失败，输出错误信息
    return;
  }
  fprintf(F, "P6\n"); // 写入文件头
  fprintf(F, "%d %d 255\n", sizeX, sizeY); // 写入图像大小

  for(int y = sizeY-1; y >=0; y--){ // 遍历每一行
    for(int x = 0; x<sizeX; x++){ // 遍历每一列	
      unsigned char c = 0;
      if (isVoronoi(x,y)) { // 如果该位置在Voronoi图中
        fputc( 255, F ); // 写入红色
        fputc( 0, F );
        fputc( 0, F );
      } else if (data[x][y].sqdist==0) { // 如果该位置是障碍物
        fputc( 0, F ); // 写入黑色
        fputc( 0, F );
        fputc( 0, F );
      } 
      else {
        float f = 80+(data[x][y].dist*5); // 计算灰度值
        if (f>255) f=255; // 如果灰度值大于255，设置为255
        if (f<0) f=0; // 如果灰度值小于0，设置为0
        c = (unsigned char)f;
        fputc( c, F ); // 写入灰度值
        fputc( c, F );
        fputc( c, F );
      }
    }
  }
  fclose(F); // 关闭文件
}

// 修剪Voronoi图
void DynamicVoronoi::prune() {
  // filler
  while(!pruneQueue.empty()) { // 当修剪队列不为空时
    INTPOINT p = pruneQueue.front(); // 获取修剪队列中的第一个点
    pruneQueue.pop(); // 弹出修剪队列中的第一个点
    int x = p.x;
    int y = p.y;

    if (data[x][y].voronoi==occupied) continue; // 如果该位置被占用，跳过
    if (data[x][y].voronoi==freeQueued) continue; // 如果该位置已排队，跳过

    data[x][y].voronoi = freeQueued; // 设置该位置的Voronoi状态为自由排队
    open.push(data[x][y].sqdist, p); // 将该位置加入开放列表

    /* tl t tr
       l c r
       bl b br */

    dataCell tr,tl,br,bl;
    tr = data[x+1][y+1]; // 获取右上角的数据单元
    tl = data[x-1][y+1]; // 获取左上角的数据单元
    br = data[x+1][y-1]; // 获取右下角的数据单元
    bl = data[x-1][y-1]; // 获取左下角的数据单元

    dataCell r,b,t,l;
    r = data[x+1][y]; // 获取右边的数据单元
    l = data[x-1][y]; // 获取左边的数据单元
    t = data[x][y+1]; // 获取上边的数据单元
    b = data[x][y-1]; // 获取下边的数据单元

    if (x+2<sizeX && r.voronoi==occupied) { // 如果右边第二个位置在地图范围内且被占用
      // fill to the right
      if (tr.voronoi!=occupied && br.voronoi!=occupied && data[x+2][y].voronoi!=occupied) { // 如果右上角和右下角未被占用且右边第二个位置未被占用
        r.voronoi = freeQueued; // 设置右边的Voronoi状态为自由排队
        open.push(r.sqdist, INTPOINT(x+1,y)); // 将右边位置加入开放列表
        data[x+1][y] = r; // 更新右边的数据单元
      }
    } 
    if (x-2>=0 && l.voronoi==occupied) { // 如果左边第二个位置在地图范围内且被占用
      // fill to the left
      if (tl.voronoi!=occupied && bl.voronoi!=occupied && data[x-2][y].voronoi!=occupied) { // 如果左上角和左下角未被占用且左边第二个位置未被占用
        l.voronoi = freeQueued; // 设置左边的Voronoi状态为自由排队
        open.push(l.sqdist, INTPOINT(x-1,y)); // 将左边位置加入开放列表
        data[x-1][y] = l; // 更新左边的数据单元
      }
    } 
    if (y+2<sizeY && t.voronoi==occupied) { // 如果上边第二个位置在地图范围内且被占用
      // fill to the top
      if (tr.voronoi!=occupied && tl.voronoi!=occupied && data[x][y+2].voronoi!=occupied) { // 如果右上角和左上角未被占用且上边第二个位置未被占用
        t.voronoi = freeQueued; // 设置上边的Voronoi状态为自由排队
        open.push(t.sqdist, INTPOINT(x,y+1)); // 将上边位置加入开放列表
        data[x][y+1] = t; // 更新上边的数据单元
      }
    } 
    if (y-2>=0 && b.voronoi==occupied) { // 如果下边第二个位置在地图范围内且被占用
      // fill to the bottom
      if (br.voronoi!=occupied && bl.voronoi!=occupied && data[x][y-2].voronoi!=occupied) { // 如果右下角和左下角未被占用且下边第二个位置未被占用
        b.voronoi = freeQueued; // 设置下边的Voronoi状态为自由排队
        open.push(b.sqdist, INTPOINT(x,y-1)); // 将下边位置加入开放列表
        data[x][y-1] = b; // 更新下边的数据单元
      }
    } 
  }


  while(!open.empty()) { // 当开放列表不为空时
    INTPOINT p = open.pop(); // 弹出开放列表中的一个点
    dataCell c = data[p.x][p.y]; // 获取当前数据单元
    int v = c.voronoi;
    if (v!=freeQueued && v!=voronoiRetry) { // 如果Voronoi状态不是自由排队且不是重试
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(p.x,p.y); // 匹配标记
    if (r==pruned) c.voronoi = voronoiPrune; // 如果结果是修剪，设置Voronoi状态为修剪
    else if (r==keep) c.voronoi = voronoiKeep; // 如果结果是保留，设置Voronoi状态为保留
    else { // r==retry
      c.voronoi = voronoiRetry; // 设置Voronoi状态为重试
      //      printf("RETRY %d %d\n", x, sizeY-1-y);
      pruneQueue.push(p); // 将该位置加入修剪队列
    }
    data[p.x][p.y] = c; // 更新数据单元

    if (open.empty()) { // 如果开放列表为空
      while (!pruneQueue.empty()) { // 当修剪队列不为空时
        INTPOINT p = pruneQueue.front(); // 获取修剪队列中的第一个点
        pruneQueue.pop(); // 弹出修剪队列中的第一个点
        open.push(data[p.x][p.y].sqdist, p); // 将该位置加入开放列表
      }
    }
  }
  //  printf("match: %d\nnomat: %d\n", matchCount, noMatchCount);
}

// 匹配标记
DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];

  int nx, ny;
  int dx, dy;

  int i=0;
  int count=0;
  //  int obstacleCount=0;
  int voroCount=0;
  int voroCountFour=0;

  for (dy=1; dy>=-1; dy--) { // 检查周围8个位置
    ny = y+dy;
    for (dx=-1; dx<=1; dx++) {
      if (dx || dy) {
        nx = x+dx;
        dataCell nc = data[nx][ny]; // 获取相邻位置的数据单元
        int v = nc.voronoi;
        bool b = (v<=free && v!=voronoiPrune); // 判断相邻位置是否在Voronoi图中
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (b) {
          voroCount++; // 统计Voronoi图中的位置数量
          if (!(dx && dy)) voroCountFour++; // 统计四连通Voronoi图中的位置数量
        }
        if (b && !(dx && dy) ) count++; // 统计四连通Voronoi图中的位置数量
        //	if (v<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }
  if (voroCount<3 && voroCountFour==1 && (f[1] || f[3] || f[4] || f[6])) { // 如果Voronoi图中的位置数量小于3且四连通Voronoi图中的位置数量为1且满足条件
    //    assert(voroCount<2);
    //    if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep; // 返回保留
  }

  // 4-connected 表示4连通的情况
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4])) return keep;
  // 如果满足以下任意一种情况，则返回keep：
  // 1. f[0]为假且f[1]和f[3]为真
  // 2. f[2]为假且f[1]和f[4]为真
  // 3. f[5]为假且f[3]和f[6]为真
  // 4. f[7]为假且f[6]和f[4]为真

  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4])) return keep;
  // 如果满足以下任意一种情况，则返回keep：
  // 1. f[3]和f[4]为真且f[1]和f[6]为假
  // 2. f[1]和f[6]为真且f[3]和f[4]为假

  // 如果voroCount大于等于5且voroCountFour大于等于3，并且当前的voronoi状态不等于voronoiRetry，则返回retry
  // 这表示需要保留voronoi单元格在块内部，并在稍后重试
  if (voroCount>=5 && voroCountFour>=3 && data[x][y].voronoi!=voronoiRetry) {
      return retry;
  }

  // 如果以上条件都不满足，则返回pruned
  return pruned;
}