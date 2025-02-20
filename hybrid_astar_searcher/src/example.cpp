#include "dynamicvoronoi.h"

#include <iostream>
#include <fstream>
#include <string.h>

// 加载PGM格式的地图文件，并将其转换为布尔类型的二维数组
void loadPGM( std::istream &is, int *sizeX, int *sizeY, bool ***map ) {
  std::string tag;
  is >> tag;
  // 检查PGM文件的标识符是否为"P5"
  if (tag!="P5") {
    std::cerr << "Awaiting 'P5' in pgm header, found " << tag << std::endl;
    exit(-1);
  }
  
  // 跳过空白字符和注释行
  while (is.peek()==' ' || is.peek()=='\n') is.ignore();
  while (is.peek()=='#') is.ignore(255, '\n');
  is >> *sizeX;
  while (is.peek()=='#') is.ignore(255, '\n');
  is >> *sizeY;
  while (is.peek()=='#') is.ignore(255, '\n');
  is >> tag;
  // 检查PGM文件的灰度值是否为"255"
  if (tag!="255") {
    std::cerr << "Awaiting '255' in pgm header, found " << tag << std::endl;
    exit(-1);
  }
  is.ignore(255, '\n');  
  
  // 分配二维布尔数组内存
  *map = new bool*[*sizeX];

  for (int x=0; x<*sizeX; x++) {
    (*map)[x] = new bool[*sizeY];
  }
  // 从下到上读取PGM文件的像素值，并将其转换为布尔值
  for (int y=*sizeY-1; y>=0; y--) {
    for (int x=0; x<*sizeX; x++) {
      int c = is.get();
      if ((double)c<255-255*0.2) (*map)[x][y] = true; // 如果像素值小于204，则认为该单元格被占用
      else (*map)[x][y] = false; // 否则认为该单元格是空闲的
      if (!is.good()) {
        std::cerr << "Error reading pgm map.\n";
        exit(-1);
      }
    }
  }
}


int main( int argc, char *argv[] ) {

  // 检查命令行参数是否正确
  if(argc<2 || argc>3 || (argc==3 && !(strcmp(argv[2],"prune")==0 || strcmp(argv[2],"pruneAlternative")==0))) {
    std::cerr<<"usage: "<<argv[0]<<" <pgm map> [prune|pruneAlternative]\n";
    exit(-1);
  }
  
  bool doPrune = false;
  bool doPruneAlternative = false;
  // 如果命令行参数中有"prune"，则设置doPrune为true
  if (argc==3) doPrune = true;

  // 如果命令行参数中有"pruneAlternative"，则设置doPruneAlternative为true
  if(doPrune && strcmp(argv[2],"pruneAlternative")==0){
    doPrune = false;
    doPruneAlternative = true;
  }


  // 加载PGM地图并初始化Voronoi图
  std::ifstream is(argv[1]);
  if (!is) {
    std::cerr << "Could not open map file for reading.\n";
    exit(-1);
  }
  
  bool **map=NULL;
  int sizeX, sizeY;
  loadPGM( is, &sizeX, &sizeY, &map );
  is.close();
  fprintf(stderr, "Map loaded (%dx%d).\n", sizeX, sizeY);

  // 创建Voronoi对象并使用加载的地图进行初始化
  DynamicVoronoi voronoi;
  voronoi.initializeMap(sizeX, sizeY, map);
  voronoi.update(); // 更新距离图和Voronoi图
  if (doPrune) voronoi.prune();  // 如果需要，修剪Voronoi图
  // if (doPruneAlternative) voronoi.updateAlternativePrunedDiagram();  // 如果需要，使用另一种方法修剪Voronoi图

  voronoi.visualize("initial.ppm");
  std::cerr << "Generated initial frame.\n";

  // 现在执行一些带有随机障碍物的更新
  char filename[20];
  int numPoints = 10 + sizeX*sizeY*0.005;
  for (int frame=1; frame<=10; frame++) {
    std::vector<IntPoint> newObstacles;
    for (int i=0; i<numPoints; i++) {
      double x = 2+rand()/(double)RAND_MAX*(sizeX-4);
      double y = 2+rand()/(double)RAND_MAX*(sizeY-4);
      newObstacles.push_back(IntPoint(x,y));
    }
    voronoi.exchangeObstacles(newObstacles); // 注册新的障碍物（旧的障碍物将被移除）
    voronoi.update();
    if (doPrune) voronoi.prune();
    sprintf(filename, "update_%03d.ppm", frame);
    voronoi.visualize(filename);
    std::cerr << "Performed update with random obstacles.\n";
  }

  // 现在移除所有随机障碍物
  // final.pgm应该与initial.pgm非常相似，除了一些模糊的区域
  std::vector<IntPoint> empty;
  voronoi.exchangeObstacles(empty);
  voronoi.update();
  if (doPrune) voronoi.prune();
  voronoi.visualize("final.ppm");
  std::cerr << "Done with final update (all random obstacles removed).\n";
  std::cerr << "Check initial.ppm, update_???.ppm and final.ppm.\n";
  return 0;
}