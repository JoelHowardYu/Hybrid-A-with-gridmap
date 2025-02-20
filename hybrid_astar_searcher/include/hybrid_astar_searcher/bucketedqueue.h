#ifndef _PRIORITYQUEUE2_H_
#define _PRIORITYQUEUE2_H_

// 定义最大距离为1000
#define MAXDIST 1000
// 预留64个空间
#define RESERVE 64

#include <vector>
#include <set>
#include <queue>
#include <assert.h>
#include "hybrid_astar_searcher/point.h"

//! 用于整数坐标的优先队列，优先级为平方距离。
/** 一个使用桶来分组具有相同优先级的元素的优先队列。
    每个桶内的元素是无序的，这在大规模分组时提高了效率。
    元素被假设为整数坐标，优先级被假设为平方欧几里得距离（整数）。
*/
class BucketPrioQueue {

 public:
  //! 标准构造函数
  /** 标准构造函数。首次调用时会创建一个查找表，
      该表将平方距离映射到桶号，这可能需要一些时间...
  */
  BucketPrioQueue();
  //! 检查队列是否为空
  bool empty() const;
  //! 将一个元素推入队列
  void push(int prio, INTPOINT t);
  //! 返回并弹出具有最小平方距离的元素
  INTPOINT pop();

 private:

  // 初始化平方距离索引
  static void initSqrIndices();
  // 平方距离索引表
  static std::vector<int> sqrIndices;
  // 桶的数量
  static int numBuckets;
  // 元素计数
  int count;
  // 下一个桶的索引
  int nextBucket;

  // 存储桶的向量，每个桶是一个队列
  std::vector<std::queue<INTPOINT> > buckets;
};

#endif