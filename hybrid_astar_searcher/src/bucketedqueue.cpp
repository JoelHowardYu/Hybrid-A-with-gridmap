#include "hybrid_astar_searcher/bucketedqueue.h"

#include "limits.h"
#include <stdio.h>
#include <stdlib.h>

// 定义静态成员变量sqrIndices，用于存储平方索引
std::vector<int> BucketPrioQueue::sqrIndices;
// 定义静态成员变量numBuckets，用于存储桶的数量
int BucketPrioQueue::numBuckets;

// BucketPrioQueue类的构造函数
BucketPrioQueue::BucketPrioQueue() {
  // 确保索引数组已创建
  if (sqrIndices.size()==0) initSqrIndices();
  // 初始化nextBucket为INT_MAX，表示下一个桶的索引
  nextBucket = INT_MAX;
    
  // 创建桶，桶的数量为numBuckets
  buckets = std::vector<std::queue<INTPOINT> >(numBuckets);

  // 重置元素计数器
  count = 0;
}

// 检查队列是否为空
bool BucketPrioQueue::empty() const {
  // 如果元素计数器为0，则队列为空
  return (count==0);
}

// 向队列中添加元素
void BucketPrioQueue::push(int prio, INTPOINT t) {
  // 检查优先级是否有效
  if (prio>=(int)sqrIndices.size()) {
    // 如果优先级无效，输出错误信息并退出程序
    fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
    exit(-1);
  }
  // 获取优先级对应的桶索引
  int id = sqrIndices[prio];
  // 检查桶索引是否有效
  if (id<0) {
    // 如果桶索引无效，输出错误信息并退出程序
    fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
    exit(-1);
  }
  // 将元素添加到对应的桶中
  buckets[id].push(t);
  // 如果当前桶索引小于nextBucket，更新nextBucket
  if (id<nextBucket) nextBucket = id;
  // 增加元素计数器
  count++;
}

// 从队列中弹出元素
INTPOINT BucketPrioQueue::pop() {
  int i;
  // 确保队列中有元素
  assert(count>0);
  // 从nextBucket开始遍历桶
  for (i = nextBucket; i<(int)buckets.size(); i++) {
    // 如果找到非空的桶，跳出循环
    if (!buckets[i].empty()) break;	
  }
  // 确保找到了非空的桶
  assert(i<(int)buckets.size());
  // 更新nextBucket为当前桶索引
  nextBucket = i;
  // 减少元素计数器
  count--;
  // 获取当前桶的第一个元素
  INTPOINT p = buckets[i].front();
  // 从当前桶中移除该元素
  buckets[i].pop();
  // 返回该元素
  return p;
}

// 初始化平方索引数组
void BucketPrioQueue::initSqrIndices() {
  // 创建一个大小为2*MAXDIST*MAXDIST+1的数组，初始值为-1
  sqrIndices = std::vector<int>(2*MAXDIST*MAXDIST+1, -1);

  int count=0;
  // 遍历所有可能的x和y值
  for (int x=0; x<=MAXDIST; x++) {
    for (int y=0; y<=x; y++) {
      // 计算平方和
      int sqr = x*x+y*y;
      // 将平方和对应的索引存入数组
      sqrIndices[sqr] = count++;
    }
  }
  // 设置桶的数量
  numBuckets = count;
}