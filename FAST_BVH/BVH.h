#ifndef BVH_h
#define BVH_h

#include "BBox.h"
#include <vector>
#include <stdint.h>
#include "Object.h"
#include "IntersectionInfo.h"
#include "Ray.h"

//! Node descriptor for the flattened tree
struct BVHFlatNode {
  BBox bbox;
  //start是Node里面多个object开始的那个的索引
  //nPrims是object的数目
  //rightOffset是从它的父节点到该结点在flatNode的偏移
  uint32_t start, nPrims, rightOffset;
};

//! \author Brandon Pelfrey
//! A Bounding Volume Hierarchy system for fast Ray-Object intersection tests
class BVH {
  uint32_t nNodes, nLeafs, leafSize;
  //建立BVH的图元列表
  std::vector<Object*>* build_prims;

  //! Build the BVH tree out of build_prims
  void build();

  //把BVH展开为一维得到快速查询的树结构
  // Fast Traversal System
  BVHFlatNode *flatTree;

  public:
  BVH(std::vector<Object*>* objects, uint32_t leafSize=4);
  bool getIntersection(const Ray& ray, IntersectionInfo *intersection, bool occlusion) const ;

  ~BVH();
};

#endif
