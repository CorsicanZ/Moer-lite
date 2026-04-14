#include "Octree.h"
#include <queue>
struct Octree::OctreeNode {
    AABB boundingBox;
    std::shared_ptr<OctreeNode> subNodes[8];
    int primCount = -1;
    std::vector<int> primIdxBuffer;
};
Octree::OctreeNode * Octree::recursiveBuild(const AABB &aabb,
                       const std::vector<int> &primIdxBuffer, int depth) {
  //* todo 完成递归构建八叉树
  //* 构建方法请看实验手册
  //* 要注意的一种特殊是当节点的某个子包围盒和当前节点所有物体都相交，我们就不用细分了，当前节点作为叶子节点即可。
    auto node = new OctreeNode();
    node->boundingBox = aabb;

    if (depth >= ocMaxDepth || primIdxBuffer.size() <= ocLeafMaxSize) {
      node->primCount = static_cast<int>(primIdxBuffer.size());
      node->primIdxBuffer = primIdxBuffer;
      return node;
    }

    // Construct sub boxes which divide the current box into 8 sub boxes
    std::vector<AABB> subBoxes;
    subBoxes.reserve(8);

    const Point3f center = aabb.Center();
    for (int i = 0; i < 8; ++i) {
        const Point3f subMin(
            (i & 1) ? center[0] : aabb.pMin[0],
            (i & 2) ? center[1] : aabb.pMin[1],
            (i & 4) ? center[2] : aabb.pMin[2]
        );
        const Point3f subMax(
            (i & 1) ? aabb.pMax[0] : center[0],
            (i & 2) ? aabb.pMax[1] : center[1],
            (i & 4) ? aabb.pMax[2] : center[2]
        );
        subBoxes.emplace_back(subMin, subMax);
    }

    // Construct the subbuffers
    std::vector<std::vector<int>> subBuffers;
    subBuffers.resize(8);

    for (int i = 0; i < 8; ++i) {
        for (int index : primIdxBuffer) {
          if (shapes[index]->getAABB().Overlap(subBoxes[i])) {
            subBuffers[i].push_back(index);
          }
        }
    }

    // Stop splitting when a child box contains all primitives.
    for (int i = 0; i < 8; ++i) {
        if (subBuffers[i].size() == primIdxBuffer.size()) {
          node->primCount = static_cast<int>(primIdxBuffer.size());
          node->primIdxBuffer = primIdxBuffer;
          return node;
        }
    }

    // Actually construct the sub nodes
    for (int i = 0; i < 8; ++i) {
        if (subBuffers[i].size() == 0) {
          node->subNodes[i] = nullptr;
          continue;
        }
        node->subNodes[i].reset(recursiveBuild(subBoxes[i], subBuffers[i], depth + 1));
    }

    return node;
}
void Octree::build() {
  //* 首先计算整个场景的范围
  for (const auto & shape : shapes) {
    //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
    //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
    //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
    //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
    shape->initInternalAcceleration();

    boundingBox.Expand(shape->getAABB());
  }

  //* 构建八叉树
  std::vector<int> primIdxBuffer(shapes.size());
  std::iota(primIdxBuffer.begin(), primIdxBuffer.end(), 0);
  root = recursiveBuild(boundingBox, primIdxBuffer, 0);
}

bool Octree::rayIntersect(Ray &ray, int *geomID, int *primID,
                          float *u, float *v) const {
  if (root == nullptr) {
    return false;
  }

  bool hit = false;
  std::queue<const OctreeNode *> nodeQueue;
  nodeQueue.push(root);

  while (!nodeQueue.empty()) {
    const OctreeNode *node = nodeQueue.front();
    nodeQueue.pop();

    if (node == nullptr || !node->boundingBox.RayIntersect(ray)) {
      continue;
    }

    // Leaf node: test all primitives stored in this node.
    if (node->primCount >= 0) {
      for (int i = 0; i < node->primCount; ++i) {
        const int shapeIdx = node->primIdxBuffer[i];
        if (shapes[shapeIdx]->rayIntersectShape(ray, primID, u, v)) {
          *geomID = shapes[shapeIdx]->geometryID;
          hit = true;
        }
      }
      continue;
    }

    // Internal node: continue traversal.
    for (const auto &subNode : node->subNodes) {
      if (subNode) {
        nodeQueue.push(subNode.get());
      }
    }
  }

  return hit;
}