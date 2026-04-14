#include "AABB.h"

Point3f minP(const Point3f &p1, const Point3f &p2) {
  return Point3f{std::min(p1[0], p2[0]), std::min(p1[1], p2[1]),
                 std::min(p1[2], p2[2])};
}

Point3f maxP(const Point3f &p1, const Point3f &p2) {
  return Point3f{std::max(p1[0], p2[0]), std::max(p1[1], p2[1]),
                 std::max(p1[2], p2[2])};
}

AABB AABB::Union(const AABB &other) const {
  Point3f min = minP(other.pMin, pMin), max = maxP(other.pMax, pMax);
  return AABB{min, max};
}

void AABB::Expand(const AABB &other) {
  pMin = minP(pMin, other.pMin);
  pMax = maxP(pMax, other.pMax);
}

AABB AABB::Union(const Point3f &other) const {
  Point3f min = minP(other, pMin), max = maxP(other, pMax);
  return AABB{min, max};
}

void AABB::Expand(const Point3f &other) {
  pMin = minP(pMin, other);
  pMax = maxP(pMax, other);
}

bool AABB::Overlap(const AABB &other) const {
  for (int dim = 0; dim < 3; ++dim) {
    if (pMin[dim] > other.pMax[dim] || pMax[dim] < other.pMin[dim]) {
      return false;
    }
  }
  return true;
}

bool AABB::RayIntersect(const Ray &ray, float *tMin, float *tMax) const {
  float nearT = ray.tNear;
  float farT = ray.tFar;

  for (int i = 0; i < 3; ++i) {
    const float dir = ray.direction[i];
    const float org = ray.origin[i];

    if (std::abs(dir) < 1e-8f) {
      // Ray is parallel to this slab. It must be inside to continue.
      if (org < pMin[i] || org > pMax[i]) {
        return false;
      }
      continue;
    }

    float invDir = 1.f / dir;
    float t0 = (pMin[i] - org) * invDir;
    float t1 = (pMax[i] - org) * invDir;
    if (t0 > t1) {
      std::swap(t0, t1);
    }

    nearT = std::max(nearT, t0);
    farT = std::min(farT, t1);
    if (nearT > farT) {
      return false;
    }
  }

  if (tMin != nullptr) {
    *tMin = nearT;
  }
  if (tMax != nullptr) {
    *tMax = farT;
  }
  return true;
}

Point3f AABB::Center() const {
  return Point3f{(pMin[0] + pMax[0]) * .5f, (pMin[1] + pMax[1]) * .5f,
                 (pMin[2] + pMax[2]) * .5f};
}