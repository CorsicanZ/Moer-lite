#include "Disk.h"
#include "ResourceLayer/Factory.h"
bool Disk::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆环的相交 填充primId,u,v.如果相交，更新光线的tFar
    //* 1.光线变换到局部空间
    Point3f originLocal = ray.origin;
    Vector3f directionLocal = ray.direction;
    vecmat::vec4f o{originLocal[0], originLocal[1], originLocal[2], 1.f},
    d{directionLocal[0], directionLocal[1], directionLocal[2], 0.f};
    o = transform.invRotate * transform.invTranslate * o;
    d = transform.invRotate * transform.invTranslate * d;
    // 归一化
    o /= o[3];
    originLocal = Point3f{o[0], o[1], o[2]};
    directionLocal = Vector3f{d[0], d[1], d[2]};
    Ray rayLocal(originLocal, directionLocal, ray.tNear, ray.tFar, ray.time);

    //* 2.判断局部光线的方向在z轴分量是否足够小
    if (std::abs(rayLocal.direction[2]) < EPSILON)
        return false;
    
    //* 3.计算光线和平面交点
    float t = -rayLocal.origin[2] / rayLocal.direction[2];
    Point3f hitPointOnPlane = rayLocal.at(t);

    //* 4.检验交点是否在圆环内
    //* 4.1.计算 t 与 tNear 和 tFar 的关系
    if (t < ray.tNear || t > ray.tFar)
        return false;
    //* 4.2 计算交点是否在圆环内
    float distanceToCenter = (hitPointOnPlane - Point3f(0,0,0)).length();
    if (distanceToCenter < innerRadius || distanceToCenter > radius)
        return false;
    //* 4.3 计算交点与圆心的夹角是否符合要求
    float angle = fm::atan2(hitPointOnPlane[1], hitPointOnPlane[0]);
    if (angle < 0) {
        angle += 2 * PI;
    }
    if (angle > phiMax) {
        return false;
    }

    //* 5.更新ray的tFar,减少光线和其他物体的相交计算次数
    ray.tFar = t;
    *primID = 0;
    *u = angle / (phiMax);
    *v = (distanceToCenter - innerRadius) / (radius - innerRadius);

    return true;
}

void Disk::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆环相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    //* 2.位置信息可以根据uv计算出，同样需要变换
    //* Write your code here.
    /// ----------------------------------------------------
    float angle = u * phiMax;
    float r = innerRadius + v * (radius - innerRadius);

    intersection->normal = normalize(transform.toWorld(Vector3f{0, 0, 1}));
    intersection->position = transform.toWorld(Point3f{r * fm::cos(angle), r * fm::sin(angle), 0});

    intersection->shape = this;
    intersection->distance = distance;
    intersection->texCoord = Vector2f{u, v};
    Vector3f tangent{1.f, 0.f, .0f};
    Vector3f bitangent;
    if (std::abs(dot(tangent, intersection->normal)) > .9f) {
        tangent = Vector3f(.0f, 1.f, .0f);
    }
    bitangent = normalize(cross(tangent, intersection->normal));
    tangent = normalize(cross(intersection->normal, bitangent));
    intersection->tangent = tangent;
    intersection->bitangent = bitangent;
}

Disk::Disk(const Json &json) : Shape(json) {
//    normal = transform.toWorld(Vector3f(0,0,1));
//    origin = transform.toWorld(Point3f(0,0,0));
//    auto
//    //radius认为是三个方向的上的scale平均
//    vecmat::vec4f v(1,1,1,0);
//    auto radiusVec = transform.scale * v;
//    radiusVec/=radiusVec[3];
//    radius = (radiusVec[0]+radiusVec[1]+radiusVec[2])/3;
     radius = fetchOptional(json,"radius",1.f);
     innerRadius = fetchOptional(json,"inner_radius",0.f);
     phiMax = fetchOptional(json,"phi_max",2 * PI);
     AABB local(Point3f(-radius,-radius,0),Point3f(radius,radius,0));
     boundingBox = transform.toWorld(local);
}

void Disk::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {
        //采样光源 暂时不用实现
}
REGISTER_CLASS(Disk, "disk")

