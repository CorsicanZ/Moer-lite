#include "Cylinder.h"
#include "ResourceLayer/Factory.h"
bool Cylinder::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
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

    //* 2.联立方程求解 
    float A = rayLocal.direction[0] * rayLocal.direction[0] + rayLocal.direction[1] * rayLocal.direction[1];
    float B = 2 * (rayLocal.origin[0] * rayLocal.direction[0] + rayLocal.origin[1] * rayLocal.direction[1]);
    float C = rayLocal.origin[0] * rayLocal.origin[0] + rayLocal.origin[1] * rayLocal.origin[1] - radius * radius;
    if (A < EPSILON) {
        return false;
    }
    float t0, t1;
    if (!Quadratic(A, B, C, &t0, &t1))
        return false;

    //* 3.检验交点是否在圆柱范围内
    auto checkTvalid = [](float t, Ray &ray, float radius, float height, float phiMax) {
        if (t < ray.tNear || t > ray.tFar)
            return false;
        Point3f hitPoint = ray.at(t);
        if (hitPoint[2] < 0 || hitPoint[2] > height)
            return false;
        float radial2 = hitPoint[0] * hitPoint[0] + hitPoint[1] * hitPoint[1];
        if (radial2 > radius * radius + EPSILON)
            return false;
        float angle = fm::atan2(hitPoint[1], hitPoint[0]);
        if (angle < 0) {
            angle += 2 * PI;
        }
        if (angle > phiMax) {
            return false;
        }
        return true;
    };
    float validt = -1;
    if (checkTvalid(t0, rayLocal, radius, height, phiMax)) {
        validt = t0;
    }
    else if (checkTvalid(t1, rayLocal, radius, height, phiMax)) {
        validt = t1;
    }
    else {
        return false;
    }

    //* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
    ray.tFar = validt;

    //* 5.计算uv
    *primID = 0;
    float angle = fm::atan2(rayLocal.at(validt)[1], rayLocal.at(validt)[0]);
    if (angle < 0) {
        angle += 2 * PI;
    }
    *u = angle / phiMax;
    *v = (rayLocal.at(validt)[2] - 0) / height;
    return true;
}

void Cylinder::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆柱相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    //* 2.位置信息可以根据uv计算出，同样需要变换
    //* Write your code here.
    /// ----------------------------------------------------
    float angle = u * phiMax;
    intersection->normal = normalize(transform.toWorld(Vector3f{fm::cos(angle), fm::sin(angle), 0}));
    intersection->position = transform.toWorld(Point3f{radius * fm::cos(angle), radius * fm::sin(angle), v * height});

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

void Cylinder::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {

}

Cylinder::Cylinder(const Json &json) : Shape(json) {
    radius = fetchOptional(json,"radius",1.f);
    height = fetchOptional(json,"height",1.f);
    phiMax = fetchOptional(json,"phi_max",2 * PI);
    AABB localAABB = AABB(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
    boundingBox = transform.toWorld(localAABB);
}

REGISTER_CLASS(Cylinder,"cylinder")
