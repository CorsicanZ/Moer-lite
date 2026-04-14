#include "ThinLens.h"

ThinLensCamera::ThinLensCamera(const Json& json) : PerspectiveCamera(json) {
    lensRadius    = json["lensRadius"].get<float>();
    focalDistance = json["focalDistance"].get<float>();
}

Ray ThinLensCamera::sampleRay(const CameraSample& sample, Vector2f NDC) const {
    float x = (NDC[0] - 0.5f) * film->size[0] + sample.xy[0],
    y = (0.5f - NDC[1]) * film->size[1] + sample.xy[1];

    float tanHalfFov = fm::tan(verticalFov * 0.5f);
    float z = -film->size[1] * 0.5f / tanHalfFov;

    float k = focalDistance / -z;
    Point3f pFocus = Point3f{x * k, y * k, z * k};

    float sampleLensRadius = lensRadius * fm::sqrt(sample.lens[0]);
    float sampleLensTheta = 2 * PI * sample.lens[1];
    Point3f pLens = Point3f{sampleLensRadius * fm::cos(sampleLensTheta), sampleLensRadius * fm::sin(sampleLensTheta), 0};

    Vector3f direction = normalize(transform.toWorld(pFocus - pLens));
    Point3f origin = transform.toWorld(pLens);
    return Ray(origin, direction, tNear, tFar, timeStart);
}

Ray ThinLensCamera::sampleRayDifferentials(const CameraSample& sample, Vector2f NDC) const {
    float x = (NDC[0] - 0.5f) * film->size[0] + sample.xy[0],
          y = (0.5f - NDC[1]) * film->size[1] + sample.xy[1];

    float tanHalfFov = fm::tan(verticalFov * 0.5f);
    float z = -film->size[1] * 0.5f / tanHalfFov;

    float k = focalDistance / -z;
    Point3f pFocus = Point3f{x * k, y * k, z * k};
    Point3f pFocusX = Point3f{(x + 1.f) * k, y * k, z * k};
    Point3f pFocusY = Point3f{x * k, (y + 1.f) * k, z * k};

    float sampleLensRadius = lensRadius * fm::sqrt(sample.lens[0]);
    float sampleLensTheta = 2 * PI * sample.lens[1];
    Point3f pLens = Point3f{sampleLensRadius * fm::cos(sampleLensTheta), sampleLensRadius * fm::sin(sampleLensTheta), 0};

    Point3f origin = transform.toWorld(pLens);
    Vector3f direction = normalize(transform.toWorld(pFocus - pLens));
    Vector3f directionX = normalize(transform.toWorld(pFocusX - pLens));
    Vector3f directionY = normalize(transform.toWorld(pFocusY - pLens));

    Ray ret = Ray(origin, direction, tNear, tFar, timeStart);
    ret.hasDifferentials = true;
    ret.originX = origin;
    ret.originY = origin;
    ret.directionX = directionX;
    ret.directionY = directionY;
    return ret;
}

REGISTER_CLASS(ThinLensCamera, "thinlens")