#pragma once

#include "Triangle.h"
#include "Array3D.h"
#include "VectorMath.h"
#include <math.h>
#include <vector>
#include "AABBUtilityFunctions.h"
#include "Constants.h"

/** Axis aligned boundary box class */
class AABB
{
public:
    VectorMath::vec3 position;
    double width = 0.0;
    double height = 0.0;
    double depth = 0.0;

    AABB();
    AABB(double x, double y, double z, double width, double height, double depth);
    AABB(VectorMath::vec3 p, double width, double height, double depth);
    AABB(VectorMath::vec3 p1, VectorMath::vec3 p2);
    AABB(std::vector<VectorMath::vec3> &points);
    AABB(Triangle t, std::vector<VectorMath::vec3> &vertices);
    AABB(GridIndex g, double dx);
    ~AABB();

    void expand(double v);
    bool isPointInside(VectorMath::vec3 p);
    bool isLineIntersecting(VectorMath::vec3 p1, VectorMath::vec3 p2);
    AABB getIntersection(AABB bbox);
    AABB getUnion(AABB bbox);
    VectorMath::vec3 getCenterPoint() const;
    bool intersects(AABB bbox);

    VectorMath::vec3 getMinPoint();
    VectorMath::vec3 getMaxPoint();
    VectorMath::vec3 getNearestPointInsideAABB(VectorMath::vec3 p, double eps);
    VectorMath::vec3 getNearestPointInsideAABB(VectorMath::vec3 p);

private:
};
