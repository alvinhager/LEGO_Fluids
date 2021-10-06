
#pragma once

#include "VectorMath.h"
#include <vector>

namespace AABBUtilityFunctions
{
    VectorMath::vec3 getMinPoint(VectorMath::vec3 &p1, VectorMath::vec3 &p2);
    VectorMath::vec3 getMaxPoint(VectorMath::vec3 &p1, VectorMath::vec3 &p2);
    void getMinMaxPoints(std::vector<VectorMath::vec3> &points, VectorMath::vec3 minmaxpts[]);
    void getMinMaxPoints(VectorMath::vec3 points[], int len, VectorMath::vec3 minmaxpts[]);
}
