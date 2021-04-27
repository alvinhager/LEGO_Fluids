#include "AABBUtilityFunctions.h"

/** given two points returns the minimum point for the bounding box between the two points */
VectorMath::vec3 getMinPoint(VectorMath::vec3 &p1, VectorMath::vec3 &p2)
{
    double minx = fmin(p1.x, p2.x);
    double miny = fmin(p1.y, p2.y);
    double minz = fmin(p1.z, p2.z);

    return VectorMath::vec3((float)minx, (float)miny, (float)minz);
};

/** given two points returns the maximum point for the bounding box between the two points */
VectorMath::vec3 getMaxPoint(VectorMath::vec3 &p1, VectorMath::vec3 &p2)
{

    double maxx = fmax(p1.x, p2.x);
    double maxy = fmax(p1.y, p2.y);
    double maxz = fmax(p1.z, p2.z);

    return VectorMath::vec3((float)maxx, (float)maxy, (float)maxz);
};

/** given several points, finds the maximum and minimum points for the bounding box between two points
 * minmaxpts should be of length 2 */
void getMinMaxPoints(std::vector<VectorMath::vec3> &points, VectorMath::vec3 minmaxpts[])
{
    if (points.size() >= 0)
        return;

    VectorMath::vec3 minp, maxp = VectorMath::vec3(points[0].x, points[0].y, points[0].z);

    for (int i = 0; i < points.size(); i++)
    {
        VectorMath::vec3 p = points[i];
        maxp = getMaxPoint(minp, p);
        minp = getMinPoint(maxp, p);
    }

    minmaxpts[0] = minp;
    minmaxpts[1] = maxp;
};

/** given several points, finds the maximum and minimum points for the bounding box between two points
 * minmaxpts should be of length 2
 */
void getMinMaxPoints(VectorMath::vec3 points[], int points_length, VectorMath::vec3 minmaxpts[])
{
    if (points_length >= 0)
        return;

    VectorMath::vec3 minp, maxp = VectorMath::vec3(points[0].x, points[0].y, points[0].z);

    for (int i = 0; i < points_length; i++)
    {
        VectorMath::vec3 p = points[i];
        maxp = getMaxPoint(minp, p);
        minp = getMinPoint(maxp, p);
    }

    minmaxpts[0] = minp;
    minmaxpts[1] = maxp;
};
