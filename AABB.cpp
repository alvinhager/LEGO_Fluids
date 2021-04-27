#include "AABB.h"
#include "AABBUtilityFunctions.h"

/** empty constructor */
AABB::AABB()
{
}

/** constructor with position of point (x,y,z ), width, height, depth */
AABB::AABB(double x, double y, double z, double w, double h, double d) : position((float)x, (float)y, (float)z), width(w), height(h), depth(d)
{
}

/** same as above but with vec3 position vector */
AABB::AABB(VectorMath::vec3 pos, double w, double h, double d) : position(pos), width(w), height(h), depth(d)
{
}

/** given two points calculates the bounding box width height depth and creates an AABB */
AABB::AABB(VectorMath::vec3 p1, VectorMath::vec3 p2)
{
    VectorMath::vec3 minp = UtilityFunctions::getMinPoint(p1, p2);
    VectorMath::vec3 maxp = UtilityFunctions::getMaxPoint(p1, p2);

    position = minp;
    width = maxp.x - minp.x;
    height = maxp.y - minp.y;
    depth = maxp.z - minp.z;
}

/** given a number of points creates a bounding box */
AABB::AABB(std::vector<VectorMath::vec3> &points)
{
    if (points.size() == 0)
    {
        return;
    }

    VectorMath::vec3 minmaxpts[2];
    UtilityFunctions::getMinMaxPoints(points, minmaxpts);
    VectorMath::vec3 maxp = minmaxpts[1];

    double eps = 1e-9;
    position = minmaxpts[0];
    width = maxp.x - position.x + eps;
    height = maxp.y - position.y + eps;
    depth = maxp.z - position.z + eps;
}

/** takes in a triangle index list and vertex list to create bounding box */
AABB::AABB(Triangle t, std::vector<VectorMath::vec3> &vertices)
{
    VectorMath::vec3 points[3] = {vertices[t.tri[0]],
                                  vertices[t.tri[1]],
                                  vertices[t.tri[2]]};

    VectorMath::vec3 minmaxpts[2];
    UtilityFunctions::getMinMaxPoints(points, 3, minmaxpts);

    VectorMath::vec3 maxp = minmaxpts[1];

    double eps = 1e-9;
    position = minmaxpts[0];
    width = maxp.x - position.x + eps;
    height = maxp.y - position.y + eps;
    depth = maxp.z - position.z + eps;
}
/** creates a bounding box at the face identified by grid index, with a dimensions dx */
AABB::AABB(GridIndex g, double dx)
{
    position = VectorMath::vec3(g.i * (float)dx, g.j * (float)dx, g.k * (float)dx);
    width = height = depth = dx;
}

/** default destructor */
AABB::~AABB()
{
}

/** expands a bounding box by a scaling factor, while remaining centered at same point */
void AABB::expand(double v)
{
    double h = 0.5 * v;
    position -= VectorMath::vec3((float)h, (float)h, (float)h);
    width += v;
    height += v;
    depth += v;
}

/** checks if a point p is inside the bounding box */
bool AABB::isPointInside(VectorMath::vec3 p)
{
    bool is_over_minpoint = p.x >= position.x && p.y >= position.y && p.z >= position.z;
    bool is_under_maxpoint = p.x < position.x + width && p.y < position.y + height && p.z < position.z + depth;

    return is_over_minpoint && is_under_maxpoint;
}

/** checks if a line is intersecting the bounding box */
bool AABB::isLineIntersecting(VectorMath::vec3 p1, VectorMath::vec3 p2)
{
    VectorMath::vec3 min = position;
    VectorMath::vec3 max = getMaxPoint();

    VectorMath::vec3 d = (p2 - p1) * 0.5f;
    VectorMath::vec3 e = (max - min) * 0.5f;
    VectorMath::vec3 c = p1 + d - (min + max) * 0.5f;
    VectorMath::vec3 ad = VectorMath::vec3(fabs(d.x), fabs(d.y), fabs(d.z));

    if (fabs(c.x) > e.x + ad.x)
    {
        return false;
    }
    if (fabs(c.y) > e.y + ad.y)
    {
        return false;
    }
    if (fabs(c.z) > e.z + ad.z)
    {
        return false;
    }

    double eps = 10e-9;
    if (fabs(d.y * c.z - d.z * c.y) > e.y * ad.z + e.z * ad.y + eps)
    {
        return false;
    }
    if (fabs(d.z * c.x - d.x * c.z) > e.z * ad.x + e.x * ad.z + eps)
    {
        return false;
    }
    if (fabs(d.x * c.y - d.y * c.x) > e.x * ad.y + e.y * ad.x + eps)
    {
        return false;
    }

    return true;
}

/** returns the intersection of two bounding boxes as a bounding box */
AABB AABB::getIntersection(AABB bbox)
{
    VectorMath::vec3 minp1 = getMinPoint();
    VectorMath::vec3 minp2 = bbox.getMinPoint();
    VectorMath::vec3 maxp1 = getMaxPoint();
    VectorMath::vec3 maxp2 = bbox.getMaxPoint();

    // if they are not intersecting at all return the original bbox
    if (minp1.x > maxp2.x || minp1.y > maxp2.y || minp1.z > maxp2.z || maxp1.x < minp2.x || maxp1.y < minp2.y || maxp1.z < minp2.z)
    {
        return AABB();
    }

    VectorMath::vec3 interminp = UtilityFunctions::getMaxPoint(minp1, minp2);
    VectorMath::vec3 intermaxp = UtilityFunctions::getMinPoint(maxp1, maxp2);

    return AABB(interminp, intermaxp);
}

/** returns the union of the two bounding boxes as a bounding box */
AABB AABB::getUnion(AABB bbox)
{
    VectorMath::vec3 minp1 = getMinPoint();
    VectorMath::vec3 minp2 = bbox.getMinPoint();
    VectorMath::vec3 maxp1 = getMaxPoint();
    VectorMath::vec3 maxp2 = bbox.getMaxPoint();

    VectorMath::vec3 unionminp = UtilityFunctions::getMinPoint(minp1, minp2);
    VectorMath::vec3 unionmaxp = UtilityFunctions::getMaxPoint(maxp1, maxp2);

    return AABB(unionminp, unionmaxp);
}

/** returns the minimum position  */
VectorMath::vec3 AABB::getMinPoint()
{
    return position;
}

/** returns the max position */
VectorMath::vec3 AABB::getMaxPoint()
{
    return position + VectorMath::vec3((float)width, (float)height, (float)depth);
}

/** return the nearest point inside the bounding box from p */
VectorMath::vec3 AABB::getNearestPointInsideAABB(VectorMath::vec3 p)
{
    return getNearestPointInsideAABB(p, 1e-6);
}

/** return the nearest point inside AABB */
VectorMath::vec3 AABB::getNearestPointInsideAABB(VectorMath::vec3 p, double eps)
{
    if (isPointInside(p))
    {
        return p;
    }

    VectorMath::vec3 max = getMaxPoint();
    VectorMath::vec3 min = getMinPoint();

    p.x = fmax(p.x, min.x);
    p.y = fmax(p.y, min.y);
    p.z = fmax(p.z, min.z);

    // avoids divide by zero in calculations later
    p.x = fmin(p.x, max.x - (float)eps);
    p.y = fmin(p.y, max.y - (float)eps);
    p.z = fmin(p.z, max.z - (float)eps);

    return p;
}