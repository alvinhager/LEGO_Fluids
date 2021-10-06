#include "AABB.h"

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
    VectorMath::vec3 minp = AABBUtilityFunctions::getMinPoint(p1, p2);
    VectorMath::vec3 maxp = AABBUtilityFunctions::getMaxPoint(p1, p2);

    position = minp;
    width = maxp.x - minp.x;
    height = maxp.y - minp.y;
    depth = maxp.z - minp.z;
}

/** returns the point at the center of the bounding box */
VectorMath::vec3 AABB::getCenterPoint() const
{
    return position + VectorMath::vec3(width / 2, height / 2, depth / 2);
}

/** given a number of points creates a bounding box */
AABB::AABB(std::vector<VectorMath::vec3> &points)
{
    if (points.size() == 0)
        return;

    VectorMath::vec3 minmaxpts[2];
    AABBUtilityFunctions::getMinMaxPoints(points, minmaxpts);
    VectorMath::vec3 maxp = minmaxpts[1];

    position = minmaxpts[0];
    width = maxp.x - position.x + Constants::EPSILON;
    height = maxp.y - position.y + Constants::EPSILON;
    depth = maxp.z - position.z + Constants::EPSILON;
}

/** takes in a triangle veretex index list and vertex position list to create bounding box */
AABB::AABB(Triangle triangle, std::vector<VectorMath::vec3> &vertices)
{
    VectorMath::vec3 points[3] = {vertices[triangle.tri_vertex_indices[0]], vertices[triangle.tri_vertex_indices[1]], vertices[triangle.tri_vertex_indices[2]]};

    VectorMath::vec3 minmaxpts[2];
    AABBUtilityFunctions::getMinMaxPoints(points, 3, minmaxpts);

    VectorMath::vec3 maxp = minmaxpts[1];
    position = minmaxpts[0];

    double eps = 1e-9;
    width = maxp.x - position.x + Constants::EPSILON;
    height = maxp.y - position.y + Constants::EPSILON;
    depth = maxp.z - position.z + Constants::EPSILON;
}
/** creates a bounding box at the position identified by grid index, with a dimensions length =  dx for w,h,d */
AABB::AABB(GridIndex g, double dx)
{
    position = VectorMath::vec3(g.i * (float)dx, g.j * (float)dx, g.k * (float)dx);
    width = height = depth = dx;
}

/** default destructor */
AABB::~AABB()
{
}

/** expands a bounding box size in all dimensions by length v, whilst remaining centered at the same point in space */
void AABB::expand(double exp_dist)
{
    float translation = (float)exp_dist * 0.5;
    position -= VectorMath::vec3(translation, translation, translation);
    width += exp_dist;
    height += exp_dist;
    depth += exp_dist;
}

/** checks if a point p is inside the bounding box */
bool AABB::isPointInside(VectorMath::vec3 p)
{
    bool is_over_minpoint = p.x >= position.x && p.y >= position.y && p.z >= position.z;
    bool is_under_maxpoint = p.x < position.x + width && p.y < position.y + height && p.z < position.z + depth;

    return is_over_minpoint && is_under_maxpoint;
}

/** checks if a line is intersecting the bounding box by using SAT theorem 
*  SAT explanation 
*  first the problem is translated so AABB is centered at origin.
* SAT says that for a line segment anda  box there are six potential separating axes:
* 1. Box axes
* 2. Cross product of the segment dir. vector and the box axes 
* 
* For each axis, we test to see if projected distance from box center 
* to the center of the line segment is greater than the sum of the objects' projected radii. 
* If this is the case, we have a separating axis and it intersects AABB. 
* 
* Tests are done for:
* 
* 1. Box axis, one for each cardinal axis = 3 cases
* Segment radius = abs value of segment dir. 
* Box radius = box radius projected on to the current axis (x,y,z cardinal axes)
* Distance = absolute value of the vector going from center of box to center of line segment 
* 
* 2. one test for each cross product of segment dir with each axis = 3 cases
* Segment radius = 0 as the cross product is always perpendicular to segment direction 
* Box radius = box radius projected onto the cross product of segment dir. vector and the box axis  = br dot cross sgmnt dir
* Distance = absolute value of the distance from center of box to center of line segment dot  box axis 
* */
bool AABB::isLineIntersecting(VectorMath::vec3 p1, VectorMath::vec3 p2)
{
    VectorMath::vec3 min = position;
    VectorMath::vec3 max = getMaxPoint();

    VectorMath::vec3 line_radius = (p2 - p1) * 0.5f;
    VectorMath::vec3 box_radius = (max - min) * 0.5f;
    VectorMath::vec3 dist_centers = p1 + line_radius - (min + max) * 0.5f;
    VectorMath::vec3 abs_line_radius = line_radius.getAbsoluteValueComponentsVector();

    if (fabs(dist_centers.x) > box_radius.x + abs_line_radius.x)
    {
        return false;
    }
    if (fabs(dist_centers.y) > box_radius.y + abs_line_radius.y)
    {
        return false;
    }
    if (fabs(dist_centers.z) > box_radius.z + abs_line_radius.z)
    {
        return false;
    }

    if (fabs(line_radius.y * dist_centers.z - line_radius.z * dist_centers.y) > box_radius.y * abs_line_radius.z + box_radius.z * abs_line_radius.y + Constants::EPSILON_SMALL)
    {
        return false;
    }
    if (fabs(line_radius.z * dist_centers.x - line_radius.x * dist_centers.z) > box_radius.z * abs_line_radius.x + box_radius.x * abs_line_radius.z + Constants::EPSILON_SMALL)
    {
        return false;
    }
    if (fabs(line_radius.x * dist_centers.y - line_radius.y * dist_centers.x) > box_radius.x * abs_line_radius.y + box_radius.y * abs_line_radius.x + Constants::EPSILON_SMALL)
    {
        return false;
    }

    return true;
}

/** returns whether two boxes are intersecting/overlapping */
bool AABB::intersects(AABB bbox)
{
    VectorMath::vec3 minp1 = getMinPoint();
    VectorMath::vec3 minp2 = bbox.getMinPoint();
    VectorMath::vec3 maxp1 = getMaxPoint();
    VectorMath::vec3 maxp2 = bbox.getMaxPoint();

    // the two boxes are not intersecting if any of the minimum point components of one box is greater than the corresponding maximum point component of the other box
    if (minp1.x > maxp2.x || minp1.y > maxp2.y || minp1.z > maxp2.z || maxp1.x < minp2.x || maxp1.y < minp2.y || maxp1.z < minp2.z)
        return false;
    else
        return true;
}

/** returns the intersection of two bounding boxes as a bounding box */
AABB AABB::getIntersection(AABB bbox)
{
    VectorMath::vec3 minp1 = getMinPoint();
    VectorMath::vec3 minp2 = bbox.getMinPoint();
    VectorMath::vec3 maxp1 = getMaxPoint();
    VectorMath::vec3 maxp2 = bbox.getMaxPoint();

    if (intersects(bbox))
        return AABB();

    VectorMath::vec3 interminp = AABBUtilityFunctions::getMaxPoint(minp1, minp2);
    VectorMath::vec3 intermaxp = AABBUtilityFunctions::getMinPoint(maxp1, maxp2);

    return AABB(interminp, intermaxp);
}

/** returns the union of the two bounding boxes as a bounding box */
AABB AABB::getUnion(AABB bbox)
{
    VectorMath::vec3 minpbox1 = getMinPoint();
    VectorMath::vec3 maxpbox1 = getMaxPoint();

    VectorMath::vec3 minpbox2 = bbox.getMinPoint();
    VectorMath::vec3 maxpbox2 = bbox.getMaxPoint();

    VectorMath::vec3 unionminp = AABBUtilityFunctions::getMinPoint(minpbox1, minpbox2);
    VectorMath::vec3 unionmaxp = AABBUtilityFunctions::getMaxPoint(maxpbox1, maxpbox2);

    return AABB(unionminp, unionmaxp);
}

/** returns the minimum position, which is equal to position assuming w,h,d are all positive  */
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
    return getNearestPointInsideAABB(p, Constants::EPSILON_MEDIUM);
}

/** return the nearest point inside AABB */
VectorMath::vec3 AABB::getNearestPointInsideAABB(VectorMath::vec3 p, double eps)
{
    if (isPointInside(p))
        return p;

    VectorMath::vec3 max = getMaxPoint();
    VectorMath::vec3 min = getMinPoint();

    p.x = fmax(p.x, min.x);
    p.y = fmax(p.y, min.y);
    p.z = fmax(p.z, min.z);

    // avoids divide by zero in calculations later by using epsilon here
    p.x = fmin(p.x, max.x - (float)eps);
    p.y = fmin(p.y, max.y - (float)eps);
    p.z = fmin(p.z, max.z - (float)eps);

    return p;
}