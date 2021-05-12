#include "interpolation.h"
#include <math.h>

/** for 3D, interpolates using cubic interpolation for each layer of the x pancake, then at the end interpolates this along z axis */
double Interpolation::tricubicInterpolate(double p[4][4][4], double x, double y, double z)
{
    double arr[4];
    arr[0] = bicubicInterpolate(p[0], x, y);
    arr[1] = bicubicInterpolate(p[1], x, y);
    arr[2] = bicubicInterpolate(p[2], x, y);
    arr[3] = bicubicInterpolate(p[3], x, y);
    return cubicInterpolate(arr, z);
}

/** for 2D, interpolates using cubic interpolate first along x-axis, then along y-axis using the results */
double Interpolation::bicubicInterpolate(double p[4][4], double x, double y)
{
    double arr[4];
    arr[0] = cubicInterpolate(p[0], x);
    arr[1] = cubicInterpolate(p[1], x);
    arr[2] = cubicInterpolate(p[2], x);
    arr[3] = cubicInterpolate(p[3], x);
    return cubicInterpolate(arr, y);
}

/** interpolates using cubic polynomial for the point p1, with the points around it used to give a better estimate */
double Interpolation::cubicInterpolate(double p[4], double x)
{
    return p[1] + 0.5 * x * (p[2] - p[0] + x * (2.0 * p[0] - 5.0 * p[1] + 4.0 * p[2] - p[3] + x * (3.0 * (p[1] - p[2]) + p[3] - p[0])));
}

/** cubic interpolation formula from Bridson's book */
double Interpolation::cubicInterpolate2(double p[4], double x)
{
    float x_2 = powf(x, 2);
    float x_3 = powf(x, 3);
    float negative_third = 1 / (float)3;
    float sixth = 1 / (float)6;

    return (negative_third + 0.5 * x_2 + sixth * x_3) * p[0] +
           (1 - x_2 + 0.5 * (x_3 - x)) * p[1] +
           (x + 0.5(x_2 - x_3)) * p[2] +
           (sixth * (x_3 - x)) * p[3];
}

/** for 3D, trilinear interpolation where the values at the 8 points of the cube edges are given along with x,y,z are fraction moved in axis directions */
double Interpolation::trilinearInterpolate(double p[8], double x, double y, double z)
{
    return p[0] * (1 - x) * (1 - y) * (1 - z) +
           p[1] * x * (1 - y) * (1 - z) +
           p[2] * (1 - x) * y * (1 - z) +
           p[3] * (1 - x) * (1 - y) * z +
           p[4] * x * (1 - y) * z +
           p[5] * (1 - x) * y * z +
           p[6] * x * y * (1 - z) +
           p[7] * x * y * z;
}

/** trilinearly interpolates the value at global position p on the grid */
double Interpolation::trilinearInterpolate(VectorMath::vec3 p, double dx, Array3D<float> &grid)
{

    GridIndex g = Grid3D::positionToGridIndex(p, dx);
    VectorMath::vec3 gpos = Grid3D::GridIndexToPosition(g, dx);

    double inv_dx = 1.0 / dx;
    double ix = (p.x - gpos.x) * inv_dx;
    double iy = (p.y - gpos.y) * inv_dx;
    double iz = (p.z - gpos.z) * inv_dx;

    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int isize = grid.width;
    int jsize = grid.height;
    int ksize = grid.depth;

    if (Grid3D::isGridIndexInRange(g.i, g.j, g.k, isize, jsize, ksize))
    {
        points[0] = grid(g.i, g.j, g.k);
    }
    if (Grid3D::isGridIndexInRange(g.i + 1, g.j, g.k, isize, jsize, ksize))
    {
        points[1] = grid(g.i + 1, g.j, g.k);
    }
    if (Grid3D::isGridIndexInRange(g.i, g.j + 1, g.k, isize, jsize, ksize))
    {
        points[2] = grid(g.i, g.j + 1, g.k);
    }
    if (Grid3D::isGridIndexInRange(g.i, g.j, g.k + 1, isize, jsize, ksize))
    {
        points[3] = grid(g.i, g.j, g.k + 1);
    }
    if (Grid3D::isGridIndexInRange(g.i + 1, g.j, g.k + 1, isize, jsize, ksize))
    {
        points[4] = grid(g.i + 1, g.j, g.k + 1);
    }
    if (Grid3D::isGridIndexInRange(g.i, g.j + 1, g.k + 1, isize, jsize, ksize))
    {
        points[5] = grid(g.i, g.j + 1, g.k + 1);
    }
    if (Grid3D::isGridIndexInRange(g.i + 1, g.j + 1, g.k, isize, jsize, ksize))
    {
        points[6] = grid(g.i + 1, g.j + 1, g.k);
    }
    if (Grid3D::isGridIndexInRange(g.i + 1, g.j + 1, g.k + 1, isize, jsize, ksize))
    {
        points[7] = grid(g.i + 1, g.j + 1, g.k + 1);
    }

    return trilinearInterpolate(points, ix, iy, iz);
}

/** bilinearly interpolates a value for a square with square edge values given */
double Interpolation::bilinearInterpolate(double val00, double val10, double val01, double val11, double sx, double sy)
{
    double xlerp1 = (1 - sx) * val00 + sx * val10;
    double xlerp2 = (1 - sx) * val01 + sx * val11;

    return (1 - sy) * xlerp1 + sy * xlerp2;
}

/** trilinearly interpolates the gradient at global position p */
void Interpolation::trilinearInterpolateGradient(VectorMath::vec3 p, double dx, Array3D<float> &grid, VectorMath::vec3 *grad)
{

    GridIndex g = Grid3D::positionToGridIndex(p, dx);
    VectorMath::vec3 gpos = Grid3D::GridIndexToPosition(g, dx);

    double inv_dx = 1.0 / dx;
    double ix = (p.x - gpos.x) * inv_dx;
    double iy = (p.y - gpos.y) * inv_dx;
    double iz = (p.z - gpos.z) * inv_dx;

    int isize = grid.width;
    int jsize = grid.height;
    int ksize = grid.depth;

    float v000 = 0, v001 = 0, v010 = 0, v011 = 0, v100 = 0, v101 = 0, v110 = 0, v111 = 0;
    if (Grid3D::isGridIndexInRange(g.i, g.j, g.k, isize, jsize, ksize))
    {
        v000 = grid(g.i, g.j, g.k);
    }
    if (Grid3D::isGridIndexInRange(g.i + 1, g.j, g.k, isize, jsize, ksize))
    {
        v100 = grid(g.i + 1, g.j, g.k);
    }
    if (Grid3D::isGridIndexInRange(g.i, g.j + 1, g.k, isize, jsize, ksize))
    {
        v010 = grid(g.i, g.j + 1, g.k);
    }
    if (Grid3D::isGridIndexInRange(g.i, g.j, g.k + 1, isize, jsize, ksize))
    {
        v001 = grid(g.i, g.j, g.k + 1);
    }
    if (Grid3D::isGridIndexInRange(g.i + 1, g.j, g.k + 1, isize, jsize, ksize))
    {
        v101 = grid(g.i + 1, g.j, g.k + 1);
    }
    if (Grid3D::isGridIndexInRange(g.i, g.j + 1, g.k + 1, isize, jsize, ksize))
    {
        v011 = grid(g.i, g.j + 1, g.k + 1);
    }
    if (Grid3D::isGridIndexInRange(g.i + 1, g.j + 1, g.k, isize, jsize, ksize))
    {
        v110 = grid(g.i + 1, g.j + 1, g.k);
    }
    if (Grid3D::isGridIndexInRange(g.i + 1, g.j + 1, g.k + 1, isize, jsize, ksize))
    {
        v111 = grid(g.i + 1, g.j + 1, g.k + 1);
    }

    float ddx00 = v100 - v000;
    float ddx10 = v110 - v010;
    float ddx01 = v101 - v001;
    float ddx11 = v111 - v011;
    float dv_dx = (float)bilinearInterpolate(ddx00, ddx10, ddx01, ddx11, iy, iz);

    float ddy00 = v010 - v000;
    float ddy10 = v110 - v100;
    float ddy01 = v011 - v001;
    float ddy11 = v111 - v101;
    float dv_dy = (float)bilinearInterpolate(ddy00, ddy10, ddy01, ddy11, ix, iz);

    float ddz00 = v001 - v000;
    float ddz10 = v101 - v100;
    float ddz01 = v011 - v010;
    float ddz11 = v111 - v110;
    float dv_dz = (float)bilinearInterpolate(ddz00, ddz10, ddz01, ddz11, ix, iy);

    grad->x = dv_dx;
    grad->y = dv_dy;
    grad->z = dv_dz;
}