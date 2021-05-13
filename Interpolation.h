#ifndef INTERPOLATION_FUNCTIONS_AH
#define INTERPOLATION_FUNCTIONS_AH

#include "Array3D.h"
#include "grid3D.h"

#include <stdio.h>
#include <iostream>
#include <math.h>

/** namespace that holds interpolation functions */
namespace Interpolation
{
    extern double cubicInterpolate(double points[4], double x);
    extern double cubicInterpolate2(double p[4], double x);
    extern double bicubicInterpolate(double points[4][4], double x, double y);
    extern double tricubicInterpolate(double points[4][4][4], double x, double y, double z);

    extern double trilinearInterpolate(double points[8], double x, double y, double z);
    extern double trilinearInterpolate(VectorMath::vec3 p, double dx, Array3D<float> &grid);

    extern double bilinearInterpolate(double v00, double v10, double v01, double v11, double ix, double iy);
    extern void trilinearInterpolateGradient(VectorMath::vec3 p, double dx, Array3D<float> &grid, VectorMath::vec3 *grad);
}

#endif