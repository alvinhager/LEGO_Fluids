#ifndef VALID_VELOCITY_GRID_AH
#define VALID_VELOCITY_GRID_AH

#include "Array3D.h"

struct ValidVelocityGrid
{
    Array3D<bool> validU;
    Array3D<bool> validV;
    Array3D<bool> validW;

    ValidVelocityGrid() {}
    ValidVelocityGrid(int i, int j, int k) : validU(i + 1, j, k, false),
                                             validV(i, j + 1, k, false),
                                             validW(i, j, k + 1, false) {}

    void reinitialize()
    {
        validU.fill(false);
        validV.fill(false);
        validW.fill(false);
    }
};

#endif