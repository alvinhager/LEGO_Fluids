#pragma once

#include <stdio.h>
#include "FluidSimulation.h"
#include "Grid3D.h"
#include "VectorMath.h"
#include "Array3D.h"

class FluidSimulation;
void computeVelocityFieldAndWeightsWithGPU(Array3D<float> &velocity_field, Array3D<bool> &isValueSet, int direction, FluidSimulation &fluidsim, Array3D<float> &weights);
