#pragma once
#include "VectorMath.h"

struct FluidParticle
{
    VectorMath::vec3 position;
    VectorMath::vec3 velocity;

    FluidParticle() {}
    FluidParticle(VectorMath::vec3 p) : position(p) {}
    FluidParticle(VectorMath::vec3 p, VectorMath::vec3 v) : position(p), velocity(v) {}
};