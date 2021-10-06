#ifndef Water_Particle_AH
#define Water_Particle_AH

#include "VectorMath.h"

/** simple water particle struct that stores a water particle's info: position + velocity */
struct WaterParticle
{
    VectorMath::vec3 velocity;
    VectorMath::vec3 position;

    WaterParticle() {}
    WaterParticle(VectorMath::vec3 pos) : position(pos) {}
    WaterParticle(VectorMath::vec3 pos, VectorMath::vec3 vel) : position(pos), velocity(vel) {}
};

#endif