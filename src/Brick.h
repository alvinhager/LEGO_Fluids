#pragma once

/** struct to store information about brick, used in water brick grid */
struct Brick
{
    float intensity = 0.0f;
    bool isActive = false;

    Brick() : intensity(0.0f), isActive(false) {}
    Brick(float i) : intensity(i), isActive(false) {}
};
