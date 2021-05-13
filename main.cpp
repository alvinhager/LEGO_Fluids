#include <iostream>
// #include "CustomAssert.h"
// #include "VectorMath.h"
#include "AABBUtilityFunctions.h"
#include "AABB.h"
#include "Grid3D.h"
#include "interpolation.h"
#include "MACVelocityField.cpp"

using namespace std;
int main()
{

    //CUSTOM_ASSERT(false);
    GridIndex hey = GridIndex(1, 2, 3);
    GridIndex hey2 = GridIndex(1, 2, 3);

    cout << "Hey" << (hey != hey2) << endl;
    return 0;
}