#include "vector_test.h"

// VECTOR DEFINITIONS

// default constructor
CUDA_CALLABLE_MEMBER vec33::vec33() : x(0.0), y(0.0), z(0.0)
{
}

// parameter constructor
CUDA_CALLABLE_MEMBER vec33::vec33(float x, float y, float z) : x(x), y(y), z(z)
{
}

// default destructor
CUDA_CALLABLE_MEMBER vec33::~vec33()
{
}