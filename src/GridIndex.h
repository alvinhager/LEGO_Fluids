#pragma once

#include <stdexcept>
#include <string>

#ifdef __CUDACC__
#define CUDA_CALLABLE_MEMBER __host__ __device__
#else
#define CUDA_CALLABLE_MEMBER
#endif

/** Simple struct used to store and compare3D grid indexes */
struct GridIndex
{
    int i, j, k;

    CUDA_CALLABLE_MEMBER GridIndex() : i(0), j(0), k(0) {}
    CUDA_CALLABLE_MEMBER GridIndex(int ival, int jval, int kval) : i(ival), j(jval), k(kval) {}

    /** returns true if the two grid indexes have the same (i,j,k) values */
    bool operator==(const GridIndex &rhs) const
    {
        return i == rhs.i && j == rhs.j && k == rhs.k;
    }

    /** returns true if at least one grid index component (i,j,k) is not the same
   * as the other */
    bool operator!=(const GridIndex &rhs) const { return !(*this == rhs); }

    /** this operator is used to access and return a reference to one of the
   * fields (i,j or k) throws if index is greater than 2
   */
    int &operator[](unsigned int idx)
    {
        if (idx > 2)
        {
            throw std::out_of_range(
                "Error, desired grid index is out of range (>2 so pointer is not pointing at any field in GridIndex struct ) \n");
        }
        return (&i)[idx];
    }
};
