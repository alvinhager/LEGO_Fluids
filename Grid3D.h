#ifndef GRID3D_AH
#define GRID3D_AH

#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <cmath>

#include "VectorMath.h"
#include "Array3D.h"
#include "AABB.h"

namespace Grid3d
{
    // converts a position in space to which grid index cell it is in */
    inline void positionToGridIndex(double x, double y, double z, double dx,
                                    int *i, int *j, int *k)
    {
        double inv_dx = 1.0 / dx;
        *i = (int)floor(x * inv_dx);
        *j = (int)floor(y * inv_dx);
        *k = (int)floor(z * inv_dx);
    }

    // converts a position in space to which grid index cell it is in
    inline void positionToGridIndex(VectorMath::vec3 p, double dx,
                                    int *i, int *j, int *k)
    {
        double inv_dx = 1.0 / dx;
        *i = (int)floor(p.x * inv_dx);
        *j = (int)floor(p.y * inv_dx);
        *k = (int)floor(p.z * inv_dx);
    }

    // converts a position in space to which grid index cell it is in
    inline GridIndex positionToGridIndex(double x, double y, double z, double dx)
    {
        double inv_dx = 1.0 / dx;
        return GridIndex((int)floor(x * inv_dx),
                         (int)floor(y * inv_dx),
                         (int)floor(z * inv_dx));
    }

    // converts a position in space to which grid index cell it is in
    inline GridIndex positionToGridIndex(VectorMath::vec3 p, double dx)
    {
        double inv_dx = 1.0 / dx;
        return GridIndex((int)floor(p.x * inv_dx),
                         (int)floor(p.y * inv_dx),
                         (int)floor(p.z * inv_dx));
    }

    // converts a grid index to a position
    inline void GridIndexToPosition(int i, int j, int k, double dx,
                                    double *x, double *y, double *z)
    {
        *x = (double)i * dx;
        *y = (double)j * dx;
        *z = (double)k * dx;
    }

    // converts a grid index to a position
    inline void GridIndexToPosition(GridIndex g, double dx,
                                    double *x, double *y, double *z)
    {
        *x = (double)g.i * dx;
        *y = (double)g.j * dx;
        *z = (double)g.k * dx;
    }

    // converts a grid index to a position
    inline VectorMath::vec3 GridIndexToPosition(int i, int j, int k, double dx)
    {
        return VectorMath::vec3((float)(i * dx), (float)(j * dx), (float)(k * dx));
    }

    // converts a grid index to a position
    inline VectorMath::vec3 GridIndexToPosition(GridIndex g, double dx)
    {
        return VectorMath::vec3((float)(g.i * dx), (float)(g.j * dx), (float)(g.k * dx));
    }

    // given a grid index, returns the position in space of the center of that cell
    inline void GridIndexToCellCenter(int i, int j, int k, double dx,
                                      double *x, double *y, double *z)
    {
        double half_dx = 0.5 * dx;
        *x = (double)i * dx + half_dx;
        *y = (double)j * dx + half_dx;
        *z = (double)k * dx + half_dx;
    }

    // given a grid index, returns the position in space of the center of that cell
    inline void GridIndexToCellCenter(GridIndex g, double dx,
                                      double *x, double *y, double *z)
    {
        double half_dx = 0.5 * dx;
        *x = (double)g.i * dx + half_dx;
        *y = (double)g.j * dx + half_dx;
        *z = (double)g.k * dx + half_dx;
    }

    // given a grid index, returns the position in space of the center of that cell
    inline VectorMath::vec3 GridIndexToCellCenter(int i, int j, int k, double dx)
    {
        double half_dx = 0.5 * dx;
        return VectorMath::vec3((float)(i * dx + half_dx), (float)(j * dx + half_dx), (float)(k * dx + half_dx));
    }

    // given a grid index, returns the position in space of the center of that cell
    inline VectorMath::vec3 GridIndexToCellCenter(GridIndex g, double dx)
    {
        double half_dx = 0.5 * dx;
        return VectorMath::vec3((float)(g.i * dx + half_dx), (float)(g.j * dx + half_dx), (float)(g.k * dx + half_dx));
    }

    // given the index of a face, returns the position of this face
    inline VectorMath::vec3 FaceIndexToPositionU(int i, int j, int k, double dx)
    {
        return VectorMath::vec3((float)(i * dx), (float)(j + 0.5 * dx), (float)(k + 0.5 * dx));
    }

    // given the index of a face, returns the position of this face for U
    inline VectorMath::vec3 FaceIndexToPositionU(GridIndex g, double dx)
    {
        return VectorMath::vec3((float)(g.i * dx), (float)(g.j + 0.5 * dx), (float)(g.k + 0.5 * dx));
    }

    // given the index of a face, returns the position of this face for V
    inline VectorMath::vec3 FaceIndexToPositionV(int i, int j, int k, double dx)
    {
        return VectorMath::vec3((float)(i + 0.5 * dx), (float)(j * dx), (float)(k + 0.5 * dx));
    }

    // given the index of a face, returns the position of this face for V
    inline VectorMath::vec3 FaceIndexToPositionV(GridIndex g, double dx)
    {
        return VectorMath::vec3((float)(g.i + 0.5 * dx), (float)(g.j * dx), (float)(g.k + 0.5 * dx));
    }

    // given the index of a face, returns the position of this face for W
    inline VectorMath::vec3 FaceIndexToPositionW(int i, int j, int k, double dx)
    {
        return VectorMath::vec3((float)(i + 0.5 * dx), (float)(j + 0.5 * dx), (float)(k * dx));
    }

    // given the index of a face, returns the position of this face for W
    inline VectorMath::vec3 FaceIndexToPositionW(GridIndex g, double dx)
    {
        return VectorMath::vec3((float)(g.i + 0.5 * dx), (float)(g.j + 0.5 * dx), (float)(g.k * dx));
    }

    // returns whether this position is in the grid from (0,0,0) to (i,j,k)
    inline bool isPositionInGrid(double x, double y, double z, double dx, int i, int j, int k)
    {
        bool aboveMin = x >= 0 && y >= 0 && z >= 0;
        bool belowMax = x < dx * i && y < dx * j && z < dx * k;

        return aboveMin && belowMax;
    }

    // returns whether this position is in the grid from (0,0,0) to (i,j,k)
    inline bool isPositionInGrid(VectorMath::vec3 p, double dx, int i, int j, int k)
    {
        bool aboveMin = p.x >= 0 && p.y >= 0 && p.z >= 0;
        bool belowMax = p.x < dx * i && p.y < dx * j && p.z < dx * k;

        return aboveMin && belowMax;
    }

    // returns whether this position is in the grid from (0,0,0) to (i,j,k)
    inline bool isPositionInGrid(double x, double y, double z, double dx, GridIndex g)
    {
        bool aboveMin = x >= 0 && y >= 0 && z >= 0;
        bool belowMax = x < dx * g.i && y < dx * g.j && z < dx * g.k;

        return aboveMin && belowMax;
    }

    // returns whether this position is in the grid from (0,0,0) to (i,j,k)
    inline bool isPositionInGrid(VectorMath::vec3 p, double dx, GridIndex g)
    {
        bool aboveMin = p.x >= 0 && p.y >= 0 && p.z >= 0;
        bool belowMax = p.x < dx * g.i && p.y < dx * g.j && p.z < dx * g.k;

        return aboveMin && belowMax;
    }

    // checks whether the grid index is between this range, [(i,j,k), imax,jmax,kmax]
    inline bool isGridIndexInRange(int i, int j, int k, int imax, int jmax, int kmax)
    {
        return i >= 0 && j >= 0 && k >= 0 && i < imax && j < jmax && k < kmax;
    }

    // checks whether the grid index is between this range, [(i,j,k), imax,jmax,kmax]
    inline bool isGridIndexInRange(GridIndex g, int imax, int jmax, int kmax)
    {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && g.i < imax && g.j < jmax && g.k < kmax;
    }

    // checks whether the grid index is between this range, [(i,j,k), imax,jmax,kmax]
    inline bool isGridIndexInRange(int i, int j, int k, GridIndex gmax)
    {
        return i >= 0 && j >= 0 && k >= 0 && i < gmax.i && j < gmax.j && k < gmax.k;
    }

    // checks whether the grid index is between this range, [(i,j,k), imax,jmax,kmax]
    inline bool isGridIndexInRange(GridIndex g, GridIndex gmax)
    {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && g.i < gmax.i && g.j < gmax.j && g.k < gmax.k;
    }

    // returns whether two grid indices are neighbors
    inline bool areGridIndicesNeighbors(int i1, int j1, int k1, int i2, int j2, int k2)
    {
        return std::abs(i1 - i2) <= 1 && std::abs(j1 - j2) <= 1 && std::abs(k1 - k2) <= 1;
    }

    // returns whether two grid indices are neighbors
    inline bool areGridIndicesNeighbors(GridIndex g1, int i2, int j2, int k2)
    {
        return std::abs(g1.i - i2) <= 1 && std::abs(g1.j - j2) <= 1 && std::abs(g1.k - k2) <= 1;
    }

    // returns whether two grid indices are neighbors
    inline bool areGridIndicesNeighbors(int i1, int j1, int k1, GridIndex g2)
    {
        return std::abs(i1 - g2.i) <= 1 && std::abs(j1 - g2.j) <= 1 && std::abs(k1 - g2.k) <= 1;
    }

    // returns whether two grid indices are neighbors
    inline bool areGridIndicesNeighbors(GridIndex g1, GridIndex g2)
    {
        return std::abs(g1.i - g2.i) <= 1 && std::abs(g1.j - g2.j) <= 1 && std::abs(g1.k - g2.k) <= 1;
    }

    // checks whether the given grid index is on the border
    inline bool isGridIndexOnBorder(int i, int j, int k, int imax, int jmax, int kmax)
    {
        return i == 0 || j == 0 || k == 0 || i == imax - 1 || j == jmax - 1 || k == kmax - 1;
    }

    // checks whether the given grid index is on the border
    inline bool isGridIndexOnBorder(GridIndex g, int imax, int jmax, int kmax)
    {
        return g.i == 0 || g.j == 0 || g.k == 0 || g.i == imax - 1 || g.j == jmax - 1 || g.k == kmax - 1;
    }

    // checks whether the given grid index is on the border
    inline bool isGridIndexOnBorder(int i, int j, int k, GridIndex gmax)
    {
        return i == 0 || j == 0 || k == 0 || i == gmax.i - 1 || j == gmax.j - 1 || k == gmax.k - 1;
    }

    // checks whether the given grid index is on the border
    inline bool isGridIndexOnBorder(GridIndex g, GridIndex gmax)
    {
        return g.i == 0 || g.j == 0 || g.k == 0 || g.i == gmax.i - 1 || g.j == gmax.j - 1 || g.k == gmax.k - 1;
    }

    // returns the cross shaped neighbor grid indices
    inline void getNeighborGridIndices6(int i, int j, int k, GridIndex n[6])
    {
        n[0] = GridIndex(i - 1, j, k);
        n[1] = GridIndex(i + 1, j, k);
        n[2] = GridIndex(i, j - 1, k);
        n[3] = GridIndex(i, j + 1, k);
        n[4] = GridIndex(i, j, k - 1);
        n[5] = GridIndex(i, j, k + 1);
    }

    // returns the cross shaped neighbor grid indices
    inline void getNeighborGridIndices6(GridIndex g, GridIndex n[6])
    {
        n[0] = GridIndex(g.i - 1, g.j, g.k);
        n[1] = GridIndex(g.i + 1, g.j, g.k);
        n[2] = GridIndex(g.i, g.j - 1, g.k);
        n[3] = GridIndex(g.i, g.j + 1, g.k);
        n[4] = GridIndex(g.i, g.j, g.k - 1);
        n[5] = GridIndex(g.i, g.j, g.k + 1);
    }

    // returns the immediate neighbors including diagonals of the grid index
    inline void getNeighborGridIndices26(int i, int j, int k, GridIndex n[26])
    {
        int array_idx = 0;
        for (int nk = k - 1; nk <= k + 1; nk++)
        {
            for (int nj = j - 1; nj <= j + 1; nj++)
            {
                for (int ni = i - 1; ni <= i + 1; ni++)
                {
                    if (!(ni == i && nj == j && nk == k))
                    {
                        n[array_idx] = GridIndex(ni, nj, nk);
                        array_idx++;
                    }
                }
            }
        }
    }

    // returns the immediate neighbors including diagonals of the grid index
    inline void getNeighborGridIndices26(GridIndex g, GridIndex n[26])
    {
        int array_idx = 0;
        for (int nk = g.k - 1; nk <= g.k + 1; nk++)
        {
            for (int nj = g.j - 1; nj <= g.j + 1; nj++)
            {
                for (int ni = g.i - 1; ni <= g.i + 1; ni++)
                {
                    if (!(ni == g.i && nj == g.j && nk == g.k))
                    {
                        n[array_idx] = GridIndex(ni, nj, nk);
                        array_idx++;
                    }
                }
            }
        }
    }

    /** returns all the neighbors within 2 grid cell radius = 124 neighbors */
    inline void getNeighborGridIndices124(int i, int j, int k, GridIndex n[124])
    {
        int array_idx = 0;
        for (int nk = k - 2; nk <= k + 2; nk++)
        {
            for (int nj = j - 2; nj <= j + 2; nj++)
            {
                for (int ni = i - 2; ni <= i + 2; ni++)
                {
                    if (!(ni == i && nj == j && nk == k))
                    {
                        n[array_idx] = GridIndex(ni, nj, nk);
                        array_idx++;
                    }
                }
            }
        }
    }

    /** returns all the neighbors within 2 grid cell radius = 124 neighbors */
    inline void getNeighborGridIndices124(GridIndex g, GridIndex n[124])
    {
        int array_idx = 0;
        for (int nk = g.k - 2; nk <= g.k + 2; nk++)
        {
            for (int nj = g.j - 2; nj <= g.j + 2; nj++)
            {
                for (int ni = g.i - 2; ni <= g.i + 2; ni++)
                {
                    if (!(ni == g.i && nj == g.j && nk == g.k))
                    {
                        n[array_idx] = GridIndex(ni, nj, nk);
                        array_idx++;
                    }
                }
            }
        }
    }

    // returns the grid indices inside the cube area defined by moving a given number of steps in every dim */
    inline void getSubdividedGridIndices(int i, int j, int k, int no_steps, GridIndex *n)
    {
        GridIndex start = GridIndex(i * no_steps, j * no_steps, k * no_steps);
        int array_idx = 0;
        for (int k_idx = 0; k_idx < no_steps; k_idx++)
        {
            for (int j_idx = 0; j_idx < no_steps; j_idx++)
            {
                for (int i_idx = 0; i_idx < no_steps; i_idx++)
                {
                    n[array_idx] = GridIndex(start.i + i_idx, start.j + j_idx, start.k + k_idx);
                    array_idx++;
                }
            }
        }
    }

    // returns the grid indices inside the cube area defined by moving a given number of steps in every dim */
    inline void getSubdividedGridIndices(GridIndex g, int no_steps, GridIndex *n)
    {
        GridIndex start = GridIndex(g.i * no_steps, g.j * no_steps, g.k * no_steps);
        int array_idx = 0;
        for (int k_idx = 0; k_idx < no_steps; k_idx++)
        {
            for (int j_idx = 0; j_idx < no_steps; j_idx++)
            {
                for (int i_idx = 0; i_idx < no_steps; i_idx++)
                {
                    n[array_idx] = GridIndex(start.i + i_idx, start.j + j_idx, start.k + k_idx);
                    array_idx++;
                }
            }
        }
    }

    // returns the grid indeex vertices index cell ijk
    inline void getGridIndexVertices(int i, int j, int k, GridIndex v[8])
    {
        v[0] = GridIndex(i, j, k);
        v[1] = GridIndex(i + 1, j, k);
        v[2] = GridIndex(i + 1, j, k + 1);
        v[3] = GridIndex(i, j, k + 1);
        v[4] = GridIndex(i, j + 1, k);
        v[5] = GridIndex(i + 1, j + 1, k);
        v[6] = GridIndex(i + 1, j + 1, k + 1);
        v[7] = GridIndex(i, j + 1, k + 1);
    }

    // returns the grid indeex vertices index cell ijk
    inline void getGridIndexVertices(GridIndex g, GridIndex v[8])
    {
        v[0] = GridIndex(g.i, g.j, g.k);
        v[1] = GridIndex(g.i + 1, g.j, g.k);
        v[2] = GridIndex(g.i + 1, g.j, g.k + 1);
        v[3] = GridIndex(g.i, g.j, g.k + 1);
        v[4] = GridIndex(g.i, g.j + 1, g.k);
        v[5] = GridIndex(g.i + 1, g.j + 1, g.k);
        v[6] = GridIndex(g.i + 1, g.j + 1, g.k + 1);
        v[7] = GridIndex(g.i, g.j + 1, g.k + 1);
    }

    // returns the grid indeex vertices index cell ijk
    inline void getVertexGridIndexNeighbours(int i, int j, int k, GridIndex n[8])
    {
        n[0] = GridIndex(i, j, k);
        n[1] = GridIndex(i - 1, j, k);
        n[2] = GridIndex(i, j, k - 1);
        n[3] = GridIndex(i - 1, j, k - 1);
        n[4] = GridIndex(i, j - 1, k);
        n[5] = GridIndex(i - 1, j - 1, k);
        n[6] = GridIndex(i, j - 1, k - 1);
        n[7] = GridIndex(i - 1, j - 1, k - 1);
    }

    // returns the grid indeex vertices index cell ijk
    inline void getVertexGridIndexNeighbours(GridIndex v, GridIndex n[8])
    {
        n[0] = GridIndex(v.i, v.j, v.k);
        n[1] = GridIndex(v.i - 1, v.j, v.k);
        n[2] = GridIndex(v.i, v.j, v.k - 1);
        n[3] = GridIndex(v.i - 1, v.j, v.k - 1);
        n[4] = GridIndex(v.i, v.j - 1, v.k);
        n[5] = GridIndex(v.i - 1, v.j - 1, v.k);
        n[6] = GridIndex(v.i, v.j - 1, v.k - 1);
        n[7] = GridIndex(v.i - 1, v.j - 1, v.k - 1);
    }

    // returns the index bounds for a given point with radius radius, cell size dx, within the grid defined by max bounds (imax, jmax, kmax)
    inline void getGridIndexBounds(VectorMath::vec3 point, double radius, double dx,
                                   int imax, int jmax, int kmax,
                                   GridIndex *g1, GridIndex *g2)
    {
        GridIndex c = positionToGridIndex(point, dx);
        VectorMath::vec3 cpos = GridIndexToPosition(c, dx);
        VectorMath::vec3 transform = point - cpos;
        double inv = 1.0 / dx;

        int gimin = c.i - (int)fmax(0, ceil((radius - transform.x) * inv));
        int gjmin = c.j - (int)fmax(0, ceil((radius - transform.y) * inv));
        int gkmin = c.k - (int)fmax(0, ceil((radius - transform.z) * inv));
        int gimax = c.i + (int)fmax(0, ceil((radius - dx + transform.x) * inv));
        int gjmax = c.j + (int)fmax(0, ceil((radius - dx + transform.y) * inv));
        int gkmax = c.k + (int)fmax(0, ceil((radius - dx + transform.z) * inv));

        *g1 = GridIndex((int)fmax(gimin, 0),
                        (int)fmax(gjmin, 0),
                        (int)fmax(gkmin, 0));
        *g2 = GridIndex((int)fmin(gimax, imax - 1),
                        (int)fmin(gjmax, jmax - 1),
                        (int)fmin(gkmax, kmax - 1));
    }

    // returns the grid Index bounds of a point with a given raidus, and dx cell size and grid defined by max bounds (imax,jmax,kmax)
    inline void getGridIndexBounds(VectorMath::vec3 point, double radius, double dx, GridIndex gmax,
                                   GridIndex *g1, GridIndex *g2)
    {
        GridIndex c = positionToGridIndex(point, dx);
        VectorMath::vec3 cpos = GridIndexToPosition(c, dx);
        VectorMath::vec3 transform = point - cpos;
        double inv = 1.0 / dx;

        int imin = c.i - (int)fmax(0, ceil((radius - transform.x) * inv));
        int jmin = c.j - (int)fmax(0, ceil((radius - transform.y) * inv));
        int kmin = c.k - (int)fmax(0, ceil((radius - transform.z) * inv));
        int imax = c.i + (int)fmax(0, ceil((radius - dx + transform.x) * inv));
        int jmax = c.j + (int)fmax(0, ceil((radius - dx + transform.y) * inv));
        int kmax = c.k + (int)fmax(0, ceil((radius - dx + transform.z) * inv));

        *g1 = GridIndex((int)fmax(imin, 0),
                        (int)fmax(jmin, 0),
                        (int)fmax(kmin, 0));
        *g2 = GridIndex((int)fmin(imax, gmax.i - 1),
                        (int)fmin(jmax, gmax.j - 1),
                        (int)fmin(kmax, gmax.k - 1));
    }

    // returns the grid Index bounds of a point with a given raidus, and dx cell size and grid defined by max bounds (imax,jmax,kmax)
    inline void getGridIndexBounds(VectorMath::vec3 point, double radius, VectorMath::mat3 G, double dx,
                                   int imax, int jmax, int kmax,
                                   GridIndex *g1, GridIndex *g2)
    {

        float lenx = (float)radius * VectorMath::length(G[0]);
        float leny = (float)radius * VectorMath::length(G[1]);
        float lenz = (float)radius * VectorMath::length(G[2]);

        float minx = point.x - lenx;
        float maxx = point.x + lenx;
        float miny = point.y - leny;
        float maxy = point.y + leny;
        float minz = point.z - lenz;
        float maxz = point.z + lenz;

        *g1 = positionToGridIndex(VectorMath::vec3(minx, miny, minz), dx);
        *g2 = positionToGridIndex(VectorMath::vec3(maxx, maxy, maxz), dx);

        *g1 = GridIndex((int)fmax((*g1).i, 0),
                        (int)fmax((*g1).j, 0),
                        (int)fmax((*g1).k, 0));
        *g2 = GridIndex((int)fmin((*g2).i, imax - 1),
                        (int)fmin((*g2).j, jmax - 1),
                        (int)fmin((*g2).k, kmax - 1));
    }

    // returns the grid Index bounds of a point with a given raidus, and dx cell size and grid defined by max bounds (imax,jmax,kmax)
    inline void getGridIndexBounds(VectorMath::vec3 p, double r, VectorMath::mat3 G, double dx, GridIndex gmax,
                                   GridIndex *g1, GridIndex *g2)
    {
        getGridIndexBounds(p, r, G, dx, gmax.i, gmax.j, gmax.k, g1, g2);
    }

    // returns the grid index bounds of an AABB */
    inline void getGridIndexBounds(AABB bbox, double dx,
                                   int imax, int jmax, int kmax,
                                   GridIndex *g1, GridIndex *g2)
    {
        VectorMath::vec3 transform = VectorMath::vec3((float)bbox.width, (float)bbox.height, (float)bbox.depth);
        *g1 = positionToGridIndex(bbox.position, dx);
        *g2 = positionToGridIndex(bbox.position + transform, dx);

        *g1 = GridIndex((int)fmax((*g1).i, 0),
                        (int)fmax((*g1).j, 0),
                        (int)fmax((*g1).k, 0));
        *g2 = GridIndex((int)fmin((*g2).i, imax - 1),
                        (int)fmin((*g2).j, jmax - 1),
                        (int)fmin((*g2).k, kmax - 1));
    }

    // returns the grid index bounds of an AABB */
    inline void getGridIndexBounds(AABB bbox, double dx, GridIndex gmax,
                                   GridIndex *g1, GridIndex *g2)
    {
        VectorMath::vec3 transform = VectorMath::vec3((float)bbox.width, (float)bbox.height, (float)bbox.depth);
        *g1 = positionToGridIndex(bbox.position, dx);
        *g2 = positionToGridIndex(bbox.position + transform, dx);

        *g1 = GridIndex((int)fmax((*g1).i, 0),
                        (int)fmax((*g1).j, 0),
                        (int)fmax((*g1).k, 0));
        *g2 = GridIndex((int)fmin((*g2).i, gmax.i - 1),
                        (int)fmin((*g2).j, gmax.j - 1),
                        (int)fmin((*g2).k, gmax.k - 1));
    }

    // fits an AABB into a grid, returning this new AABB
    inline AABB fitAABBintoGrid(AABB bbox, double dx, int imax, int jmax, int kmax)
    {
        VectorMath::vec3 pmin = bbox.getMinPoint();
        VectorMath::vec3 pmax = bbox.getMaxPoint();
        GridIndex gmin = positionToGridIndex(pmin, dx);
        GridIndex gmax = positionToGridIndex(pmax, dx);

        if (!isGridIndexInRange(gmin, imax, jmax, kmax))
        {
            pmin = VectorMath::vec3();
        }

        VectorMath::vec3 eps = VectorMath::vec3(10e-9f, 10e-9f, 10e-9f);
        if (!isGridIndexInRange(gmax, imax, jmax, kmax))
        {
            pmax = GridIndexToPosition(gmax, dx) + VectorMath::vec3((float)dx, (float)dx, (float)dx) - eps;
        }

        return AABB(pmin, pmax);
    }

    // fits an AABB into a grid, returning this new AABB
    inline AABB fitAABBintoGrid(AABB bbox, double dx, GridIndex gmax)
    {
        return fitAABBintoGrid(bbox, dx, gmax.i, gmax.j, gmax.k);
    }

    // returns the flat index of a given grid position
    inline unsigned int getFlatIndex(GridIndex g, int isize, int jsize)
    {
        return (unsigned int)g.i + (unsigned int)isize *
                                       ((unsigned int)g.j + (unsigned int)jsize * (unsigned int)g.k);
    }

    // returns the flat index of a given grid position
    inline unsigned int getFlatIndex(int i, int j, int k, int isize, int jsize)
    {
        return (unsigned int)i + (unsigned int)isize *
                                     ((unsigned int)j + (unsigned int)jsize * (unsigned int)k);
    }

    // returns the flat index of a given grid position
    inline GridIndex getUnflattenedIndex(unsigned int flatarray_idx, int isize, int jsize)
    {
        int i = flatarray_idx % isize;
        int j = (flatarray_idx / isize) % jsize;
        int k = flatarray_idx / (jsize * isize);
        return GridIndex(i, j, k);
    }

    // checks if the given U face with index (i,j,k) is bodering a value m
    template <class T>
    inline bool isUFaceBorderingValue(int i, int j, int k, T m, Array3D<T> &grid)
    {
        if (i == grid.width)
        {
            return grid(i - 1, j, k) == m;
        }
        else if (i > 0)
        {
            return grid(i, j, k) == m || grid(i - 1, j, k) == m;
        }
        else
        {
            return grid(i, j, k) == m;
        }
    }

    // checks if the given U face with index (i,j,k) is bodering a value m
    template <class T>
    inline bool isUFaceBorderingValue(GridIndex g, T m, Array3D<T> &grid)
    {
        return isUFaceBorderingValue(g.i, g.j, g.k, m, grid);
    }

    // checks if the given V face with index(i, j, k) is bodering a value m
    template <class T>
    inline bool isVFaceBorderingValue(int i, int j, int k, T m, Array3D<T> &grid)
    {
        if (j == grid.height)
        {
            return grid(i, j - 1, k) == m;
        }
        else if (j > 0)
        {
            return grid(i, j, k) == m || grid(i, j - 1, k) == m;
        }
        else
        {
            return grid(i, j, k) == m;
        }
    }

    // checks if the given V face with index(i, j, k) is bodering a value m
    template <class T>
    inline bool isVFaceBorderingValue(GridIndex g, T m, Array3D<T> &grid)
    {
        return isVFaceBorderingValue(g.i, g.j, g.k, m, grid);
    }

    // checks if the given W face with index(i, j, k) is bodering a value m
    template <class T>
    inline bool isWFaceBorderingValue(int i, int j, int k, T m, Array3D<T> &grid)
    {
        if (k == grid.depth)
        {
            return grid(i, j, k - 1) == m;
        }
        else if (k > 0)
        {
            return grid(i, j, k) == m || grid(i, j, k - 1) == m;
        }
        else
        {
            return grid(i, j, k) == m;
        }
    }

    // checks if the given W face with index(i, j, k) is bodering a value m
    template <class T>
    inline bool isWFaceBorderingValue(GridIndex g, T m, Array3D<T> &grid)
    {
        return isWFaceBorderingValue(g.i, g.j, g.k, m, grid);
    }
}

#endif
