#ifndef MAC_VELOCITY_FIELD_AH
#define MAC_VELOCITY_FIELD_AH

#include "Array3D.h"
#include "Grid3D.h"
#include "VectorMath.h"
#include "ValidVelocityGrid.h"
#include "Interpolation.h"
#include "CustomAssert.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <limits>

/** MAC velocity field class that stores u,v,w velocity component grids */
class MACVelocityField
{

public:
    MACVelocityField();
    MACVelocityField(int i_size, int j_size, int k_size, double dx);
    ~MACVelocityField();

    void getGridDim(int *i, int *j, int *k);
    double getGridCellSize();

    float U(int i, int j, int k);
    float V(int i, int j, int k);
    float W(int i, int j, int k);
    float U(GridIndex g);
    float V(GridIndex g);
    float W(GridIndex g);

    void set(MACVelocityField &vfield);
    void setU(int i, int j, int k, double val);
    void setV(int i, int j, int k, double val);
    void setW(int i, int j, int k, double val);
    void setU(GridIndex g, double val);
    void setV(GridIndex g, double val);
    void setW(GridIndex g, double val);
    void setU(Array3D<float> &ugrid);
    void setV(Array3D<float> &vgrid);
    void setW(Array3D<float> &wgrid);
    void addU(int i, int j, int k, double val);
    void addV(int i, int j, int k, double val);
    void addW(int i, int j, int k, double val);

    Array3D<float> *getArray3DU();
    Array3D<float> *getArray3DV();
    Array3D<float> *getArray3DW();

    float *getRawArrayPointerU();
    float *getRawArrayPointerV();
    float *getRawArrayPointerW();

    void clear();
    void clearU();
    void clearV();
    void clearW();

    inline bool isIndexInRangeU(int i, int j, int k)
    {
        return Grid3D::isGridIndexInRange(i, j, k, _isize + 1, _jsize, _ksize);
    }
    inline bool isIndexInRangeV(int i, int j, int k)
    {
        return Grid3D::isGridIndexInRange(i, j, k, _isize, _jsize + 1, _ksize);
    }
    inline bool isIndexInRangeW(int i, int j, int k)
    {
        return Grid3D::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize + 1);
    }
    inline bool isIndexInRangeU(GridIndex g)
    {
        return Grid3D::isGridIndexInRange(g, _isize + 1, _jsize, _ksize);
    }
    inline bool isIndexInRangeV(GridIndex g)
    {
        return Grid3D::isGridIndexInRange(g, _isize, _jsize + 1, _ksize);
    }
    inline bool isIndexInRangeW(GridIndex g)
    {
        return Grid3D::isGridIndexInRange(g, _isize, _jsize, _ksize + 1);
    }

    VectorMath::vec3 evaluateVelocityAtCellCenter(int i, int j, int k);
    float evaluateVelocityMagnitudeAtCellCenter(int i, int j, int k);
    float evaluateVelocityMagnitudeSquaredAtCellCenter(int i, int j, int k);
    float evaluateMaximumVelocityMagnitude();

    VectorMath::vec3 evaluateVelocityAtFaceCenterU(int i, int j, int k);
    VectorMath::vec3 evaluateVelocityAtFaceCenterV(int i, int j, int k);
    VectorMath::vec3 evaluateVelocityAtFaceCenterW(int i, int j, int k);

    VectorMath::vec3 evaluateVelocityAtPosition(double x, double y, double z);
    VectorMath::vec3 evaluateVelocityAtPosition(VectorMath::vec3 pos);
    VectorMath::vec3 evaluateVelocityAtPositionLinear(double x, double y, double z);
    VectorMath::vec3 evaluateVelocityAtPositionLinear(VectorMath::vec3 pos);

    void extrapolateVelocityField(ValidVelocityGrid &validGrid, int numLayers);

private:
    int _isize = 10;
    int _jsize = 10;
    int _ksize = 10;
    double _dx = 0.1;

    Array3D<float> _u;
    Array3D<float> _v;
    Array3D<float> _w;

    int _numExtrapolationLayers = 0;
    void _initializeVelocityGrids();

    float _default_out_of_range_value = 0.0f;

    inline double _getRandomFloat(double min, double max)
    {
        return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    }

    double _interpolateU(double x, double y, double z);
    double _interpolateV(double x, double y, double z);
    double _interpolateW(double x, double y, double z);
    double _linearInterpolateU(double x, double y, double z);
    double _linearInterpolateV(double x, double y, double z);
    double _linearInterpolateW(double x, double y, double z);

    void _extrapolateGrid(Array3D<float> &grid, Array3D<bool> &valid, int numLayers);
};

#endif