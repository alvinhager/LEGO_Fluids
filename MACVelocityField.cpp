
#include "MACVelocityField.h"

MACVelocityField::MACVelocityField()
{
    _initializeVelocityGrids();
}

MACVelocityField::MACVelocityField(int isize, int jsize, int ksize, double dx) : _isize(isize), _jsize(jsize), _ksize(ksize), _dx(dx)
{
    _initializeVelocityGrids();
}

MACVelocityField::~MACVelocityField()
{
}

void MACVelocityField::_initializeVelocityGrids()
{
    _u = Array3D<float>(_isize + 1, _jsize, _ksize, 0.0f);
    _v = Array3D<float>(_isize, _jsize + 1, _ksize, 0.0f);
    _w = Array3D<float>(_isize, _jsize, _ksize + 1, 0.0f);

    _u.setOutOfRangeReturnValue(0.0f);
    _v.setOutOfRangeReturnValue(0.0f);
    _w.setOutOfRangeReturnValue(0.0f);
}

void MACVelocityField::getGridDim(int *i, int *j, int *k)
{
    *i = _isize;
    *j = _jsize;
    *k = _ksize;
}

double MACVelocityField::getGridCellSize()
{
    return _dx;
}

void MACVelocityField::clearU()
{
    _u.fill(0.0);
}

void MACVelocityField::clearV()
{
    _v.fill(0.0);
}

void MACVelocityField::clearW()
{
    _w.fill(0.0);
}

void MACVelocityField::clear()
{
    clearU();
    clearV();
    clearW();
}

Array3D<float> *MACVelocityField::getArray3DU()
{
    return &_u;
}

Array3D<float> *MACVelocityField::getArray3DV()
{
    return &_v;
}

Array3D<float> *MACVelocityField::getArray3DW()
{
    return &_w;
}

float *MACVelocityField::getRawArrayPointerU()
{
    return _u.getRawArray();
}

float *MACVelocityField::getRawArrayPointerV()
{
    return _v.getRawArray();
}

float *MACVelocityField::getRawArrayPointerW()
{
    return _w.getRawArray();
}

float MACVelocityField::U(int i, int j, int k)
{
    if (!isIndexInRangeU(i, j, k))
    {
        return _default_out_of_range_value;
    }

    return _u(i, j, k);
}

float MACVelocityField::V(int i, int j, int k)
{
    if (!isIndexInRangeV(i, j, k))
    {
        return _default_out_of_range_value;
    }

    return _v(i, j, k);
}

float MACVelocityField::W(int i, int j, int k)
{
    if (!isIndexInRangeW(i, j, k))
    {
        return _default_out_of_range_value;
    }

    return _w(i, j, k);
}

float MACVelocityField::U(GridIndex g)
{
    if (!isIndexInRangeU(g))
    {
        return _default_out_of_range_value;
    }

    return _u(g);
}

float MACVelocityField::V(GridIndex g)
{
    if (!isIndexInRangeV(g))
    {
        return _default_out_of_range_value;
    }

    return _v(g);
}

float MACVelocityField::W(GridIndex g)
{
    if (!isIndexInRangeW(g))
    {
        return _default_out_of_range_value;
    }

    return _w(g);
}

void MACVelocityField::set(MACVelocityField &vfield)
{
    int vi, vj, vk;
    vfield.getGridDim(&vi, &vj, &vk);
    CUSTOM_ASSERT(_isize == vi && _jsize == vj && _ksize == vk);

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize + 1; i++)
            {
                setU(i, j, k, vfield.U(i, j, k));
            }
        }
    }

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize + 1; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                setV(i, j, k, vfield.V(i, j, k));
            }
        }
    }

    for (int k = 0; k < _ksize + 1; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                setW(i, j, k, vfield.W(i, j, k));
            }
        }
    }
}

void MACVelocityField::setU(int i, int j, int k, double val)
{
    if (!isIndexInRangeU(i, j, k))
    {
        return;
    }

    _u.set(i, j, k, (float)val);
}

void MACVelocityField::setV(int i, int j, int k, double val)
{
    if (!isIndexInRangeV(i, j, k))
    {
        return;
    }

    _v.set(i, j, k, (float)val);
}

void MACVelocityField::setW(int i, int j, int k, double val)
{
    if (!isIndexInRangeW(i, j, k))
    {
        return;
    }

    _w.set(i, j, k, (float)val);
}

void MACVelocityField::setU(GridIndex g, double val)
{
    setU(g.i, g.j, g.k, val);
}

void MACVelocityField::setV(GridIndex g, double val)
{
    setV(g.i, g.j, g.k, val);
}

void MACVelocityField::setW(GridIndex g, double val)
{
    setW(g.i, g.j, g.k, val);
}

void MACVelocityField::setU(Array3D<float> &ugrid)
{
    CUSTOM_ASSERT(ugrid.width == _u.width &&
                  ugrid.height == _u.height &&
                  ugrid.depth == _u.depth);
    _u = ugrid;
}

void MACVelocityField::setV(Array3D<float> &vgrid)
{
    CUSTOM_ASSERT(vgrid.width == _v.width &&
                  vgrid.height == _v.height &&
                  vgrid.depth == _v.depth);
    _v = vgrid;
}

void MACVelocityField::setW(Array3D<float> &wgrid)
{
    CUSTOM_ASSERT(wgrid.width == _w.width &&
                  wgrid.height == _w.height &&
                  wgrid.depth == _w.depth);
    _w = wgrid;
}

void MACVelocityField::addU(int i, int j, int k, double val)
{
    if (!isIndexInRangeU(i, j, k))
    {
        return;
    }

    _u.add(i, j, k, (float)val);
}

void MACVelocityField::addV(int i, int j, int k, double val)
{
    if (!isIndexInRangeV(i, j, k))
    {
        return;
    }

    _v.add(i, j, k, (float)val);
}

void MACVelocityField::addW(int i, int j, int k, double val)
{
    if (!isIndexInRangeW(i, j, k))
    {
        return;
    }

    _w.add(i, j, k, (float)val);
}

VectorMath::vec3 MACVelocityField::evaluateVelocityAtCellCenter(int i, int j, int k)
{
    CUSTOM_ASSERT(Grid3D::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));

    double xavg = 0.5 * (U(i + 1, j, k) + U(i, j, k));
    double yavg = 0.5 * (V(i, j + 1, k) + V(i, j, k));
    double zavg = 0.5 * (W(i, j, k + 1) + W(i, j, k));

    return VectorMath::vec3((float)xavg, (float)yavg, (float)zavg);
}

float MACVelocityField::evaluateVelocityMagnitudeSquaredAtCellCenter(int i, int j, int k)
{
    CUSTOM_ASSERT(Grid3D::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));

    double xavg = 0.5 * (U(i + 1, j, k) + U(i, j, k));
    double yavg = 0.5 * (V(i, j + 1, k) + V(i, j, k));
    double zavg = 0.5 * (W(i, j, k + 1) + W(i, j, k));

    return (float)(xavg * xavg + yavg * yavg + zavg * zavg);
}

float MACVelocityField::evaluateVelocityMagnitudeAtCellCenter(int i, int j, int k)
{
    CUSTOM_ASSERT(Grid3D::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));

    double mag = evaluateVelocityMagnitudeSquaredAtCellCenter(i, j, k);
    if (mag > 0.0)
    {
        return (float)sqrt(mag);
    }
    else
    {
        return 0.0;
    }
}

float MACVelocityField::evaluateMaximumVelocityMagnitude()
{
    double maxsq = 0.0;
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {

                double m = evaluateVelocityMagnitudeSquaredAtCellCenter(i, j, k);
                maxsq = fmax(maxsq, m);
            }
        }
    }

    double max = maxsq;
    if (maxsq > 0.0)
    {
        max = sqrt(maxsq);
    }

    return (float)max;
}

VectorMath::vec3 MACVelocityField::evaluateVelocityAtFaceCenterU(int i, int j, int k)
{
    CUSTOM_ASSERT(isIndexInRangeU(i, j, k));
    i--;

    double vx = U(i + 1, j, k);
    double vy = 0.25 * (V(i, j, k) + V(i, j + 1, k) + V(i + 1, j, k) + V(i + 1, j + 1, k));
    double vz = 0.25 * (W(i, j, k) + W(i, j, k + 1) + W(i + 1, j, k) + W(i + 1, j, k + 1));

    return VectorMath::vec3((float)vx, (float)vy, (float)vz);
}

VectorMath::vec3 MACVelocityField::evaluateVelocityAtFaceCenterV(int i, int j, int k)
{
    CUSTOM_ASSERT(isIndexInRangeV(i, j, k));
    j--;

    double vx = 0.25 * (U(i, j, k) + U(i + 1, j, k) + U(i, j + 1, k) + U(i + 1, j + 1, k));
    double vy = V(i, j + 1, k);
    double vz = 0.25 * (W(i, j, k) + W(i, j, k + 1) + W(i, j + 1, k) + W(i, j + 1, k + 1));

    return VectorMath::vec3((float)vx, (float)vy, (float)vz);
}

VectorMath::vec3 MACVelocityField::evaluateVelocityAtFaceCenterW(int i, int j, int k)
{
    CUSTOM_ASSERT(isIndexInRangeW(i, j, k));
    k--;

    double vx = 0.25 * (U(i, j, k) + U(i + 1, j, k) + U(i, j, k + 1) + U(i + 1, j, k + 1));
    double vy = 0.25 * (V(i, j, k) + V(i, j + 1, k) + V(i, j, k + 1) + V(i, j + 1, k + 1));
    double vz = W(i, j, k + 1);

    return VectorMath::vec3((float)vx, (float)vy, (float)vz);
}

double MACVelocityField::_interpolateU(double x, double y, double z)
{
    if (!Grid3D::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize))
    {
        return 0.0;
    }

    y -= 0.5 * _dx;
    z -= 0.5 * _dx;

    int i, j, k;
    double gx, gy, gz;
    Grid3D::positionToGridIndex(x, y, z, _dx, &i, &j, &k);
    Grid3D::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (x - gx) * inv_dx;
    double iy = (y - gy) * inv_dx;
    double iz = (z - gz) * inv_dx;

    int refi = i - 1;
    int refj = j - 1;
    int refk = k - 1;

    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++)
    {
        for (int pj = 0; pj < 4; pj++)
        {
            for (int pi = 0; pi < 4; pi++)
            {
                points[pk][pj][pi] = U(pi + refi, pj + refj, pk + refk);
            }
        }
    }

    return Interpolation::tricubicInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_interpolateV(double x, double y, double z)
{
    if (!Grid3D::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize))
    {
        return 0.0;
    }

    x -= 0.5 * _dx;
    z -= 0.5 * _dx;

    int i, j, k;
    double gx, gy, gz;
    Grid3D::positionToGridIndex(x, y, z, _dx, &i, &j, &k);
    Grid3D::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (x - gx) * inv_dx;
    double iy = (y - gy) * inv_dx;
    double iz = (z - gz) * inv_dx;

    int refi = i - 1;
    int refj = j - 1;
    int refk = k - 1;

    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++)
    {
        for (int pj = 0; pj < 4; pj++)
        {
            for (int pi = 0; pi < 4; pi++)
            {
                points[pk][pj][pi] = V(pi + refi, pj + refj, pk + refk);
            }
        }
    }

    return Interpolation::tricubicInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_interpolateW(double x, double y, double z)
{
    if (!Grid3D::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize))
    {
        return 0.0;
    }

    x -= 0.5 * _dx;
    y -= 0.5 * _dx;

    int i, j, k;
    double gx, gy, gz;
    Grid3D::positionToGridIndex(x, y, z, _dx, &i, &j, &k);
    Grid3D::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (x - gx) * inv_dx;
    double iy = (y - gy) * inv_dx;
    double iz = (z - gz) * inv_dx;

    int refi = i - 1;
    int refj = j - 1;
    int refk = k - 1;

    double points[4][4][4];
    for (int pk = 0; pk < 4; pk++)
    {
        for (int pj = 0; pj < 4; pj++)
        {
            for (int pi = 0; pi < 4; pi++)
            {
                points[pk][pj][pi] = W(pi + refi, pj + refj, pk + refk);
            }
        }
    }

    return Interpolation::tricubicInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_linearInterpolateU(double x, double y, double z)
{
    if (!Grid3D::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize))
    {
        return 0.0;
    }

    y -= 0.5 * _dx;
    z -= 0.5 * _dx;

    int i, j, k;
    double gx, gy, gz;
    Grid3D::positionToGridIndex(x, y, z, _dx, &i, &j, &k);
    Grid3D::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (x - gx) * inv_dx;
    double iy = (y - gy) * inv_dx;
    double iz = (z - gz) * inv_dx;

    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (_u.isIndexInRange(i, j, k))
    {
        points[0] = _u(i, j, k);
    }
    if (_u.isIndexInRange(i + 1, j, k))
    {
        points[1] = _u(i + 1, j, k);
    }
    if (_u.isIndexInRange(i, j + 1, k))
    {
        points[2] = _u(i, j + 1, k);
    }
    if (_u.isIndexInRange(i, j, k + 1))
    {
        points[3] = _u(i, j, k + 1);
    }
    if (_u.isIndexInRange(i + 1, j, k + 1))
    {
        points[4] = _u(i + 1, j, k + 1);
    }
    if (_u.isIndexInRange(i, j + 1, k + 1))
    {
        points[5] = _u(i, j + 1, k + 1);
    }
    if (_u.isIndexInRange(i + 1, j + 1, k))
    {
        points[6] = _u(i + 1, j + 1, k);
    }
    if (_u.isIndexInRange(i + 1, j + 1, k + 1))
    {
        points[7] = _u(i + 1, j + 1, k + 1);
    }

    return Interpolation::trilinearInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_linearInterpolateV(double x, double y, double z)
{
    if (!Grid3D::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize))
    {
        return 0.0;
    }

    x -= 0.5 * _dx;
    z -= 0.5 * _dx;

    int i, j, k;
    double gx, gy, gz;
    Grid3D::positionToGridIndex(x, y, z, _dx, &i, &j, &k);
    Grid3D::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (x - gx) * inv_dx;
    double iy = (y - gy) * inv_dx;
    double iz = (z - gz) * inv_dx;

    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (_v.isIndexInRange(i, j, k))
    {
        points[0] = _v(i, j, k);
    }
    if (_v.isIndexInRange(i + 1, j, k))
    {
        points[1] = _v(i + 1, j, k);
    }
    if (_v.isIndexInRange(i, j + 1, k))
    {
        points[2] = _v(i, j + 1, k);
    }
    if (_v.isIndexInRange(i, j, k + 1))
    {
        points[3] = _v(i, j, k + 1);
    }
    if (_v.isIndexInRange(i + 1, j, k + 1))
    {
        points[4] = _v(i + 1, j, k + 1);
    }
    if (_v.isIndexInRange(i, j + 1, k + 1))
    {
        points[5] = _v(i, j + 1, k + 1);
    }
    if (_v.isIndexInRange(i + 1, j + 1, k))
    {
        points[6] = _v(i + 1, j + 1, k);
    }
    if (_v.isIndexInRange(i + 1, j + 1, k + 1))
    {
        points[7] = _v(i + 1, j + 1, k + 1);
    }

    return Interpolation::trilinearInterpolate(points, ix, iy, iz);
}

double MACVelocityField::_linearInterpolateW(double x, double y, double z)
{
    if (!Grid3D::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize))
    {
        return 0.0;
    }

    x -= 0.5 * _dx;
    y -= 0.5 * _dx;

    int i, j, k;
    double gx, gy, gz;
    Grid3D::positionToGridIndex(x, y, z, _dx, &i, &j, &k);
    Grid3D::GridIndexToPosition(i, j, k, _dx, &gx, &gy, &gz);

    double inv_dx = 1 / _dx;
    double ix = (x - gx) * inv_dx;
    double iy = (y - gy) * inv_dx;
    double iz = (z - gz) * inv_dx;

    double points[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (_w.isIndexInRange(i, j, k))
    {
        points[0] = _w(i, j, k);
    }
    if (_w.isIndexInRange(i + 1, j, k))
    {
        points[1] = _w(i + 1, j, k);
    }
    if (_w.isIndexInRange(i, j + 1, k))
    {
        points[2] = _w(i, j + 1, k);
    }
    if (_w.isIndexInRange(i, j, k + 1))
    {
        points[3] = _w(i, j, k + 1);
    }
    if (_w.isIndexInRange(i + 1, j, k + 1))
    {
        points[4] = _w(i + 1, j, k + 1);
    }
    if (_w.isIndexInRange(i, j + 1, k + 1))
    {
        points[5] = _w(i, j + 1, k + 1);
    }
    if (_w.isIndexInRange(i + 1, j + 1, k))
    {
        points[6] = _w(i + 1, j + 1, k);
    }
    if (_w.isIndexInRange(i + 1, j + 1, k + 1))
    {
        points[7] = _w(i + 1, j + 1, k + 1);
    }

    return Interpolation::trilinearInterpolate(points, ix, iy, iz);
}

VectorMath::vec3 MACVelocityField::evaluateVelocityAtPosition(VectorMath::vec3 pos)
{
    return evaluateVelocityAtPosition(pos.x, pos.y, pos.z);
}

VectorMath::vec3 MACVelocityField::evaluateVelocityAtPosition(double x, double y, double z)
{
    if (!Grid3D::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize))
    {
        return VectorMath::vec3();
    }

    double xvel = _interpolateU(x, y, z);
    double yvel = _interpolateV(x, y, z);
    double zvel = _interpolateW(x, y, z);

    return VectorMath::vec3((float)xvel, (float)yvel, (float)zvel);
}

VectorMath::vec3 MACVelocityField::evaluateVelocityAtPositionLinear(VectorMath::vec3 pos)
{
    return evaluateVelocityAtPositionLinear(pos.x, pos.y, pos.z);
}

VectorMath::vec3 MACVelocityField::evaluateVelocityAtPositionLinear(double x, double y, double z)
{
    if (!Grid3D::isPositionInGrid(x, y, z, _dx, _isize, _jsize, _ksize))
    {
        return VectorMath::vec3();
    }

    double xvel = _linearInterpolateU(x, y, z);
    double yvel = _linearInterpolateV(x, y, z);
    double zvel = _linearInterpolateW(x, y, z);

    return VectorMath::vec3((float)xvel, (float)yvel, (float)zvel);
}

void MACVelocityField::_extrapolateGrid(Array3D<float> &grid, Array3D<bool> &valid, int numLayers)
{

    char UNKNOWN = 0x00;
    char WAITING = 0x01;
    char KNOWN = 0x02;
    char DONE = 0x03;

    Array3D<char> status(grid.width, grid.height, grid.depth);
    for (int k = 0; k < grid.depth; k++)
    {
        for (int j = 0; j < grid.height; j++)
        {
            for (int i = 0; i < grid.width; i++)
            {
                status.set(i, j, k, valid(i, j, k) ? KNOWN : UNKNOWN);
                if (status(i, j, k) == UNKNOWN &&
                    Grid3D::isGridIndexOnBorder(i, j, k, grid.width, grid.height, grid.depth))
                {
                    status.set(i, j, k, DONE);
                }
            }
        }
    }

    std::vector<GridIndex> extrapolationCells;
    for (int layers = 0; layers < numLayers; layers++)
    {

        extrapolationCells.clear();
        for (int k = 1; k < grid.depth - 1; k++)
        {
            for (int j = 1; j < grid.height - 1; j++)
            {
                for (int i = 1; i < grid.width - 1; i++)
                {
                    if (status(i, j, k) != KNOWN)
                    {
                        continue;
                    }

                    int count = 0;
                    if (status(i - 1, j, k) == UNKNOWN)
                    {
                        extrapolationCells.push_back(GridIndex(i - 1, j, k));
                        status.set(i - 1, j, k, WAITING);
                        count++;
                    }
                    else if (status(i - 1, j, k) == WAITING)
                    {
                        count++;
                    }

                    if (status(i + 1, j, k) == UNKNOWN)
                    {
                        extrapolationCells.push_back(GridIndex(i + 1, j, k));
                        status.set(i + 1, j, k, WAITING);
                        count++;
                    }
                    else if (status(i + 1, j, k) == WAITING)
                    {
                        count++;
                    }

                    if (status(i, j - 1, k) == UNKNOWN)
                    {
                        extrapolationCells.push_back(GridIndex(i, j - 1, k));
                        status.set(i, j - 1, k, WAITING);
                        count++;
                    }
                    else if (status(i, j - 1, k) == WAITING)
                    {
                        count++;
                    }

                    if (status(i, j + 1, k) == UNKNOWN)
                    {
                        extrapolationCells.push_back(GridIndex(i, j + 1, k));
                        status.set(i, j + 1, k, WAITING);
                        count++;
                    }
                    else if (status(i, j + 1, k) == WAITING)
                    {
                        count++;
                    }

                    if (status(i, j, k - 1) == UNKNOWN)
                    {
                        extrapolationCells.push_back(GridIndex(i, j, k - 1));
                        status.set(i, j, k - 1, WAITING);
                        count++;
                    }
                    else if (status(i, j, k - 1) == WAITING)
                    {
                        count++;
                    }

                    if (status(i, j, k + 1) == UNKNOWN)
                    {
                        extrapolationCells.push_back(GridIndex(i, j, k + 1));
                        status.set(i, j, k + 1, WAITING);
                        count++;
                    }
                    else if (status(i, j, k + 1) == WAITING)
                    {
                        count++;
                    }

                    if (count == 0)
                    {
                        status.set(i, j, k, DONE);
                    }
                }
            }
        }

        GridIndex g;
        for (size_t i = 0; i < extrapolationCells.size(); i++)
        {
            g = extrapolationCells[i];

            float sum = 0;
            int count = 0;
            if (status(g.i - 1, g.j, g.k) == KNOWN)
            {
                sum += grid(g.i - 1, g.j, g.k);
                count++;
            }
            if (status(g.i + 1, g.j, g.k) == KNOWN)
            {
                sum += grid(g.i + 1, g.j, g.k);
                count++;
            }
            if (status(g.i, g.j - 1, g.k) == KNOWN)
            {
                sum += grid(g.i, g.j - 1, g.k);
                count++;
            }
            if (status(g.i, g.j + 1, g.k) == KNOWN)
            {
                sum += grid(g.i, g.j + 1, g.k);
                count++;
            }
            if (status(g.i, g.j, g.k - 1) == KNOWN)
            {
                sum += grid(g.i, g.j, g.k - 1);
                count++;
            }
            if (status(g.i, g.j, g.k + 1) == KNOWN)
            {
                sum += grid(g.i, g.j, g.k + 1);
                count++;
            }

            CUSTOM_ASSERT(count != 0)
            grid.set(g, sum / (float)count);
        }
        status.set(extrapolationCells, KNOWN);
    }
}

void MACVelocityField::extrapolateVelocityField(ValidVelocityGrid &validGrid, int numLayers)
{
    _extrapolateGrid(_u, validGrid.validU, numLayers);
    _extrapolateGrid(_v, validGrid.validV, numLayers);
    _extrapolateGrid(_w, validGrid.validW, numLayers);
}