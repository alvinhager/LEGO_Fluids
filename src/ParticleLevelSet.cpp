#include "ParticleLevelSet.h"

ParticleLevelSet::ParticleLevelSet()
{
}

ParticleLevelSet::ParticleLevelSet(int i, int j, int k, double dx) : _isize(i), _jsize(j), _ksize(k), _dx(dx)
{
    _phi = Array3D<float>(i, j, k, _getMaxDistance());
}

ParticleLevelSet::~ParticleLevelSet()
{
}

float ParticleLevelSet::operator()(int i, int j, int k)
{
    return get(i, j, k);
}

float ParticleLevelSet::operator()(GridIndex g)
{
    return get(g);
}

float ParticleLevelSet::get(int i, int j, int k)
{
    CUSTOM_ASSERT(Grid3D::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize));
    return _phi(i, j, k);
}

float ParticleLevelSet::get(GridIndex g)
{
    CUSTOM_ASSERT(Grid3D::isGridIndexInRange(g, _isize, _jsize, _ksize));
    return _phi(g);
}

float ParticleLevelSet::getFaceWeightU(int i, int j, int k)
{
    CUSTOM_ASSERT(Grid3D::isGridIndexInRange(i, j, k, _isize + 1, _jsize, _ksize));
    return LevelsetFunctions::fractionInside(_phi(i - 1, j, k), _phi(i, j, k));
}

float ParticleLevelSet::getFaceWeightU(GridIndex g)
{
    return getFaceWeightU(g.i, g.j, g.k);
}

float ParticleLevelSet::getFaceWeightV(int i, int j, int k)
{
    CUSTOM_ASSERT(Grid3D::isGridIndexInRange(i, j, k, _isize, _jsize + 1, _ksize));
    return LevelsetFunctions::fractionInside(_phi(i, j - 1, k), _phi(i, j, k));
}

float ParticleLevelSet::getFaceWeightV(GridIndex g)
{
    return getFaceWeightV(g.i, g.j, g.k);
}

float ParticleLevelSet::getFaceWeightW(int i, int j, int k)
{
    CUSTOM_ASSERT(Grid3D::isGridIndexInRange(i, j, k, _isize, _jsize, _ksize + 1));
    return LevelsetFunctions::fractionInside(_phi(i, j, k - 1), _phi(i, j, k));
}

void ParticleLevelSet::calculateSignedDistanceField(std::vector<VectorMath::vec3> &particles,
                                                    double radius,
                                                    MeshLevelSet &solidPhi)
{
    int si, sj, sk;
    solidPhi.getGridDimensions(&si, &sj, &sk);
    CUSTOM_ASSERT(si == _isize && sj == _jsize && sk == _ksize);

    _computeSignedDistanceFromParticles(particles, radius);
    _extrapolateSignedDistanceIntoSolids(solidPhi);
}

float ParticleLevelSet::trilinearInterpolate(VectorMath::vec3 pos)
{
    float hdx = (float)(0.5 * _dx);
    VectorMath::vec3 offset = VectorMath::vec3(hdx, hdx, hdx);
    return (float)Interpolation::trilinearInterpolate(pos - offset, _dx, _phi);
}

float ParticleLevelSet::_getMaxDistance()
{
    return 3.0f * (float)_dx;
}

void ParticleLevelSet::_computeSignedDistanceFromParticles(std::vector<VectorMath::vec3> &particles,
                                                           double radius)
{
    _phi.fill(_getMaxDistance());

    GridIndex g, gmin, gmax;
    VectorMath::vec3 p;
    for (size_t pidx = 0; pidx < particles.size(); pidx++)
    {
        p = particles[pidx];
        g = Grid3D::positionToGridIndex(particles[pidx], _dx);
        gmin = GridIndex((int)fmax(0, g.i - 1), (int)fmax(0, g.j - 1), (int)fmax(0, g.k - 1));
        gmax = GridIndex((int)fmin(g.i + 1, _isize - 1),
                         (int)fmin(g.j + 1, _jsize - 1),
                         (int)fmin(g.k + 1, _ksize - 1));

        for (int k = gmin.k; k <= gmax.k; k++)
        {
            for (int j = gmin.j; j <= gmax.j; j++)
            {
                for (int i = gmin.i; i <= gmax.i; i++)
                {
                    VectorMath::vec3 cpos = Grid3D::GridIndexToCellCenter(i, j, k, _dx);
                    float dist = VectorMath::length(cpos - p) - (float)radius;
                    if (dist < _phi(i, j, k))
                    {
                        _phi.set(i, j, k, dist);
                    }
                }
            }
        }
    }
}

void ParticleLevelSet::_extrapolateSignedDistanceIntoSolids(MeshLevelSet &solidPhi)
{
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                if (_phi(i, j, k) < 0.5 * _dx)
                {
                    if (solidPhi.getDistanceAtCellCenter(i, j, k) < 0)
                    {
                        _phi.set(i, j, k, -0.5f * (float)_dx);
                    }
                }
            }
        }
    }
}