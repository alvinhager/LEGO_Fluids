#ifndef PARTICLELEVELSET_AH
#define PARTICLELEVELSET_AH

#include <vector>

#include "Array3D.h"
#include "Grid3d.h"
#include "LevelSetFunctions.h"
#include "MeshLevelSet.h"
#include "CustomAssert.h"

class ParticleLevelSet
{

public:
    ParticleLevelSet();
    ParticleLevelSet(int i, int j, int k, double dx);
    ~ParticleLevelSet();

    float operator()(int i, int j, int k);
    float operator()(GridIndex g);
    float get(int i, int j, int k);
    float get(GridIndex g);
    float getFaceWeightU(int i, int j, int k);
    float getFaceWeightU(GridIndex g);
    float getFaceWeightV(int i, int j, int k);
    float getFaceWeightV(GridIndex g);
    float getFaceWeightW(int i, int j, int k);
    float getFaceWeightW(GridIndex g);

    void calculateSignedDistanceField(std::vector<VectorMath::vec3> &particles,
                                      double radius,
                                      MeshLevelSet &solidPhi);
    float trilinearInterpolate(VectorMath::vec3 pos);

private:
    float _getMaxDistance();
    void _computeSignedDistanceFromParticles(std::vector<VectorMath::vec3> &particles,
                                             double radius);
    void _extrapolateSignedDistanceIntoSolids(MeshLevelSet &solidPhi);

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;
    Array3D<float> _phi;
};

#endif
