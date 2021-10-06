#ifndef FLUIDSIMULATION_H
#define FLUIDSIMULATION_H

#include "VectorMath.h"
#include "Array3D.h"
#include "Grid3D.h"
#include "MACVelocityField.h"
#include "ParticleLevelSet.h"
#include "Interpolation.h"
#include "PressureSolver.h"
#include "MeshLevelSet.h"
#include "CustomAssert.h"
#include <vector>
#include <string.h>
#include "WaterBrickGrid.h"
#include "GeneralUtilityFunctions.h"
#include "TriangleMeshFormat.h"
#include "FluidParticle.h"
#include "CUDAFunctions.h"

class FluidSimulation
{

public:
    std::vector<FluidParticle> particles;
    bool _wyvillkernelUsedIfNotUsingGPU = true;

    MACVelocityField &getMACVelocityField();
    void initialize(int i, int j, int k, float dx);
    void addBoundary(TriangleMesh &boundary, bool isInverted = false);
    void resetBoundary();
    void addLiquid(TriangleMesh &mesh);
    void setGravity(VectorMath::vec3 gravity);
    void setGravity(float gx, float gy, float gz);
    void step(float dt);

    float getPICPercentage();
    void setPICPercentage(float percentage);
    float getFLIPPercentage();
    void setFLIPPercentage(float percentage);

    void getDimensions(int *i, int *j, int *k);
    void getDimensions(int &i, int &j, int &k);
    float getdx();
    int getNumberOfParticlesSeededPerCell();
    void setNumberOfParticlesSeededPerCell(int no_particles);

    void setIsUsingGPU(bool isUsingGPU);
    bool getIsUsingGPU();

    bool isBrickOutputEnabled();
    void enableBrickOutput(AABB brickbbox);
    void enableBrickOutput(double width, double height, double depth);
    void outputBrickMesh(int frameno, double dt);

private:
    //Grid dimensions
    int _isize;
    int _jsize;
    int _ksize;
    float _dx;
    float _no_particles_seeded_per_cell = 8;
    float _FLIP_percentage = 0.95f;
    bool _isUsingGPU = true;

    //Fluid velocity
    MACVelocityField _MACVelocity;
    MACVelocityField _savedVelocityField;
    ValidVelocityGrid _validVelocities;

    MeshLevelSet _solidSDF;
    int _meshLevelSetExactBand = 3;

    ParticleLevelSet _liquidSDF;
    float _particleRadius;

    WeightGrid _weightGrid;

    float _CFLConditionNumber = 5.0;
    float _minfrac = 0.01f;

    Array3D<float> _viscosity;
    VectorMath::vec3 _gravity;

    //brick grid
    bool _isBrickOutputEnabled = false;
    int _currentBrickMeshFrame = 0;
    WaterBrickGrid _waterBrickGrid;

    TriangleMeshFormat _brickMeshOutputFormat = TriangleMeshFormat::obj;
    VectorMath::vec3 _domainOffset = VectorMath::vec3(0.0, 0.0, 0.0);

    TriangleMesh _getTriangleMeshFromAABB(AABB bbox);
    TriangleMesh _getBoundaryTriangleMesh();
    void _initializeBoundary();
    VectorMath::vec3 _traceRK2(VectorMath::vec3 position, float dt);
    VectorMath::vec3 _traceRK4(VectorMath::vec3 position, float dt);

    float _cfl();
    VectorMath::vec3 _getVelocity(VectorMath::vec3 position);
    void _updateLiquidSDF();

    void _advectVelocityField();
    void _advectVelocityFieldU(Array3D<bool> &fluidCellGrid);
    void _advectVelocityFieldV(Array3D<bool> &fluidCellGrid);
    void _advectVelocityFieldW(Array3D<bool> &fluidCellGrid);
    void _computeVelocityScalarField(Array3D<float> &field, Array3D<bool> &isValueSet, int dir);

    void _addBodyForce(float dt);
    void _project(float dt);
    void _extrapolateVelocityField(MACVelocityField &vfield, ValidVelocityGrid &valid);
    void _constrainVelocityField();

    // added methods
    std::string _getFrameString(int number);
    void _updateBrickGrid(int dt);

    void _writeBrickMaterialToFile(std::string brickfile,
                                   std::string colorfile);

    void _writeBrickTriangleMeshToFile(TriangleMesh &mesh, std::string filename);

    void _writeBrickColorListToFile(TriangleMesh &mesh, std::string filename);

    //helpers for pressure projection
    void _computeWeights();
    Array3D<float> _solvePressure(float dt);
    void _applyPressure(float dt, Array3D<float> &pressureGrid);

    void _advectFluidParticles(float dt);
    void _updateFluidParticleVelocities();

    inline double _randomDouble(double min, double max)
    {
        return min + (double)rand() / ((double)RAND_MAX / (max - min));
    }

    template <typename T>
    T _clamp(const T &n, const T &lower, const T &upper)
    {
        return std::max(lower, std::min(n, upper));
    }
};

#endif