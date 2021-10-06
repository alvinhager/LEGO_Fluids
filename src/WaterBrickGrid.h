#pragma once

#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include "AABB.h"
#include "Array3D.h"
#include "Brick.h"
#include "TriangleMesh.h"
#include "CustomAssert.h"
#include "Grid3D.h"
#include "ParticleLevelSet.h"
#include "GridIndexVector.h"
#include "FluidParticle.h"

class WaterBrickGrid
{

public:
    WaterBrickGrid();
    WaterBrickGrid(int isize, int jsize, int ksize, double dx, AABB brick);
    ~WaterBrickGrid();

    void getGridDim(int *i, int *j, int *k);
    double getCellSize();
    void getBrickGridDim(int *i, int *j, int *k);
    AABB getBrickAABB();

    void setBrickDim(double width, double height, double depth);
    void setBrickDim(AABB brick);

    bool getBrickMesh(TriangleMesh &mesh);

    bool isBrickMeshReady();

    int getNumUpdates();

    void update(ParticleLevelSet &liquidSDF, MeshLevelSet solidSDF, std::vector<VectorMath::vec3> &particles, double dt);
    void update2(ParticleLevelSet &liquidSDF, MeshLevelSet solidSDF, std::vector<FluidParticle> &particles, double dt);

    void getDensityGridCurrentDensityValues(Array3D<float> &grid);
    void getDensityGridTargetDensityValues(Array3D<float> &grid);
    void getDensityGridVelocityValues(Array3D<float> &grid);

    Array3D<Brick> *getPointerToBrickGridQueue();
    bool isBrickGridInitialized();

    void _updateDensityGrid2(std::vector<VectorMath::vec3> &points);
    void _updateBrickGrid2(ParticleLevelSet &liquidSDF, MeshLevelSet solidSDF);
    float _getBrickIntensity2(int i, int j, int k);
    void _updateAcceleration(ParticleLevelSet &liquidSDF, MeshLevelSet solidSDF, std::vector<FluidParticle> &particles, double dt);

private:
    struct DensityInfo
    {
        float currentDensity = 0.0f;
        float targetDensity = 0.0f;
        float velocityIntensity = 0.0f;
    };

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;

    AABB _brick;
    Array3D<DensityInfo> _densityGrid;
    Array3D<Brick> _brickGrid;
    Array3D<Brick> _brickGridQueue[3];
    int _brickGridQueueSize = 0;
    Array3D<Brick> _currentBrickGrid;

    struct VelocityInfo
    {
        VectorMath::vec3 currentVelocity = VectorMath::vec3(0.0f, 0.0f, 0.0f);
        VectorMath::vec3 previousVelocity = VectorMath::vec3(0.0f, 0.0f, 0.0f);
    };

    Array3D<VelocityInfo> _velocityGrid;

    struct DensityInfo2
    {
        float currentDensity = 0.0f;
        float prevDensity = 0.0f;
        float currentVelocity = 0.0f;
        float targetVelocity = 0.0f;
    };

    Array3D<DensityInfo2> _densityGrid2;
    //Array3D<float> _intensityValues;

    //necessary?
    bool _isCurrentBrickGridReady = false;

    int _minParticleDensity = 0;
    int _maxParticleDensity = 8;
    float _maxVelocityIntensity = 10.0f;
    float _maxAccelerationIntensity = 10.0f;
    float _decelerationRadius = 0.05f;
    unsigned int _minNumberOfBricksInStructure = 0;

    int _numUpdates = 0;
    bool _isBrickGridInitialized = false;

    void _initialize();
    void _initializeBrickGrid();
    void _reset();
    void _updateDensityGrid(std::vector<VectorMath::vec3> &particles, double dt);
    void _updateTargetDensities(std::vector<VectorMath::vec3> &particles);
    void _updateDensities(double dt);
    void _updateDensity(int i, int j, int k, double dt);

    void _updateBrickGrid(ParticleLevelSet &liquidSDF, MeshLevelSet solidSDF);

    float _getBrickIntensityAcceleration(int i, int j, int k, double dt);

    float _getBrickIntensity(int i, int j, int k);
    bool _isBrickNextToActiveNeighbour(int i, int j, int k);
    void _postProcessBrickGrid();
    void _removeStrayBricks();
    void _removeSmallBrickStructures();
    void _mergeBrickGrids();
    void _getNewBrickLocations(Array3D<Brick> &b1, Array3D<Brick> &b2, Array3D<bool> &newBricks);
    void _getbrickStructures(Array3D<Brick> &brickGrid, Array3D<bool> &newBricks, std::vector<GridIndexVector> &brickStructures);
    void _getConnectedBricks(int i, int j, int k, Array3D<Brick> &brickGrid, Array3D<bool> &newBricks, GridIndexVector &connectedBricks);
    void _removeInvalidbrickStructures(Array3D<Brick> &brickCurrent, Array3D<Brick> &brickNext, std::vector<GridIndexVector> &brickStructures);
    bool _isBrickMassInBrickGrid(GridIndexVector &cells, Array3D<Brick> &brickGrid);
    void _removeBrickStructureFromBrickGrid(GridIndexVector &cells, Array3D<Brick> &brickGrid);
    void _getBrickIntensity();
    int getBrickGridQueueSize();
};
