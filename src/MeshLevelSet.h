#ifndef MESHLEVELSET_AH
#define MESHLEVELSET_AH

#include "VectorMath.h"
#include "Grid3D.h"
#include "Array3D.h"
#include "TriangleMesh.h"
#include "Triangle.h"
#include "Interpolation.h"
#include "LevelSetFunctions.h"

class MeshLevelSet
{

public:
    MeshLevelSet();
    MeshLevelSet(int isize, int jsize, int ksize, double dx);
    ~MeshLevelSet();

    float operator()(int i, int j, int k);
    float operator()(GridIndex g);
    float get(int i, int j, int k);
    float get(GridIndex g);
    int getClosestTriangleIndex(int i, int j, int k);
    int getClosestTriangleIndex(GridIndex g);
    float getDistanceAtCellCenter(int i, int j, int k);
    float getDistanceAtCellCenter(GridIndex g);
    float trilinearInterpolate(VectorMath::vec3 pos);
    VectorMath::vec3 trilinearInterpolateGradient(VectorMath::vec3 pos);
    float getFaceWeightU(int i, int j, int k);
    float getFaceWeightU(GridIndex g);
    float getFaceWeightV(int i, int j, int k);
    float getFaceWeightV(GridIndex g);
    float getFaceWeightW(int i, int j, int k);
    float getFaceWeightW(GridIndex g);

    void getGridDimensions(int *i, int *j, int *k);
    TriangleMesh *getTriangleMesh();

    void calculateSignedDistanceField(TriangleMesh &m, int bandwidth = 1);
    void calculateUnion(MeshLevelSet &levelset);
    void negate();

    Array3D<float> _phi;

private:
    void _computeExactBandDistanceField(int bandwidth,
                                        Array3D<int> &intersectionCounts);
    void _propagateDistanceField();
    void _computeDistanceFieldSigns(Array3D<int> &intersectionCounts);
    float _pointToTriangleDistance(VectorMath::vec3 x0, VectorMath::vec3 x1,
                                   VectorMath::vec3 x2,
                                   VectorMath::vec3 x3);
    bool _getBarycentricCoordinates(
        double x0, double y0,
        double x1, double y1, double x2, double y2, double x3, double y3,
        double *a, double *b, double *c);
    float _pointToSegmentDistance(VectorMath::vec3 x0, VectorMath::vec3 x1, VectorMath::vec3 x2);
    int _orientation(double x1, double y1, double x2, double y2, double *twiceSignedArea);

    template <typename T>
    T _clamp(const T &n, const T &lower, const T &upper)
    {
        return std::max(lower, std::min(n, upper));
    }

    int _isize = 0;
    int _jsize = 0;
    int _ksize = 0;
    double _dx = 0.0;

    TriangleMesh _mesh;
    //Array3D<float> _phi;
    Array3D<int> _closestTriangles;
};

#endif
