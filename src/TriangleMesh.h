#ifndef TRIANGLEMESH_H
#define TRIANGLEMESH_H

#include <stdlib.h>
#include <queue>
#include <vector>
#include <sstream>
#include <fstream>
#include <string.h>
#include <algorithm>

#include "Triangle.h"
#include "VectorMath.h"

class TriangleMesh
{
public:
    TriangleMesh();
    ~TriangleMesh();

    bool loadPLY(std::string PLYFilename);
    bool loadASCIIPLY(std::string PLYFilename);
    bool loadBOBJ(std::string BOBJFilename);
    bool loadOBJ(std::string filename);
    void writeMeshToPLY(std::string filename);
    void writeMeshToASCIIPLY(std::string filename);
    void writeMeshToBOBJ(std::string filename);
    void writeMeshToOBJ(std::string filename);

    int numVertices();
    int numFaces();
    int numTriangles() { return numFaces(); }
    void translate(VectorMath::vec3 t);

    std::vector<VectorMath::vec3> vertices;
    std::vector<VectorMath::vec3> vertexcolors; // r, g, b values in range [0.0, 1.0]
    std::vector<VectorMath::vec3> normals;
    std::vector<Triangle> triangles;

private:
    bool _getPLYHeader(std::ifstream *file, std::string *header);
    bool _getElementNumberInPlyHeader(std::string &header,
                                      std::string &element, int *n);
    bool _getNumVerticesInPLYHeader(std::string &header, int *n);
    bool _getNumFacesInPLYHeader(std::string &header, int *n);
    bool _isVertexColorsEnabledInPLYHeader(std::string &header);
    bool _loadPLYVertexData(std::ifstream *file, std::string &header);
    bool _loadPLYTriangleData(std::ifstream *file, std::string &header);

    bool _loadPLYVertexDataASCII(std::ifstream *file, std::string &header);

    int _numDigitsInInteger(int num);

    template <class T>
    std::string _toString(T item)
    {
        std::ostringstream sstream;
        sstream << item;

        return sstream.str();
    }
};

#endif
