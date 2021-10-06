#include "FluidSimulation.h"

void FluidSimulation::initialize(int i, int j, int k, float dx)
{
    _isize = i;
    _jsize = j;
    _ksize = k;
    _dx = dx;

    _MACVelocity = MACVelocityField(_isize, _jsize, _ksize, _dx);
    _validVelocities = ValidVelocityGrid(_isize, _jsize, _ksize);

    // make the particles large enough so they always appear on the grid
    _particleRadius = (float)(_dx * 1.01 * sqrt(3.0) / 2.0);
    _liquidSDF = ParticleLevelSet(_isize, _jsize, _ksize, _dx);
    _weightGrid = WeightGrid(_isize, _jsize, _ksize);
    _viscosity = Array3D<float>(_isize + 1, _jsize + 1, _ksize + 1, 1.0);
    _gravity = VectorMath::vec3(0.0f, -9.81f, 0.0f);

    _initializeBoundary();
}

void FluidSimulation::addBoundary(TriangleMesh &boundary, bool isInverted)
{
    AABB domain(0.0, 0.0, 0.0, _isize * _dx, _jsize * _dx, _ksize * _dx);
    AABB bbox(boundary.vertices);
    CUSTOM_ASSERT(domain.isPointInside(bbox.getMinPoint()) &&
                  domain.isPointInside(bbox.getMaxPoint()));

    MeshLevelSet boundarySDF(_isize, _jsize, _ksize, _dx);
    boundarySDF.calculateSignedDistanceField(boundary, _meshLevelSetExactBand);
    if (isInverted)
    {
        boundarySDF.negate();
    }

    _solidSDF.calculateUnion(boundarySDF);
}

void FluidSimulation::resetBoundary()
{
    _initializeBoundary();
}

void FluidSimulation::setNumberOfParticlesSeededPerCell(int no_particles)
{
    _no_particles_seeded_per_cell = no_particles;
}

int FluidSimulation::getNumberOfParticlesSeededPerCell()
{
    return _no_particles_seeded_per_cell;
}
void FluidSimulation::addLiquid(TriangleMesh &mesh)
{
    AABB domain(0.0, 0.0, 0.0, _isize * _dx, _jsize * _dx, _ksize * _dx);
    AABB bbox(mesh.vertices);
    CUSTOM_ASSERT(domain.isPointInside(bbox.getMinPoint()) &&
                  domain.isPointInside(bbox.getMaxPoint()));

    MeshLevelSet meshSDF(_isize, _jsize, _ksize, _dx);
    meshSDF.calculateSignedDistanceField(mesh, _meshLevelSetExactBand);

    // initialize particles
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                VectorMath::vec3 gpos = Grid3D::GridIndexToPosition(i, j, k, _dx);

                for (int i_dx = 0; i_dx < _no_particles_seeded_per_cell; i_dx++)
                {
                    float a = (float)_randomDouble(0.0, _dx);
                    float b = (float)_randomDouble(0.0, _dx);
                    float c = (float)_randomDouble(0.0, _dx);
                    VectorMath::vec3 jitter = VectorMath::vec3(a, b, c);
                    VectorMath::vec3 pos = gpos + jitter;

                    if (meshSDF.trilinearInterpolate(pos) < 0.0)
                    {
                        float solid_phi = _solidSDF.trilinearInterpolate(pos);
                        if (solid_phi >= 0)
                            particles.push_back(pos);
                    }
                }
            }
        }
    }
}

void FluidSimulation::setGravity(VectorMath::vec3 gravity)
{
    _gravity = gravity;
}

void FluidSimulation::setGravity(float gx, float gy, float gz)
{
    setGravity(VectorMath::vec3(gx, gy, gz));
}

void FluidSimulation::setFLIPPercentage(float FLIP_percentage)
{
    _FLIP_percentage = FLIP_percentage;
}
void FluidSimulation::setPICPercentage(float PIC_percentage)
{
    _FLIP_percentage = 1.0f - PIC_percentage;
}
float FluidSimulation::getFLIPPercentage()
{
    return _FLIP_percentage;
}
float FluidSimulation::getPICPercentage()
{
    return 1.0f - _FLIP_percentage;
}

//The main fluid simulation step
void FluidSimulation::step(float dt)
{
    float t = 0;
    while (t < dt)
    {
        float substep = _cfl();

        if (t + substep > dt)
            substep = dt - t;

        _updateLiquidSDF();
        _advectVelocityField();
        _addBodyForce(substep);
        _project(substep);
        _constrainVelocityField();
        _advectFluidParticles(substep);

        t += substep;
    }
}

void FluidSimulation::enableBrickOutput(double brick_width, double brick_height, double brick_depth)
{
    if (!(brick_width > 0.0 && brick_height > 0.0 && brick_depth > 0.0))
    {
        std::string msg = "Error, brick dim must be greater than 0.\n";
        msg += "brick width: " + std::to_string(brick_width) +
               "brick height: " + std::to_string(brick_height) +
               "brick depth: " + std::to_string(brick_depth) + "\n";
        throw std::domain_error(msg);
    }
    AABB brick = AABB(VectorMath::vec3(), brick_width, brick_height, brick_depth);
    int i, j, k;
    _waterBrickGrid.getGridDim(&i, &j, &k);
    if (i != _isize || j != _jsize || k != _ksize)
        _waterBrickGrid = WaterBrickGrid(_isize, _jsize, _ksize, _dx, brick);
    _waterBrickGrid.setBrickDim(brick);
    _isBrickOutputEnabled = true;
}

void FluidSimulation::enableBrickOutput(AABB brickbbox)
{
    enableBrickOutput(brickbbox.width, brickbbox.height, brickbbox.depth);
}

bool FluidSimulation::isBrickOutputEnabled()
{
    return _isBrickOutputEnabled;
}

void FluidSimulation::outputBrickMesh(int frameno, double dt)
{
    if (!_isBrickOutputEnabled)
    {
        return;
    }
    _updateBrickGrid(dt);
    if (!_waterBrickGrid.isBrickMeshReady())
        return;

    std::string framestr = _getFrameString(frameno);

    _writeBrickMaterialToFile("brick_locations/brickloc" + framestr + ".obj", "brick_colors/brickcol" + framestr + ".data");
}

std::string FluidSimulation::_getFrameString(int number)
{
    std::string currentFrame = GeneralUtilityFunctions::numberToString(number);
    currentFrame.insert(currentFrame.begin(), 6 - currentFrame.size(), '0');
    return currentFrame;
};

void FluidSimulation::_updateBrickGrid(int dt)
{

    std::vector<VectorMath::vec3> points;
    points.reserve(particles.size());
    for (unsigned int i = 0; i < particles.size(); i++)
        points.push_back(particles[i].position);

    //std::cout << "dt 2" << dt << std::endl;
    _waterBrickGrid.update2(_liquidSDF, _solidSDF, particles, dt);
}

void FluidSimulation::_writeBrickMaterialToFile(std::string brickfile, std::string colorfile)
{
    TriangleMesh brickmesh;
    _waterBrickGrid.getBrickMesh(brickmesh);
    brickmesh.translate(_domainOffset);
    _writeBrickTriangleMeshToFile(brickmesh, brickfile);
    _writeBrickColorListToFile(brickmesh, colorfile);
}

void FluidSimulation::_writeBrickTriangleMeshToFile(TriangleMesh &mesh, std::string filename)
{
    mesh.writeMeshToOBJ(filename);
}

void FluidSimulation::_writeBrickColorListToFile(TriangleMesh &mesh, std::string filename)
{
    int binsize = sizeof(unsigned char) * (int)mesh.vertexcolors.size();
    char *storage = new char[binsize];
    VectorMath::vec3 c;
    for (unsigned int i = 0; i < mesh.vertexcolors.size(); i++)
    {
        c = mesh.vertexcolors[i];
        storage[i] = (unsigned char)(c.x * 255.0);
    }

    std::ofstream erasefile;
    erasefile.open(filename, std::ofstream::out | std::ofstream::trunc);
    erasefile.close();

    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    file.write(storage, binsize);
    file.close();
    unsigned char p = storage[0];
    delete[] storage;
}

TriangleMesh FluidSimulation::_getTriangleMeshFromAABB(AABB bbox)
{
    VectorMath::vec3 p = bbox.position;
    std::vector<VectorMath::vec3> verts{
        VectorMath::vec3(p.x, p.y, p.z),
        VectorMath::vec3(p.x + (float)bbox.width, p.y, p.z),
        VectorMath::vec3(p.x + (float)bbox.width, p.y, p.z + (float)bbox.depth),
        VectorMath::vec3(p.x, p.y, p.z + (float)bbox.depth),
        VectorMath::vec3(p.x, p.y + (float)bbox.height, p.z),
        VectorMath::vec3(p.x + (float)bbox.width, p.y + (float)bbox.height, p.z),
        VectorMath::vec3(p.x + (float)bbox.width, p.y + (float)bbox.height, p.z + (float)bbox.depth),
        VectorMath::vec3(p.x, p.y + (float)bbox.height, p.z + (float)bbox.depth)};

    std::vector<Triangle> tris{
        Triangle(0, 1, 2), Triangle(0, 2, 3), Triangle(4, 7, 6), Triangle(4, 6, 5),
        Triangle(0, 3, 7), Triangle(0, 7, 4), Triangle(1, 5, 6), Triangle(1, 6, 2),
        Triangle(0, 4, 5), Triangle(0, 5, 1), Triangle(3, 2, 6), Triangle(3, 6, 7)};

    TriangleMesh m;
    m.vertices = verts;
    m.triangles = tris;

    return m;
}

TriangleMesh FluidSimulation::_getBoundaryTriangleMesh()
{
    double eps = 1e-6;
    AABB domainAABB(0.0, 0.0, 0.0, _isize * _dx, _jsize * _dx, _ksize * _dx);
    domainAABB.expand(-3 * _dx - eps);

    TriangleMesh domainMesh = _getTriangleMeshFromAABB(domainAABB);
    return domainMesh;
}

void FluidSimulation::_initializeBoundary()
{
    TriangleMesh boundaryMesh = _getBoundaryTriangleMesh();
    _solidSDF = MeshLevelSet(_isize, _jsize, _ksize, _dx);
    _solidSDF.calculateSignedDistanceField(boundaryMesh, _meshLevelSetExactBand);
    _solidSDF.negate();
}

float FluidSimulation::_cfl()
{

    float maxvel = 0;
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize + 1; i++)
            {
                maxvel = fmax(maxvel, fabs(_MACVelocity.U(i, j, k)));
            }
        }
    }

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize + 1; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                maxvel = fmax(maxvel, fabs(_MACVelocity.V(i, j, k)));
            }
        }
    }

    for (int k = 0; k < _ksize + 1; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                maxvel = fmax(maxvel, fabs(_MACVelocity.W(i, j, k)));
            }
        }
    }

    return (float)((_CFLConditionNumber * _dx) / maxvel);
}

void FluidSimulation::_addBodyForce(float dt)
{
    Array3D<bool> fgrid(_isize, _jsize, _ksize, false);
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                if (_liquidSDF(i, j, k) < 0.0)
                {
                    fgrid.set(i, j, k, true);
                }
            }
        }
    }

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize + 1; i++)
            {
                if (Grid3D::isUFaceBorderingValue(i, j, k, true, fgrid))
                {
                    _MACVelocity.addU(i, j, k, _gravity.x * dt);
                }
            }
        }
    }

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize + 1; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                if (Grid3D::isVFaceBorderingValue(i, j, k, true, fgrid))
                {
                    _MACVelocity.addV(i, j, k, _gravity.y * dt);
                }
            }
        }
    }

    for (int k = 0; k < _ksize + 1; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                if (Grid3D::isWFaceBorderingValue(i, j, k, true, fgrid))
                {
                    _MACVelocity.addW(i, j, k, _gravity.z * dt);
                }
            }
        }
    }
}

void FluidSimulation::_advectFluidParticles(float dt)
{
    _updateFluidParticleVelocities();

    AABB boundary(0.0, 0.0, 0.0, _isize * _dx, _jsize * _dx, _ksize * _dx);
    boundary.expand(-2 * _dx - 1e-4);
    for (unsigned int p = 0; p < particles.size(); p++)
    {
        particles[p].position = _traceRK4(particles[p].position, dt);

        //check boundaries and project exterior particles back in
        float phi_val = _solidSDF.trilinearInterpolate(particles[p].position);
        if (phi_val < 0)
        {
            VectorMath::vec3 grad = _solidSDF.trilinearInterpolateGradient(particles[p].position);
            if (VectorMath::lengthsq(grad) > 0)
                grad = VectorMath::normalize(grad);
            particles[p].position -= phi_val * grad;
        }

        if (!boundary.isPointInside(particles[p].position))
        {
            particles[p].position = boundary.getNearestPointInsideAABB(particles[p].position);
        }
    }
}

void FluidSimulation::_updateFluidParticleVelocities()
{

    for (size_t i = 0; i < particles.size(); i++)
    {
        VectorMath::vec3 pos = particles[i].position;
        VectorMath::vec3 v_new = _MACVelocity.evaluateVelocityAtPositionLinear(pos);
        VectorMath::vec3 v_old = _savedVelocityField.evaluateVelocityAtPositionLinear(pos);

        VectorMath::vec3 velocityPIC = v_new;
        VectorMath::vec3 velocityFLIP = particles[i].velocity + v_new - v_old;
        particles[i].velocity = (1.0f - _FLIP_percentage) * velocityPIC + _FLIP_percentage * velocityFLIP;
    }
}

void FluidSimulation::_updateLiquidSDF()
{
    std::vector<VectorMath::vec3> points;
    points.reserve(particles.size());
    for (size_t i = 0; i < particles.size(); i++)
    {
        points.push_back(particles[i].position);
    }

    _liquidSDF.calculateSignedDistanceField(points, _particleRadius, _solidSDF);
}

void FluidSimulation::_computeVelocityScalarField(Array3D<float> &field,
                                                  Array3D<bool> &isValueSet,
                                                  int dir)
{

    Array3D<float> weights(field.width, field.height, field.depth, 0.0);

    if (!_isUsingGPU)
    {
        int U = 0;
        int V = 1;
        int W = 2;

        VectorMath::vec3 offset;
        float hdx = (float)(0.5 * _dx);
        if (dir == U)
            offset = VectorMath::vec3(0.0f, hdx, hdx);
        else if (dir == V)
            offset = VectorMath::vec3(hdx, 0.0f, hdx);
        else if (dir == W)
            offset = VectorMath::vec3(hdx, hdx, 0.0f);
        else
            return;

        if (_wyvillkernelUsedIfNotUsingGPU)
        {
            // coefficients for Wyvill kernel
            float r = _dx;
            float rsq = r * r;
            float coef1 = (4.0f / 9.0f) * (1.0f / (r * r * r * r * r * r));
            float coef2 = (17.0f / 9.0f) * (1.0f / (r * r * r * r));
            float coef3 = (22.0f / 9.0f) * (1.0f / (r * r));

            Array3D<float> field2 = field;
            Array3D<bool> isValueSet2 = isValueSet;
            Array3D<float> weights2 = weights;

            for (size_t pidx = 0; pidx < particles.size(); pidx++)
            {
                VectorMath::vec3 p = particles[pidx].position - offset;
                float velocityComponent = particles[pidx].velocity[dir];

                GridIndex g = Grid3D::positionToGridIndex(p, _dx);
                GridIndex gmin((int)fmax(g.i - 1, 0),
                               (int)fmax(g.j - 1, 0),
                               (int)fmax(g.k - 1, 0));
                GridIndex gmax((int)fmin(g.i + 1, field.width - 1),
                               (int)fmin(g.j + 1, field.height - 1),
                               (int)fmin(g.k + 1, field.depth - 1));

                for (int k = gmin.k; k <= gmax.k; k++)
                {
                    for (int j = gmin.j; j <= gmax.j; j++)
                    {
                        for (int i = gmin.i; i <= gmax.i; i++)
                        {
                            VectorMath::vec3 gpos = Grid3D::GridIndexToPosition(i, j, k, _dx);
                            VectorMath::vec3 v = gpos - p;
                            float distsq = VectorMath::dot(v, v);
                            if (distsq < rsq)
                            {
                                float weight = 1.0f - coef1 * distsq * distsq * distsq + coef2 * distsq * distsq - coef3 * distsq;
                                field.add(i, j, k, weight * velocityComponent);
                                weights.add(i, j, k, weight);
                            }
                        }
                    }
                }
            }
        }

        // use metaballs kernel
        else
        {
            // coefficients for Wyvill kernel
            float r = _dx;
            float rsq = r * r;
            Array3D<float> field2 = field;
            Array3D<bool> isValueSet2 = isValueSet;
            Array3D<float> weights2 = weights;

            for (size_t pidx = 0; pidx < particles.size(); pidx++)
            {
                VectorMath::vec3 p = particles[pidx].position - offset;
                float velocityComponent = particles[pidx].velocity[dir];

                GridIndex g = Grid3D::positionToGridIndex(p, _dx);
                GridIndex gmin((int)fmax(g.i - 1, 0),
                               (int)fmax(g.j - 1, 0),
                               (int)fmax(g.k - 1, 0));
                GridIndex gmax((int)fmin(g.i + 1, field.width - 1),
                               (int)fmin(g.j + 1, field.height - 1),
                               (int)fmin(g.k + 1, field.depth - 1));

                for (int k = gmin.k; k <= gmax.k; k++)
                {
                    for (int j = gmin.j; j <= gmax.j; j++)
                    {
                        for (int i = gmin.i; i <= gmax.i; i++)
                        {
                            VectorMath::vec3 gpos = Grid3D::GridIndexToPosition(i, j, k, _dx);
                            VectorMath::vec3 v = gpos - p;
                            float distsq = VectorMath::dot(v, v);
                            if (distsq < rsq)
                            {
                                //float weight = 1.0f - coef1 * distsq * distsq * distsq + coef2 * distsq * distsq - coef3 * distsq;

                                double weight;
                                double radiusQuotient = sqrt(distsq) / r;

                                if (0.0 < radiusQuotient && radiusQuotient < (1 / 3.0f))
                                {
                                    weight = 1 - 3 * r * r;
                                }

                                else if ((1 / 3.0f) < radiusQuotient && radiusQuotient <= 1.0)
                                {
                                    weight = (3 / 2.0f) * pow((1.0f - r), 2);
                                }

                                else
                                {
                                    weight = 0.0f;
                                }

                                field.add(i, j, k, weight * velocityComponent);
                                weights.add(i, j, k, weight);
                            }
                        }
                    }
                }
            }
        }
    }

    // GPU parallelsation with wyvill kernel used
    else
    {
        computeVelocityFieldAndWeightsWithGPU(field, isValueSet, dir, *this, weights);
    }

    double eps = 1e-9;
    for (int k = 0; k < field.depth; k++)
    {
        for (int j = 0; j < field.height; j++)
        {
            for (int i = 0; i < field.width; i++)
            {
                float value = field(i, j, k);
                float weight = weights(i, j, k);

                if (weight < eps)
                {
                    continue;
                }
                field.set(i, j, k, value / weight);
                isValueSet.set(i, j, k, true);
            }
        }
    }
}

void FluidSimulation::_advectVelocityFieldU(Array3D<bool> &fluidCellGrid)
{
    Array3D<float> ugrid = Array3D<float>(_isize + 1, _jsize, _ksize, 0.0f);
    Array3D<bool> isValueSet = Array3D<bool>(_isize + 1, _jsize, _ksize, false);
    _computeVelocityScalarField(ugrid, isValueSet, 0);

    _MACVelocity.clearU();
    for (int k = 0; k < ugrid.depth; k++)
    {
        for (int j = 0; j < ugrid.height; j++)
        {
            for (int i = 0; i < ugrid.width; i++)
            {
                if (Grid3D::isUFaceBorderingValue(i, j, k, true, fluidCellGrid))
                {
                    if (isValueSet(i, j, k))
                    {
                        _MACVelocity.setU(i, j, k, ugrid(i, j, k));
                        _validVelocities.validU.set(i, j, k, true);
                    }
                }
            }
        }
    }
}

void FluidSimulation::getDimensions(int *i, int *j, int *k)
{
    *i = _isize;
    *j = _jsize;
    *k = _ksize;
}

void FluidSimulation::getDimensions(int &i, int &j, int &k)
{
    i = _isize;
    j = _jsize;
    k = _ksize;
}

float FluidSimulation::getdx()
{
    return _dx;
};

bool FluidSimulation::getIsUsingGPU()
{
    return _isUsingGPU;
};

void FluidSimulation::setIsUsingGPU(bool isUsingGPU)
{
    _isUsingGPU = isUsingGPU;
}

void FluidSimulation::_advectVelocityFieldV(Array3D<bool> &fluidCellGrid)
{
    Array3D<float> vgrid = Array3D<float>(_isize, _jsize + 1, _ksize, 0.0f);
    Array3D<bool> isValueSet = Array3D<bool>(_isize, _jsize + 1, _ksize, false);
    _computeVelocityScalarField(vgrid, isValueSet, 1);

    _MACVelocity.clearV();
    for (int k = 0; k < vgrid.depth; k++)
    {
        for (int j = 0; j < vgrid.height; j++)
        {
            for (int i = 0; i < vgrid.width; i++)
            {
                if (Grid3D::isVFaceBorderingValue(i, j, k, true, fluidCellGrid))
                {
                    if (isValueSet(i, j, k))
                    {
                        _MACVelocity.setV(i, j, k, vgrid(i, j, k));
                        _validVelocities.validV.set(i, j, k, true);
                    }
                }
            }
        }
    }
}

MACVelocityField &FluidSimulation::getMACVelocityField()
{
    return _MACVelocity;
}

void FluidSimulation::_advectVelocityFieldW(Array3D<bool> &fluidCellGrid)
{
    Array3D<float> wgrid = Array3D<float>(_isize, _jsize, _ksize + 1, 0.0f);
    Array3D<bool> isValueSet = Array3D<bool>(_isize, _jsize, _ksize + 1, 0.0f);
    _computeVelocityScalarField(wgrid, isValueSet, 2);

    _MACVelocity.clearW();
    for (int k = 0; k < wgrid.depth; k++)
    {
        for (int j = 0; j < wgrid.height; j++)
        {
            for (int i = 0; i < wgrid.width; i++)
            {
                if (Grid3D::isWFaceBorderingValue(i, j, k, true, fluidCellGrid))
                {
                    if (isValueSet(i, j, k))
                    {
                        _MACVelocity.setW(i, j, k, wgrid(i, j, k));
                        _validVelocities.validW.set(i, j, k, true);
                    }
                }
            }
        }
    }
}

void FluidSimulation::_advectVelocityField()
{
    Array3D<bool> fluidCellGrid(_isize, _jsize, _ksize, false);
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                if (_liquidSDF(i, j, k) < 0.0)
                {
                    fluidCellGrid.set(i, j, k, true);
                }
            }
        }
    }

    _validVelocities.reinitialize();

    _advectVelocityFieldU(fluidCellGrid);
    _advectVelocityFieldV(fluidCellGrid);
    _advectVelocityFieldW(fluidCellGrid);

    _extrapolateVelocityField(_MACVelocity, _validVelocities);
    _savedVelocityField = _MACVelocity;
}

void FluidSimulation::_project(float dt)
{
    // Compute finite-volume type face area weight for each velocity sample.
    _computeWeights();

    // Set up and solve the variational _pressure solve.
    Array3D<float> pressureGrid = _solvePressure(dt);
    _applyPressure(dt, pressureGrid);

    _extrapolateVelocityField(_MACVelocity, _validVelocities);
}

// Apply RK2 to advect a point in the domain.
VectorMath::vec3 FluidSimulation::_traceRK2(VectorMath::vec3 position, float dt)
{
    VectorMath::vec3 input = position;
    VectorMath::vec3 velocity = _getVelocity(input);
    velocity = _getVelocity(input + 0.5f * dt * velocity);
    input += dt * velocity;
    return input;
}

// Apply RK2 to advect a point in the domain.
VectorMath::vec3 FluidSimulation::FluidSimulation::_traceRK4(VectorMath::vec3 position, float dt)
{
    VectorMath::vec3 x0 = position;
    VectorMath::vec3 k1 = dt * _getVelocity(x0);
    VectorMath::vec3 k2 = dt * _getVelocity(x0 + k1 / 2);
    VectorMath::vec3 k3 = dt * _getVelocity(x0 + k2 / 2);
    VectorMath::vec3 k4 = dt * _getVelocity(x0 + k3);

    VectorMath::vec3 newpos = x0 + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
    return newpos;
}

//Interpolate velocity from the MAC grid.
VectorMath::vec3 FluidSimulation::_getVelocity(VectorMath::vec3 position)
{
    return _MACVelocity.evaluateVelocityAtPositionLinear(position);
}

//Compute finite-volume style face-weights for fluid from nodal signed distances
void FluidSimulation::_computeWeights()
{

    //Compute face area fractions (using marching squares cases).
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize + 1; i++)
            {
                float weight = 1.0f - _solidSDF.getFaceWeightU(i, j, k);
                weight = _clamp(weight, 0.0f, 1.0f);
                _weightGrid.U.set(i, j, k, weight);
            }
        }
    }

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize + 1; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                float weight = 1.0f - _solidSDF.getFaceWeightV(i, j, k);
                weight = _clamp(weight, 0.0f, 1.0f);
                _weightGrid.V.set(i, j, k, weight);
            }
        }
    }

    for (int k = 0; k < _ksize + 1; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                float weight = 1.0f - _solidSDF.getFaceWeightW(i, j, k);
                weight = _clamp(weight, 0.0f, 1.0f);
                _weightGrid.W.set(i, j, k, weight);
            }
        }
    }
}

//An implementation of the variational _pressure projection solve for static geometry
Array3D<float> FluidSimulation::_solvePressure(float dt)
{
    PressureSolverParameters params;
    params.cellwidth = _dx;
    params.density = 1.0;
    params.deltaTime = dt;
    params.velocityField = &_MACVelocity;
    params.liquidSDF = &_liquidSDF;
    params.weightGrid = &_weightGrid;

    PressureSolver solver;
    return solver.solve(params);
}

void FluidSimulation::_applyPressure(float dt, Array3D<float> &pressureGrid)
{
    Array3D<bool> fgrid(_isize, _jsize, _ksize, false);
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                if (_liquidSDF(i, j, k) < 0.0)
                {
                    fgrid.set(i, j, k, true);
                }
            }
        }
    }

    _validVelocities.reinitialize();
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 1; i < _isize; i++)
            {

                if (_weightGrid.U(i, j, k) > 0 && Grid3D::isUFaceBorderingValue(i, j, k, true, fgrid))
                {
                    float p0 = pressureGrid(i - 1, j, k);
                    float p1 = pressureGrid(i, j, k);
                    float theta = fmax(_liquidSDF.getFaceWeightU(i, j, k), _minfrac);
                    _MACVelocity.addU(i, j, k, -dt * (p1 - p0) / (_dx * theta));
                    _validVelocities.validU.set(i, j, k, true);
                }
            }
        }
    }

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 1; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {

                if (_weightGrid.V(i, j, k) > 0 && Grid3D::isVFaceBorderingValue(i, j, k, true, fgrid))
                {
                    float p0 = pressureGrid(i, j - 1, k);
                    float p1 = pressureGrid(i, j, k);
                    float theta = fmax(_liquidSDF.getFaceWeightV(i, j, k), _minfrac);
                    _MACVelocity.addV(i, j, k, -dt * (p1 - p0) / (_dx * theta));
                    _validVelocities.validV.set(i, j, k, true);
                }
            }
        }
    }

    for (int k = 1; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {

                if (_weightGrid.W(i, j, k) > 0 && Grid3D::isWFaceBorderingValue(i, j, k, true, fgrid))
                {
                    float p0 = pressureGrid(i, j, k - 1);
                    float p1 = pressureGrid(i, j, k);
                    float theta = fmax(_liquidSDF.getFaceWeightW(i, j, k), _minfrac);
                    _MACVelocity.addW(i, j, k, -dt * (p1 - p0) / (_dx * theta));
                    _validVelocities.validW.set(i, j, k, true);
                }
            }
        }
    }

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize + 1; i++)
            {
                if (!_validVelocities.validU(i, j, k))
                {
                    _MACVelocity.setU(i, j, k, 0.0);
                }
            }
        }
    }

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize + 1; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                if (!_validVelocities.validV(i, j, k))
                {
                    _MACVelocity.setV(i, j, k, 0.0);
                }
            }
        }
    }

    for (int k = 0; k < _ksize + 1; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                if (!_validVelocities.validW(i, j, k))
                {
                    _MACVelocity.setW(i, j, k, 0.0);
                }
            }
        }
    }
}

void FluidSimulation::_extrapolateVelocityField(MACVelocityField &vfield,
                                                ValidVelocityGrid &valid)
{
    int numLayers = (int)ceil(_CFLConditionNumber) + 2;
    vfield.extrapolateVelocityField(valid, numLayers);
}

void FluidSimulation::_constrainVelocityField()
{
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize + 1; i++)
            {
                if (_weightGrid.U(i, j, k) == 0)
                {
                    _MACVelocity.setU(i, j, k, 0.0);
                    _savedVelocityField.setU(i, j, k, 0.0);
                }
            }
        }
    }

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize + 1; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                if (_weightGrid.V(i, j, k) == 0)
                {
                    _MACVelocity.setV(i, j, k, 0.0);
                    _savedVelocityField.setV(i, j, k, 0.0);
                }
            }
        }
    }

    for (int k = 0; k < _ksize + 1; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                if (_weightGrid.W(i, j, k) == 0)
                {
                    _MACVelocity.setW(i, j, k, 0.0);
                    _savedVelocityField.setW(i, j, k, 0.0);
                }
            }
        }
    }
}
