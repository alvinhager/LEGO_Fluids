#include "WaterBrickGrid.h"

/** empty constructor */
WaterBrickGrid::WaterBrickGrid()
{
}

/** initialises fluid brick with (isize,jsize,ksize) size, dx size of cell, and brick size as well as density grid of (isize,jsize,ksize) dim */
WaterBrickGrid::WaterBrickGrid(int isize, int jsize, int ksize, double dx, AABB brick) : _isize(isize), _jsize(jsize), _ksize(ksize), _dx(dx), _brick(brick), _densityGrid(isize, jsize, ksize), _densityGrid2(isize, jsize, ksize), _velocityGrid(isize, jsize, ksize)
{
    _initializeBrickGrid();
}

/** Destructor */
WaterBrickGrid::~WaterBrickGrid()
{
}

/** returns dimensions of the grid (not the actual brick grid) */
void WaterBrickGrid::getGridDim(int *i, int *j, int *k)
{
    *i = _isize;
    *j = _jsize;
    *k = _ksize;
}

/** returns the cell size of the grid (not brick grid) */
double WaterBrickGrid::getCellSize()
{
    return _dx;
}

/** returns the dimensions of the brick grid */
void WaterBrickGrid::getBrickGridDim(int *i, int *j, int *k)
{
    *i = _brickGrid.width;
    *j = _brickGrid.height;
    *k = _brickGrid.depth;
}

/** returns the dimensions of the brick in the brick grid in the form of AABB */
AABB WaterBrickGrid::getBrickAABB()
{
    return _brick;
}

/** sets the bricks dimensions to (width,height,depth) */
void WaterBrickGrid::setBrickDim(double width, double height, double depth)
{
    setBrickDim(AABB(VectorMath::vec3(), width, height, depth));
}

/** sets the brick dimensins to the dimensions of given AABB */
void WaterBrickGrid::setBrickDim(AABB brick)
{
    CUSTOM_ASSERT(isBrickGridInitialized());

    // If brick dimensions are significantly different from previous dimensions -> reset the brick grid and then initialize it from scratch using the dimensions */
    if (fabs(brick.width - _brick.width) > Constants::EPSILON_SMALL || fabs(brick.height - _brick.height) > Constants::EPSILON_SMALL || fabs(brick.depth - _brick.depth) > Constants::EPSILON_SMALL)
    {
        _reset();
    }

    _brick = brick;
    _initializeBrickGrid();
}

/** sets currentbrickready = false, brick queue size = 0 and numupdates = 0 */
void WaterBrickGrid::_reset()
{
    _brickGridQueueSize = 0;
    _isCurrentBrickGridReady = false;
    _numUpdates = 0;
}

/** updates the brick grid */
void WaterBrickGrid::_updateBrickGrid(ParticleLevelSet &liquidSDF, MeshLevelSet solidSDF)
{
    _brickGrid.fill(Brick());

    double bw = _brick.width;
    double bh = _brick.height;
    double bd = _brick.depth;

    VectorMath::vec3 coffset = VectorMath::vec3(0.5 * bw, 0.5 * bh, 0.5 * bd);

    VectorMath::vec3 p;
    GridIndex g;
    for (int k = 0; k < _brickGrid.depth; k++)
    {
        for (int j = 0; j < _brickGrid.height; j++)
        {
            for (int i = 0; i < _brickGrid.width; i++)
            {
                p = coffset + VectorMath::vec3(i * bw, j * bh, k * bd);
                g = Grid3D::positionToGridIndex(p, _dx);

                if (Grid3D::isPositionInGrid(p, _dx, _isize, _jsize, _ksize) && liquidSDF(g) < 0.0 && solidSDF(g) > 0.0)
                {
                    //std::cout << "WaterBrickGrid::_updateBrickGrid()->Active brick found!" << std::endl;
                    float intensity = _getBrickIntensity(i, j, k);
                    //std::cout << "intensity=" << intensity << std::endl;
                    Brick b = Brick(intensity);
                    b.isActive = true;
                    _brickGrid.set(i, j, k, b);
                }
            }
        }
    }

    Brick *b;
    GridIndex n;
    for (int k = 0; k < _brickGrid.depth; k++)
    {
        for (int j = 0; j < _brickGrid.height; j++)
        {
            for (int i = 0; i < _brickGrid.width; i++)
            {
                if (_brickGrid(i, j, k).isActive && !_isBrickNextToActiveNeighbour(i, j, k))
                {
                    b = _brickGrid.getPointer(i, j, k);
                    b->isActive = false;
                }
            }
        }
    }
}

/** returns whether the brick has an immediate active neighbor */
bool WaterBrickGrid::_isBrickNextToActiveNeighbour(int i, int j, int k)
{
    GridIndex nbs[6];
    GridIndex n;
    Grid3D::getNeighborGridIndices6(i, j, k, nbs);
    bool hasNeighbour = false;
    for (int idx = 0; idx < 6; idx++)
    {
        n = nbs[idx];
        if (Grid3D::isGridIndexInRange(n, _brickGrid.width, _brickGrid.height, _brickGrid.depth) && _brickGrid(n).isActive)
        {
            hasNeighbour = true;
            break;
        }
    }

    return hasNeighbour;
}

/** returns the intensity for the brick at position (i,j,k)
 * does so by figuring out the pmin and pmax of the brick, converting it to grid index bounds.
 * This is then used to calculate the average intensity for the grid indices that contain a part of this brick
 * and we return this as the value calculated for the given brick.
 */
float WaterBrickGrid::_getBrickIntensity(int i, int j, int k)
{
    VectorMath::vec3 pmin = VectorMath::vec3(i * _brick.width, j * _brick.height, k * _brick.depth);
    VectorMath::vec3 pmax = pmin + VectorMath::vec3(_brick.width, _brick.height, _brick.depth);
    AABB bbox = AABB(pmin, pmax);
    GridIndex gmin, gmax;
    Grid3D::getGridIndexBounds(bbox, _dx, _isize, _jsize, _ksize, &gmin, &gmax);

    double avg = 0.0;
    double count = 0.0;
    double eps = 0.001;
    for (int k = gmin.k; k <= gmax.k; k++)
    {
        for (int j = gmin.j; j <= gmax.j; j++)
        {
            for (int i = gmin.i; i <= gmax.i; i++)
            {
                double val = (double)_densityGrid(i, j, k).currentDensity;
                if (val > eps)
                {
                    avg += val;
                    count++;
                }
            }
        }
    }

    if (count == 0.0)
    {
        return 0.0;
    }

    return (float)(avg /= count);
}

/** updates the density grid with particles and dt by calling updateTargetDensities and updateDensities */
void WaterBrickGrid::_updateDensityGrid(std::vector<VectorMath::vec3> &particles, double dt)
{
    _updateTargetDensities(particles);
    _updateDensities(dt);
}

void WaterBrickGrid::_updateTargetDensities(std::vector<VectorMath::vec3> &points)
{
    Array3D<int> _countGrid = Array3D<int>(_isize, _jsize, _ksize, 0);
    GridIndex g;
    for (unsigned int i = 0; i < points.size(); i++)
    {
        g = Grid3D::positionToGridIndex(points[i], _dx);
        if (Grid3D::isGridIndexInRange(g, _isize, _jsize, _ksize))
        {
            //std::cout << "density in updateTargetDensities " << _countGrid(g) << std::endl;
            _countGrid.add(g, 1);
        }
    }

    double min = (double)_minParticleDensity;
    double max = (double)_maxParticleDensity;
    DensityInfo *node;
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                double density = ((double)_countGrid(i, j, k) - min) / (max - min);
                density = fmax(0.0, density);
                density = fmin(1.0, density);

                node = _densityGrid.getPointer(i, j, k);
                node->targetDensity = (float)density;

                if (_numUpdates == 0)
                {
                    node->currentDensity = (float)density;
                }
            }
        }
    }
}

/** updates the brick grid by setting the intensity of each brick if it is not in solid and then removes lone fluid cells */
void WaterBrickGrid::_updateBrickGrid2(ParticleLevelSet &liquidSDF, MeshLevelSet solidSDF)
{
    _brickGrid.fill(Brick());

    double bw = _brick.width;
    double bh = _brick.height;
    double bd = _brick.depth;

    VectorMath::vec3 coffset = VectorMath::vec3(0.5 * bw, 0.5 * bh, 0.5 * bd);

    VectorMath::vec3 p;
    GridIndex g;
    for (int k = 0; k < _brickGrid.depth; k++)
    {
        for (int j = 0; j < _brickGrid.height; j++)
        {
            for (int i = 0; i < _brickGrid.width; i++)
            {
                p = coffset + VectorMath::vec3(i * bw, j * bh, k * bd);
                g = Grid3D::positionToGridIndex(p, _dx);

                if (Grid3D::isPositionInGrid(p, _dx, _isize, _jsize, _ksize) && liquidSDF(g) < 0.0 && solidSDF(g) > 0.0)
                {
                    //std::cout << "WaterBrickGrid::_updateBrickGrid()->Active brick found!" << std::endl;
                    float intensity = _getBrickIntensity2(i, j, k);

                    if (intensity != 0)
                    {
                        //std::cout << "intensity (average density) = " << intensity << std::endl;
                    }

                    Brick b = Brick(intensity);
                    b.isActive = true;
                    _brickGrid.set(i, j, k, b);
                }
            }
        }
    }

    Brick *b;
    GridIndex n;
    for (int k = 0; k < _brickGrid.depth; k++)
    {
        for (int j = 0; j < _brickGrid.height; j++)
        {
            for (int i = 0; i < _brickGrid.width; i++)
            {
                if (_brickGrid(i, j, k).isActive && !_isBrickNextToActiveNeighbour(i, j, k))
                {
                    b = _brickGrid.getPointer(i, j, k);
                    b->isActive = false;
                }
            }
        }
    }
}

/** gets the brick intensity at (i,j,k) for each brick by averaging the density across the boundaries of the brick 
brick intensity is right now just the average density */
float WaterBrickGrid::_getBrickIntensity2(int i, int j, int k)
{
    VectorMath::vec3 pmin = VectorMath::vec3(i * _brick.width, j * _brick.height, k * _brick.depth);
    VectorMath::vec3 pmax = pmin + VectorMath::vec3(_brick.width, _brick.height, _brick.depth);
    AABB bbox = AABB(pmin, pmax);
    GridIndex gmin, gmax;
    Grid3D::getGridIndexBounds(bbox, _dx, _isize, _jsize, _ksize, &gmin, &gmax);

    double avg = 0.0;
    double count = 0.0;
    double eps = 0.001;
    for (int k = gmin.k; k <= gmax.k; k++)
    {
        for (int j = gmin.j; j <= gmax.j; j++)
        {
            for (int i = gmin.i; i <= gmax.i; i++)
            {
                double val = (double)_densityGrid2(i, j, k).currentDensity;
                if (val > eps)
                {
                    avg += val;
                    count++;
                }
            }
        }
    }

    if (count == 0.0)
    {
        return 0.0;
    }

    return (float)(avg /= count);
}

float WaterBrickGrid::_getBrickIntensityAcceleration(int i, int j, int k, double dt)
{
    VectorMath::vec3 pmin = VectorMath::vec3(i * _brick.width, j * _brick.height, k * _brick.depth);
    VectorMath::vec3 pmax = pmin + VectorMath::vec3(_brick.width, _brick.height, _brick.depth);
    AABB bbox = AABB(pmin, pmax);
    GridIndex gmin, gmax;
    Grid3D::getGridIndexBounds(bbox, _dx, _isize, _jsize, _ksize, &gmin, &gmax);

    double avg = 0.0;
    double count = 0.0;
    double eps = 0.001;
    for (int k = gmin.k; k <= gmax.k; k++)
    {
        for (int j = gmin.j; j <= gmax.j; j++)
        {
            for (int i = gmin.i; i <= gmax.i; i++)
            {
                VelocityInfo *node = _velocityGrid.getPointer(i, j, k);
                VectorMath::vec3 accelerationVector = (node->currentVelocity - node->previousVelocity);

                //std::cout << "dt " << dt << std::endl;
                //std::cout << "accelerationVector "
                // << " " << accelerationVector.x << " " << accelerationVector.y << " " << accelerationVector.z << std::endl;
                double val = sqrt(accelerationVector.x * accelerationVector.x + accelerationVector.y * accelerationVector.y + accelerationVector.z * accelerationVector.z);

                if (val > eps)
                {
                    avg += val;
                    count++;
                }
            }
        }
    }

    if (count == 0.0)
    {
        return 0.0;
    }

    return (float)(avg /= count);
}

/** updates the current density and prevDensity. If it is the first frame, prevDensity is set to -1 */
void WaterBrickGrid::_updateDensityGrid2(std::vector<VectorMath::vec3> &points)
{

    // count the number of particles in each grid cell
    Array3D<int> countGrid = Array3D<int>(_isize, _jsize, _ksize, 0);
    GridIndex g;
    for (unsigned int i = 0; i < points.size(); i++)
    {
        g = Grid3D::positionToGridIndex(points[i], _dx);
        if (Grid3D::isGridIndexInRange(g, _isize, _jsize, _ksize))
        {
            countGrid.add(g, 1);
        }
    }

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {

                DensityInfo2 *node = _densityGrid2.getPointer(i, j, k);

                float density = (float)countGrid(i, j, k);

                if (density > 0)
                    //std::cout << "density count " << density << std::endl;

                    if (_numUpdates == 0)
                    {
                        node->currentDensity = (float)density;
                        node->prevDensity = -1;
                    }

                    else
                    {
                        node->prevDensity = node->currentDensity;
                        node->currentDensity = (float)density;
                    }
            }
        }
    }
}

/** updates the currenet density of a given node given the amount of time dt that has passed with the use of target density */
void WaterBrickGrid::_updateDensity(int i, int j, int k, double dt)
{
    DensityInfo *node = _densityGrid.getPointer(i, j, k);

    float target = node->targetDensity;
    float current = node->currentDensity;
    float desired = target - current;
    if (fabs(desired) < _decelerationRadius)
    {
        float r = fabs(desired) / _decelerationRadius;
        float mag = r * _maxVelocityIntensity;
        //if the change in density is positive -> set desired to positive mag, if it's negative set to negative mag
        desired = desired > 0.0f ? mag : -mag;
    }
    else
    // if desired = change in density is >= decelerationRadius simply set it to the max Intensity value, with a sign that is decided by increase or decrease in density
    {
        desired = desired > 0.0f ? _maxVelocityIntensity : -_maxVelocityIntensity;
    }

    // acceleration is equal to change from last velocityIntensity.
    float acc = desired - node->velocityIntensity;
    // if acceleration magnitude is greater than max intensity acceleration, set to maxIntensityAcceleration with sign decided by whether it is accelerating or decelerating
    // basically just a cap on acceleration
    if (fabs(acc) > _maxAccelerationIntensity)
    {
        acc = acc > 0.0f ? _maxAccelerationIntensity : -_maxAccelerationIntensity;
    }

    // calculates the new velocity by adding the acceleration * dt
    node->velocityIntensity += acc * (float)dt;
    // current density is changed by velocityIntensity* (float)dt and clamped between 0 and 1
    node->currentDensity += node->velocityIntensity * (float)dt;
    node->currentDensity = fmax(0.0f, node->currentDensity);
    node->currentDensity = fmin(1.0f, node->currentDensity);
}

/** updates the densities using _updateDensity() of every cell in brick grid */
void WaterBrickGrid::_updateDensities(double dt)
{
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                _updateDensity(i, j, k, dt);
            }
        }
    }
}

/** initializes the brick grid according to the dimensions of the brick field, NOTE brick dimensions each > 0 for this to work */
void WaterBrickGrid::_initializeBrickGrid()
{
    CUSTOM_ASSERT(_brick.width > 0.0);
    CUSTOM_ASSERT(_brick.height > 0.0);
    CUSTOM_ASSERT(_brick.depth > 0.0);

    double width = _isize * _dx;
    double height = _jsize * _dx;
    double depth = _ksize * _dx;
    int bi = (int)ceil(width / _brick.width);
    int bj = (int)ceil(height / _brick.height);
    int bk = (int)ceil(depth / _brick.depth);

    _brickGrid = Array3D<Brick>(bi, bj, bk);

    _isBrickGridInitialized = true;
}

/** returns the triangle mesh of the brick grid as a TriangleMesh if the current brick grid is ready 
 * does this by looping over the brick grid and returning the position of the center of each vertex, 
 * the color of the grid which is set to the color of each vertex.
*/
bool WaterBrickGrid::getBrickMesh(TriangleMesh &mesh)
{
    CUSTOM_ASSERT(isBrickGridInitialized());
    if (!_isCurrentBrickGridReady)
    {
        return false;
    }

    double bw = _brick.width;
    double bh = _brick.height;
    double bd = _brick.depth;
    VectorMath::vec3 coffset = VectorMath::vec3(0.5 * bw, 0.5 * bh, 0.5 * bd);

    VectorMath::vec3 p;
    for (int k = 0; k < _currentBrickGrid.depth; k++)
    {
        for (int j = 0; j < _currentBrickGrid.height; j++)
        {
            for (int i = 0; i < _currentBrickGrid.width; i++)
            {
                if (_currentBrickGrid(i, j, k).isActive)
                {
                    p = coffset + VectorMath::vec3(i * bw, j * bh, k * bd);
                    double intensity = _currentBrickGrid(i, j, k).intensity;
                    mesh.vertices.push_back(p);
                    mesh.vertexcolors.push_back(VectorMath::vec3(intensity, intensity, intensity));
                }
            }
        }
    }

    return true;
}

/** returns if the brick mesh is ready */
bool WaterBrickGrid::isBrickMeshReady()
{
    return _isCurrentBrickGridReady;
}

/** returns the size of the brick grid queue, i.e. number of bricks in queue. */
int WaterBrickGrid::getBrickGridQueueSize()
{
    return _brickGridQueueSize;
}

/** returns the number of updates that have been made since the start of sim. */
int WaterBrickGrid::getNumUpdates()
{
    return _numUpdates;
}

/** returns the current grid density values in a grid, which has to be the same size as the density grid stored in WaterBrickGrid 
 * if grid has been initialised otherwise throws error
*/
void WaterBrickGrid::getDensityGridCurrentDensityValues(Array3D<float> &grid)
{
    CUSTOM_ASSERT(isBrickGridInitialized());

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                float val = _densityGrid(i, j, k).currentDensity;
                grid.set(i, j, k, val);
            }
        }
    }
}

/** returns the target density grid values on the grid, same as previous but with target_density, 
 * if grid has been initialised otherwise throws error */
void WaterBrickGrid::getDensityGridTargetDensityValues(Array3D<float> &grid)
{
    CUSTOM_ASSERT(isBrickGridInitialized());

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                float val = _densityGrid(i, j, k).targetDensity;
                grid.set(i, j, k, val);
            }
        }
    }
}

/** returns the density grid velocity values,
 * if grid has been initialised otherwise throws error
 */
void WaterBrickGrid::getDensityGridVelocityValues(Array3D<float> &grid)
{
    CUSTOM_ASSERT(isBrickGridInitialized());

    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                float val = _densityGrid(i, j, k).velocityIntensity;
                grid.set(i, j, k, val);
            }
        }
    }
}

/** returns a pointer to the brick grid queue */
Array3D<Brick> *WaterBrickGrid::getPointerToBrickGridQueue()
{
    CUSTOM_ASSERT(isBrickGridInitialized());
    return _brickGridQueue;
}

/** returns whether the brick grid is initialized */
bool WaterBrickGrid::isBrickGridInitialized()
{
    return _isBrickGridInitialized;
}

/** Finds the interconnected bricks in brickGrid and returns these as a list of grid indexes in connectedBricks,
 *  newBricks seems to initially contain a bool grid that is then set to false if a given brick is added to the interconnected list
 * this is so that if it is called repeatedly we dont call it again on the same index  */
void WaterBrickGrid::_getConnectedBricks(int i, int j, int k,
                                         Array3D<Brick> &brickGrid,
                                         Array3D<bool> &newBricks,
                                         GridIndexVector &connectedBricks)
{
    int bw = _brickGrid.width;
    int bh = _brickGrid.height;
    int bd = _brickGrid.depth;

    Array3D<bool> isProcessed = Array3D<bool>(bw, bh, bd, false);

    GridIndexVector queue(bw, bh, bd);
    queue.push_back(i, j, k);
    newBricks.set(i, j, k, false);
    isProcessed.set(i, j, k, true);

    GridIndex nbs[6];
    GridIndex g, n;
    while (!queue.empty())
    {
        // returns a reference to element on the top of queue ( or last in list )
        g = queue.back();
        // pops this off the list so removes the last element
        queue.pop_back();

        Grid3D::getNeighborGridIndices6(g, nbs);
        for (int idx = 0; idx < 6; idx++)
        {
            n = nbs[idx];
            if (Grid3D::isGridIndexInRange(n, bw, bh, bd) && brickGrid(n).isActive &&
                !isProcessed(n))
            {
                queue.push_back(n);
                newBricks.set(n, false);
                isProcessed.set(n, true);
            }
        }

        connectedBricks.push_back(g);
    }
}

/** for every brick in newBricks<bool> that is true, add its connected bricks to connectedBricks, using the brickGrid to check if the bricks are active, 
 * newBricks is just used to store what parts of the brickGrid should be checked 
 */
void WaterBrickGrid::_getbrickStructures(Array3D<Brick> &brickGrid, Array3D<bool> &newBricks,
                                         std::vector<GridIndexVector> &brickStructures)
{
    for (int k = 0; k < newBricks.depth; k++)
    {
        for (int j = 0; j < newBricks.height; j++)
        {
            for (int i = 0; i < newBricks.width; i++)
            {
                if (newBricks(i, j, k))
                {
                    GridIndexVector connectedBricks(newBricks.width, newBricks.height, newBricks.depth);
                    _getConnectedBricks(i, j, k, brickGrid, newBricks, connectedBricks);
                    brickStructures.push_back(connectedBricks);
                }
            }
        }
    }
}

/** checks if brickGrid contain at least one active brick for the cells in the cells list, if so return true */
bool WaterBrickGrid::_isBrickMassInBrickGrid(GridIndexVector &cells,
                                             Array3D<Brick> &brickGrid)
{
    bool isInGrid = false;
    for (unsigned int i = 0; i < cells.size(); i++)
    {
        if (brickGrid(cells[i]).isActive)
        {
            isInGrid = true;
            break;
        }
    }

    return isInGrid;
}

/** sets all the bricks inside brickGrid in the given cells to inactive */
void WaterBrickGrid::_removeBrickStructureFromBrickGrid(GridIndexVector &cells,
                                                        Array3D<Brick> &brickGrid)
{
    Brick *b;
    for (unsigned int i = 0; i < cells.size(); i++)
    {
        b = brickGrid.getPointer(cells[i]);
        b->isActive = false;
    }
}

/** sets all the active brick cells in brickCurrent to inactive if brickNext is inactive at that cell for the area defined by brickStructures */
void WaterBrickGrid::_removeInvalidbrickStructures(Array3D<Brick> &brickCurrent,
                                                   Array3D<Brick> &brickNext,
                                                   std::vector<GridIndexVector> &brickStructures)
{
    for (unsigned int i = 0; i < brickStructures.size(); i++)
    {
        // brickNext contains no active brick cell at grid index defined by brickStructures...
        if (!_isBrickMassInBrickGrid(brickStructures[i], brickNext))
        {
            // then set the same brick cell inactive
            _removeBrickStructureFromBrickGrid(brickStructures[i], brickCurrent);
        }
    }
}

/** 
 * removes stray bricks by setting bricks to inactive in brickCurrent if brickNext brick is inactive, does this for the area defined by brickStructures 
 * brickStructures is the interconnected areas that are created from the new bricks in brickNext
 * 
*/
void WaterBrickGrid::_removeStrayBricks()
{
    Array3D<Brick> brickPrev = _brickGridQueue[0];
    Array3D<Brick> brickCurrent = _brickGridQueue[1];
    Array3D<Brick> brickNext = _brickGridQueue[2];

    int bw = _brickGrid.width;
    int bh = _brickGrid.height;
    int bd = _brickGrid.depth;
    Array3D<bool> newBricks = Array3D<bool>(bw, bh, bd, false);
    _getNewBrickLocations(brickPrev, brickCurrent, newBricks);

    std::vector<GridIndexVector> brickStructures;
    _getbrickStructures(brickCurrent, newBricks, brickStructures);
    _removeInvalidbrickStructures(brickCurrent, brickNext, brickStructures);

    _brickGridQueue[1] = brickCurrent;
}

/** given a 3d brick array b1 and b2, creates a new brick array where every brick where b1 is inactive and b2 is active is set to active, 
 * areas where b1 is active and b2 is active become inactive in new brick array. 
 * 
 * Basically if you call getNewBrickLocations(brickPrev, brickcurrent, brickNew)-> brickNew will tell us which of the bricks is a new addition to the brick grid */
void WaterBrickGrid::_getNewBrickLocations(Array3D<Brick> &b1, Array3D<Brick> &b2,
                                           Array3D<bool> &newBricks)
{
    for (int k = 0; k < b1.depth; k++)
    {
        for (int j = 0; j < b1.height; j++)
        {
            for (int i = 0; i < b1.width; i++)
            {
                if (!b1(i, j, k).isActive && b2(i, j, k).isActive)
                {
                    newBricks.set(i, j, k, true);
                }
            }
        }
    }
}

/**
 * removes small brick structures that are smaller (have fewer bricks) than a given threshold from brickCurrent grid*/
void WaterBrickGrid::_removeSmallBrickStructures()
{
    Array3D<Brick> brickCurrent = _brickGridQueue[1];
    Array3D<bool> allBricks = Array3D<bool>(brickCurrent.width,
                                            brickCurrent.height,
                                            brickCurrent.depth, false);

    for (int k = 0; k < brickCurrent.depth; k++)
    {
        for (int j = 0; j < brickCurrent.height; j++)
        {
            for (int i = 0; i < brickCurrent.width; i++)
            {
                if (brickCurrent(i, j, k).isActive)
                {
                    allBricks.set(i, j, k, true);
                }
            }
        }
    }

    std::vector<GridIndexVector> brickStructures;
    _getbrickStructures(brickCurrent, allBricks, brickStructures);
    for (unsigned int i = 0; i < brickStructures.size(); i++)
    {
        if (brickStructures[i].size() < _minNumberOfBricksInStructure)
        {
            _removeBrickStructureFromBrickGrid(brickStructures[i], brickCurrent);
        }
    }

    _brickGridQueue[1] = brickCurrent;
}

/** merges brickPrev with brickCurrent by setting everything in brickPrev to what is essentially the union of this with brickCurrent.
 * Sets the brickGridQueue 0 element to this new list 
 */
void WaterBrickGrid::_mergeBrickGrids()
{
    Array3D<Brick> brickPrev = _brickGridQueue[0];
    Array3D<Brick> brickCurrent = _brickGridQueue[1];

    for (int k = 0; k < brickPrev.depth; k++)
    {
        for (int j = 0; j < brickPrev.height; j++)
        {
            for (int i = 0; i < brickPrev.width; i++)
            {
                if (!brickPrev(i, j, k).isActive && brickCurrent(i, j, k).isActive)
                {
                    brickPrev.set(i, j, k, brickCurrent(i, j, k));
                }
            }
        }
    }

    _brickGridQueue[0] = brickPrev;
}

/** updates the water brick grid */
void WaterBrickGrid::update(ParticleLevelSet &liquidSDF, MeshLevelSet solidSDF, std::vector<VectorMath::vec3> &particles, double dt)
{
    CUSTOM_ASSERT(isBrickGridInitialized());

    _updateDensityGrid(particles, dt);
    _updateBrickGrid(liquidSDF, solidSDF);

    if (_brickGridQueueSize == 3)
    {
        _postProcessBrickGrid();
    }

    if (_brickGridQueueSize == 3)
    {
        _currentBrickGrid = _brickGridQueue[0];
        _isCurrentBrickGridReady = true;

        _brickGridQueue[0] = _brickGridQueue[1];
        _brickGridQueue[1] = _brickGridQueue[2];
        _brickGridQueue[2] = _brickGrid;
    }
    else
    {
        _brickGridQueue[_brickGridQueueSize] = _brickGrid;
        _brickGridQueueSize++;
    }
    _numUpdates++;
}

/** post process the brick by removing stray bricks, small brick structures and by merging brick grids */
void WaterBrickGrid::_postProcessBrickGrid()
{
    _removeStrayBricks();
    _removeSmallBrickStructures();
    _mergeBrickGrids();
}

void WaterBrickGrid::_updateAcceleration(ParticleLevelSet &liquidSDF, MeshLevelSet solidSDF, std::vector<FluidParticle> &particles, double dt)
{

    // count the number of particles in each cell and the average velocity vector
    Array3D<int> countGrid = Array3D<int>(_isize, _jsize, _ksize, 0);
    Array3D<VectorMath::vec3> avgVelocity = Array3D<VectorMath::vec3>(_isize, _jsize, _ksize, VectorMath::vec3(0, 0, 0));
    GridIndex g;

    for (unsigned int i = 0; i < particles.size(); i++)
    {
        FluidParticle particle = particles[i];
        g = Grid3D::positionToGridIndex(particle.position, _dx);
        if (Grid3D::isGridIndexInRange(g, _isize, _jsize, _ksize))
        {
            countGrid.add(g, 1);
            *avgVelocity.getPointer(g) = avgVelocity(g) + particle.velocity;
        }
    }

    // divide the velocity sum by count to get the average velocity for each cell
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {
                if (countGrid(i, j, k) != 0)
                {
                    *avgVelocity.getPointer(i, j, k) = avgVelocity(i, j, k) / countGrid(i, j, k);
                    //std::cout << "velocity " << avgVelocity(i, j, k) << std::endl;
                }
            }
        }
    }

    // set current and previous average velocity
    for (int k = 0; k < _ksize; k++)
    {
        for (int j = 0; j < _jsize; j++)
        {
            for (int i = 0; i < _isize; i++)
            {

                VectorMath::vec3 avgVelocityForCell = avgVelocity(i, j, k);
                if (avgVelocityForCell.x == 0.0f && avgVelocityForCell.y == 0.0f && avgVelocityForCell.z == 0.0f)
                {
                    continue;
                }

                VelocityInfo *node2 = _velocityGrid.getPointer(i, j, k);

                //std::cout << node2->currentVelocity.x << node2->currentVelocity.y << node2->currentVelocity.z << std::endl;

                if (_numUpdates == 0)
                {
                    node2->currentVelocity = VectorMath::vec3(0, 0, 0);
                }

                else
                {
                    node2->previousVelocity = node2->currentVelocity;
                    node2->currentVelocity = avgVelocityForCell;
                }
            }
        }
    }

    // done updating velocity grid

    _brickGrid.fill(Brick());

    double bw = _brick.width;
    double bh = _brick.height;
    double bd = _brick.depth;

    VectorMath::vec3 coffset = VectorMath::vec3(0.5 * bw, 0.5 * bh, 0.5 * bd);

    VectorMath::vec3 p;
    for (int k = 0; k < _brickGrid.depth; k++)
    {
        for (int j = 0; j < _brickGrid.height; j++)
        {
            for (int i = 0; i < _brickGrid.width; i++)
            {
                p = coffset + VectorMath::vec3(i * bw, j * bh, k * bd);
                g = Grid3D::positionToGridIndex(p, _dx);

                if (Grid3D::isPositionInGrid(p, _dx, _isize, _jsize, _ksize) && liquidSDF(g) < 0.0 && solidSDF(g) > 0.0)
                {

                    float intensity = _getBrickIntensityAcceleration(i, j, k, dt);

                    //std::cout << intensity << std::endl;

                    Brick b = Brick(intensity);
                    b.isActive = true;
                    _brickGrid.set(i, j, k, b);
                }
            }
        }
    }

    // remove active cells that have no active neighbors!
    Brick *b;
    GridIndex n;
    for (int k = 0; k < _brickGrid.depth; k++)
    {
        for (int j = 0; j < _brickGrid.height; j++)
        {
            for (int i = 0; i < _brickGrid.width; i++)
            {
                if (_brickGrid(i, j, k).isActive && !_isBrickNextToActiveNeighbour(i, j, k))
                {
                    b = _brickGrid.getPointer(i, j, k);
                    b->isActive = false;
                }
            }
        }
    }
}

/** updates the water brick grid based on density */
void WaterBrickGrid::update2(ParticleLevelSet &liquidSDF, MeshLevelSet solidSDF, std::vector<FluidParticle> &particles, double dt)
{
    CUSTOM_ASSERT(isBrickGridInitialized());
    //std::cout << "outside" << std::endl;
    //_updateDensityGrid2(particles);
    //_updateBrickGrid2(liquidSDF, solidSDF);
    _updateAcceleration(liquidSDF, solidSDF, particles, dt);

    if (_brickGridQueueSize == 3)
    {
        _postProcessBrickGrid();
    }

    if (_brickGridQueueSize == 3)
    {
        _currentBrickGrid = _brickGridQueue[0];
        _isCurrentBrickGridReady = true;

        _brickGridQueue[0] = _brickGridQueue[1];
        _brickGridQueue[1] = _brickGridQueue[2];
        _brickGridQueue[2] = _brickGrid;
    }
    else
    {
        _brickGridQueue[_brickGridQueueSize] = _brickGrid;
        _brickGridQueueSize++;
    }
    _numUpdates++;
}
