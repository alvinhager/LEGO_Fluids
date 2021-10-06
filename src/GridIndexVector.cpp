#include "GridIndexVector.h"

GridIndexVector::GridIndexVector()
{
}

GridIndexVector::GridIndexVector(int i, int j, int k) : width(i), height(j), depth(k)
{
}

GridIndexVector::~GridIndexVector()
{
}

GridIndex GridIndexVector::operator[](int i)
{
    CUSTOM_ASSERT(i >= 0 && i < (int)_indices.size());
    return _getUnflattenedIndex(_indices[i]);
}

void GridIndexVector::insert(std::vector<GridIndex> &indices)
{
    reserve(_indices.size() + indices.size());
    for (unsigned int i = 0; i < indices.size(); i++)
    {
        push_back(indices[i]);
    }
}

void GridIndexVector::insert(GridIndexVector &indices)
{
    CUSTOM_ASSERT(width == indices.width && height == indices.height && depth == indices.depth);

    reserve(_indices.size() + indices.size());
    int maxidx = width * height * depth - 1;
    for (unsigned int i = 0; i < indices.size(); i++)
    {
        int flatidx = indices.getFlatIndex(i);
        CUSTOM_ASSERT(flatidx >= 0 && flatidx <= maxidx);
        _indices.push_back(flatidx);
    }
}

std::vector<GridIndex> GridIndexVector::getVector()
{
    std::vector<GridIndex> vector;
    vector.reserve(size());

    for (unsigned int i = 0; i < size(); i++)
    {
        vector.push_back((*this)[i]);
    }

    return vector;
}

void GridIndexVector::getVector(std::vector<GridIndex> &vector)
{
    vector.reserve(size());

    for (unsigned int i = 0; i < size(); i++)
    {
        vector.push_back((*this)[i]);
    }
}