#pragma once

#include "GridIndex.h"
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <sstream>

/** polymorphic 3D array class that can store different types of values on a 3D grid */
template <class T>
class Array3D
{
public:
    int width = 0;
    int height = 0;
    int depth = 0;

    /** empty constructor */
    Array3D()
    {
        _initializeGrid();
    };

    /** constructor with width, height depth */
    Array3D(int w, int h, int d) : width(w), height(h), depth(d), _size(w * h * d)
    {
        _initializeGrid();
    }

    /** constructor with width, height depth that also fills grid with fillval */
    Array3D(int w, int h, int d, T fillVal) : width(w), height(h), depth(d), _size(w * h * d)
    {
        _initializeGrid();
        fill(fillVal);
    }

    /** copy constructor */
    Array3D(const Array3D &obj)
    {
        width = obj.width;
        height = obj.height;
        depth = obj.depth;
        _size = obj._size;

        _initializeGrid();

        T value;
        for (int k = 0; k < depth; k++)
        {
            for (int j = 0; j < height; j++)
            {
                for (int i = 0; i < width; i++)
                {
                    value = obj._grid[_getFlatIndex(i, j, k)];
                    set(i, j, k, value);
                }
            }
        }

        if (obj._outOfRangeAllowed)
        {
            _outOfRangeReturnValue = obj._outOfRangeReturnValue;
            _outOfRangeAllowed = true;
        }
    }

    /** assignment operator: sets the object to the rhs of equal operator */
    Array3D operator=(const Array3D &rhs)
    {
        delete[] _grid;
        width = rhs.width;
        height = rhs.height;
        depth = rhs.depth;
        _size = rhs._size;

        _initializeGrid();

        T val;
        for (int k = 0; k < depth; k++)
        {
            for (int j = 0; j < height; j++)
            {
                for (int i = 0; i < width; i++)
                {
                    val = rhs._grid[_getFlatIndex(i, j, k)];
                    set(i, j, k, val);
                }
            }
        }

        if (rhs._outOfRangeAllowed)
        {
            _outOfRangeReturnValue = rhs._outOfRangeReturnValue;
            _outOfRangeAllowed = true;
        }

        return *this;
    }

    /** destructor */
    ~Array3D()
    {
        delete[] _grid;
    }

    /** fills the entire grid with this value */
    void fill(T val)
    {
        for (int i = 0; i < width * height * depth; i++)
        {
            _grid[i] = val;
        }
    }

    /** returns a copy of the value at (i,j,k) */
    T operator()(int i, int j, int k)
    {
        bool inRange = _isIndexInRange(i, j, k);
        if (!inRange && _outOfRangeAllowed)
        {
            return _outOfRangeReturnValue;
        }

        if (!inRange)
        {
            std::string errorMsg = "Error, index out of range. ";
            errorMsg += "i: " + _toString(i) + " j: " + _toString(j) + " k: " + _toString(k) + "\n";
            throw std::out_of_range(errorMsg);
        }

        return _grid[_getFlatIndex(i, j, k)];
    }

    /** returns a copy of the value at a grid index */
    T operator()(GridIndex g)
    {
        bool inRange = _isIndexInRange(g.i, g.j, g.k);
        if (!inRange && _outOfRangeAllowed)
        {
            return _outOfRangeReturnValue;
        }

        if (!inRange)
        {
            std::string errorMsg = "Error, index out of range. ";
            errorMsg += "i: " + _toString(g.i) + " j: " + _toString(g.j) + " k: " + _toString(g.k) + "\n";
            throw std::out_of_range(errorMsg);
        }

        return _grid[_getFlatIndex(g)];
    }

    /** returns a copy of the value at the flat index given */
    T operator()(int flatIndex)
    {
        bool inRange = flatIndex >= 0 && flatIndex < _size;
        if (!inRange && _outOfRangeAllowed)
        {
            return _outOfRangeReturnValue;
        }

        if (!inRange)
        {
            std::string errorMsg = "Error, index out of range. ";
            errorMsg += "index: " + _toString(flatIndex) + "\n";
            throw std::out_of_range(errorMsg);
        }

        return _grid[flatIndex];
    }

    /** returns a copy of the value on the grid at the (i,j,k) */
    T get(int i, int j, int k)
    {
        bool inRange = _isIndexInRange(i, j, k);
        if (!inRange && _outOfRangeAllowed)
        {
            return _outOfRangeReturnValue;
        }

        if (!inRange)
        {
            std::string errorMsg = "Error, index out of range. ";
            errorMsg += "i: " + _toString(i) + " j: " + _toString(j) + " k: " + _toString(k) + "\n";
            throw std::out_of_range(errorMsg);
        }

        return _grid[_getFlatIndex(i, j, k)];
    }

    /** return the a copy of the value on teh grid at the flat index */
    T get(int flatIndex)
    {
        bool inRange = flatIndex >= 0 && flatIndex < _size;
        if (!inRange && _outOfRangeAllowed)
        {
            return _outOfRangeReturnValue;
        }

        if (!inRange)
        {
            std::string errorMsg = "Error, index out of range. ";
            errorMsg += "index: " + _toString(flatIndex) + "\n";
            throw std::out_of_range(errorMsg);
        }

        return _grid[flatIndex];
    }

    /** sets (i,j,k) to the given value */
    void set(int i, int j, int k, T value)
    {
        if (!_isIndexInRange(i, j, k))
        {
            std::string errorMsg = "Error, index out of range. ";
            errorMsg += "i: " + _toString(i) + " j: " + _toString(j) + " k: " + _toString(k) + "\n";
            throw std::out_of_range(errorMsg);
        }

        _grid[_getFlatIndex(i, j, k)] = value;
    }

    /** sets the grid index to the value */
    void set(GridIndex g, T value)
    {
        if (!_isIndexInRange(g))
        {
            std::string msg = "Error: index out of range.\n";
            msg += "i: " + _toString(g.i) + " j: " + _toString(g.j) + " k: " + _toString(g.k) + "\n";
            throw std::out_of_range(msg);
        }

        _grid[_getFlatIndex(g)] = value;
    }

    /** sets the given cells to the given value */
    void set(std::vector<GridIndex> &cells, T value)
    {
        for (unsigned int i = 0; i < cells.size(); i++)
        {
            set(cells[i], value);
        }
    }

    /** sets the cell specified by the flat index to a specific value */
    void set(int flatIndex, T value)
    {
        if (!(flatIndex >= 0 && flatIndex < _size))
        {
            std::string errorMsg = "Error, index out of range. ";
            errorMsg += "index: " + _toString(flatIndex) + "\n";
            throw std::out_of_range(errorMsg);
        }

        _grid[flatIndex] = value;
    }

    /** adds a given value to the given cell (i,j,k) */
    void add(int i, int j, int k, T value)
    {
        if (!_isIndexInRange(i, j, k))
        {
            std::string errorMsg = "Error, index out of range. ";
            errorMsg += "i: " + _toString(i) + " j: " + _toString(j) + " k: " + _toString(k) + "\n";
            throw std::out_of_range(errorMsg);
        }

        _grid[_getFlatIndex(i, j, k)] += value;
    }

    /** adds a given value to the given cell g */
    void add(GridIndex g, T value)
    {
        if (!_isIndexInRange(g))
        {
            std::string errorMsg = "Error, index out of range. ";
            errorMsg += "i: " + _toString(g.i) + " j: " + _toString(g.j) + " k: " + _toString(g.k) + "\n";
            throw std::out_of_range(errorMsg);
        }

        _grid[_getFlatIndex(g)] += value;
    }

    /** adds a given value to the given cell specified by the flat index */
    void add(int flatIndex, T value)
    {
        if (!(flatIndex >= 0 && flatIndex < _size))
        {
            std::string errorMsg = "Error, index out of range. ";
            errorMsg += "index: " + _toString(flatIndex) + "\n";
            throw std::out_of_range(errorMsg);
        }

        _grid[flatIndex] += value;
    }

    /** returns a pointer to the (i,j,k) cell */
    T *getPointer(int i, int j, int k)
    {
        bool inRange = _isIndexInRange(i, j, k);
        if (!inRange && _outOfRangeAllowed)
        {
            return &_outOfRangeReturnValue;
        }

        if (!inRange)
        {
            std::string errorMsg = "Error, index out of range. ";
            errorMsg += "i: " + _toString(i) + " j: " + _toString(j) + " k: " + _toString(k) + "\n";
            throw std::out_of_range(errorMsg);
        }

        return &_grid[_getFlatIndex(i, j, k)];
    }

    /** returns a pointer to the g indeed cell */
    T *getPointer(GridIndex g)
    {
        bool inRange = _isIndexInRange(g.i, g.j, g.k);
        if (!inRange && _outOfRangeAllowed)
        {
            return &_outOfRangeReturnValue;
        }

        if (!inRange)
        {
            std::string errorMsg = "Error, index out of range. ";
            errorMsg += "i: " + _toString(g.i) + " j: " + _toString(g.j) + " k: " + _toString(g.k) + "\n";
            throw std::out_of_range(errorMsg);
        }

        return &_grid[_getFlatIndex(g)];
    }

    /** returns a pointer to the cell specified by the flat index */
    T *getPointer(int flatIndex)
    {
        bool inRange = flatIndex >= 0 && flatIndex < _size;
        if (!inRange && _outOfRangeAllowed)
        {
            return &_outOfRangeReturnValue;
        }

        if (!inRange)
        {
            std::string errorMsg = "Error, index out of range. ";
            errorMsg += "index: " + _toString(flatIndex) + "\n";
            throw std::out_of_range(errorMsg);
        }

        return &_grid[flatIndex];
    }

    /** returns a pointer to the grid field i.e. raw array */
    T *getRawArray()
    {
        return _grid;
    }

    /** returns the size of the grid, i.e. number of elements it (can) store */
    int getSize()
    {
        return _size;
    }

    /** sets the out of range value set field to false */
    void setOutOfRangeAllowedToFalse()
    {
        _outOfRangeAllowed = false;
    }

    /** sets the out of range value to val + out of range value set to true */
    void setOutOfRangeReturnValue(T val)
    {
        _outOfRangeReturnValue = val;
        _outOfRangeAllowed = true;
    }

    /** return out of range value set */
    bool outOfRangeAllowed()
    {
        return _outOfRangeAllowed;
    }

    /** return out of range value */
    T getoutOfRangeReturnValue()
    {
        return _outOfRangeReturnValue;
    }

    /** return if index (i,j,k) is in range of the grid */
    inline bool isIndexInRange(int i, int j, int k)
    {
        return i >= 0 && j >= 0 && k >= 0 && i < width && j < height && k < depth;
    }

    /** return if index g is in range of the grid */
    inline bool isIndexInRange(GridIndex g)
    {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && g.i < width && g.j < height && g.k < depth;
    }

private:
    T *_grid;
    int _size = 0;
    bool _outOfRangeAllowed = false;
    // returns this value as error code if we try to access something out of range
    T _outOfRangeReturnValue;

    /** used to convert different items to strings */
    template <class S>
    std::string _toString(S item)
    {
        std::ostringstream sstream;
        sstream << item;

        return sstream.str();
    }

    /** initializes the grid by checking that w,h,d > 0 
     * and initialising a grid on the heap with dimensions specified by class values */
    void _initializeGrid()
    {
        if (width < 0 || height < 0 || depth < 0)
        {
            std::string errorMsg = "Error, dimensions of the 3D array grid cannot be negative. \n";
            errorMsg += "width: " + _toString(width) +
                        " height: " + _toString(height) +
                        " depth: " + _toString(depth) + "\n";
            throw std::domain_error(errorMsg);
        }

        _grid = new T[width * height * depth];
    }

    /** checks if the given index is in range of grid */
    inline bool _isIndexInRange(int i, int j, int k)
    {
        return i >= 0 && j >= 0 && k >= 0 && i < width && j < height && k < depth;
    }

    /** checks if the given index is in range of */
    inline bool _isIndexInRange(GridIndex g)
    {
        return g.i >= 0 && g.j >= 0 && g.k >= 0 && g.i < width && g.j < height && g.k < depth;
    }

    /** converts an grid index to a flat index */
    inline unsigned int _getFlatIndex(int i, int j, int k)
    {
        return (unsigned int)i + (unsigned int)width * ((unsigned int)j + (unsigned int)height * (unsigned int)k);
    }

    /** converts an grid index to a flat index */
    inline unsigned int _getFlatIndex(GridIndex g)
    {
        return (unsigned int)g.i + (unsigned int)width * ((unsigned int)g.j + (unsigned int)height * (unsigned int)g.k);
    }
};