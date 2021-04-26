#include <iostream>

#include "triangle.h"
#include "GridIndex.h"
#include "Array3D.h"

using namespace std;
int main()
{

    GridIndex hey = GridIndex(1, 2, 3);
    GridIndex hey2 = GridIndex(1, 2, 3);

    cout << "Hey" << (hey != hey2) << endl;
    return 0;
}