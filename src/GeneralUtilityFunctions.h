
#pragma once
#include <sstream>
#include "TriangleMeshFormat.h"

namespace GeneralUtilityFunctions
{
    int getNumberOfDigitsInInteger(int num);
    std::string numberToString(int num);
    std::string getFileExtension(TriangleMeshFormat format);
}
