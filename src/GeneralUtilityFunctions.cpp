#include "GeneralUtilityFunctions.h"

int GeneralUtilityFunctions::getNumberOfDigitsInInteger(int num)
{
    int count = 0;
    while (num != 0)
    {
        num /= 10;
        count++;
    }
    return count;
};

std::string GeneralUtilityFunctions::numberToString(int number)
{
    std::ostringstream ss;
    ss << number;
    return ss.str();
};

std::string GeneralUtilityFunctions::getFileExtension(TriangleMeshFormat fmt)
{
    if (fmt == TriangleMeshFormat::ply)
    {
        return "ply";
    }
    else
    {
        return "bobj";
    }
}