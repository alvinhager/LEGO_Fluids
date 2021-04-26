#ifndef VECTORMATH_AH
#define VECTORMATH_AH

#include "CustomAssert.h"
#include <math.h>
#include <iostream>

// namespace containing different useful math utility functions and classes for vectors
namespace VectorMath
{
    // 3-dim vector using floats
    class vec3
    {
    public:
        float x;
        float y;
        float z;

        vec3();
        vec3(float x, float y, float z);
        ~vec3();

        vec3 add(const vec3 &v);
        vec3 subtract(const vec3 &v);
        vec3 mult(float s);
        vec3 divide(float s);
        vec3 negate();

        float get(int i);
        float operator[](int i);
        float dot(const vec3 &v);
        vec3 cross(const vec3 &v);

        float length();
        float lengthsq();
        vec3 normalize();
    };

    vec3 operator+(const vec3 &v1, const vec3 &v2);
    vec3 &operator+=(vec3 &v1, const vec3 &v2);
    vec3 operator-(const vec3 &v1, const vec3 &v2);
    vec3 &operator-=(vec3 &v1, const vec3 &v2);
    vec3 operator*(float s, const vec3 &v);
    vec3 operator*(const vec3 &v, float s);
    vec3 &operator*=(vec3 &v, float s);
    vec3 operator/(const vec3 &v, float s);
    vec3 &operator/=(vec3 &v1, float s);
    vec3 operator-(const vec3 &v);
    std::ostream &operator<<(std::ostream &os, const vec3 &v);

    inline float dot(const vec3 &v1, const vec3 &v2)
    {
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    inline vec3 cross(const vec3 &v1, const vec3 &v2)
    {
        return vec3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
    }

    inline float lengthsq(const vec3 &v)
    {
        return v.x * v.x + v.y * v.y + v.z * v.z;
    }

    inline float length(const vec3 &v)
    {
        return sqrt(VectorMath::lengthsq(v));
    }

    inline vec3 normalize(const vec3 &v)
    {
        return v / VectorMath::length(v);
    }

    inline bool equals(const vec3 &v1, const vec3 &v2, double eps)
    {
        bool x_equal = fabs((double)v1.x - (double)v2.x) < eps;
        bool y_equal = fabs((double)v1.x - (double)v2.x) < eps;
        bool z_equal = fabs((double)v1.x - (double)v2.x) < eps;

        return x_equal && y_equal && z_equal;
    }

    // 3x3 matrix class stored in column major order
    class mat3
    {
    public:
        mat3();
        mat3(const vec3 &v1, const vec3 &v2, const vec3 &v3);
        mat3(const float vals[9]);
        mat3(float v0, float v1, float v2,
             float v3, float v4, float v5,
             float v6, float v7, float v8);
        mat3(float fillval);
        ~mat3();

        mat3 add(const mat3 &m);
        mat3 subtract(const mat3 &m);
        mat3 mult(float s);
        mat3 mult(const mat3 &m);
        vec3 mult(const vec3 &v);
        mat3 divide(float s);
        mat3 negate();
        vec3 get(int i);
        vec3 operator[](int i);
        mat3 transpose();

        float m[9];
    };

    std::ostream &operator<<(std::ostream &os, const mat3 &m);
    mat3 operator+(const mat3 &m1, const mat3 &m2);
    mat3 &operator+=(mat3 &m1, const mat3 &m2);
    mat3 operator-(const mat3 &m1, const mat3 &m2);
    mat3 &operator-=(mat3 &m1, const mat3 &m2);
    mat3 operator*(float s, const mat3 &m);
    mat3 operator*(const mat3 &m, float s);
    mat3 operator*(const mat3 &m1, const mat3 &m2);
    vec3 operator*(const mat3 &m, const vec3 &v);
    mat3 &operator*=(mat3 &m, float s);
    mat3 operator/(const mat3 &m, float s);
    mat3 &operator/=(mat3 &m, float s);
    mat3 operator-(const mat3 &m);

    inline mat3 transpose(const mat3 &m)
    {
        return mat3(m.m[0], m.m[3], m.m[6], m.m[1], m.m[4], m.m[7], m.m[2], m.m[5], m.m[8]);
    }

} // namespace VectorMath
#endif
