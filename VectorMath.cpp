
#include "VectorMath.h"

// VECTOR DEFINITIONS

// default constructor
VectorMath::vec3::vec3() : x(0.0), y(0.0), z(0.0)
{
}

// parameter constructor
VectorMath::vec3::vec3(float x, float y, float z) : x(x), y(y), z(z)
{
}

// default destructor
VectorMath::vec3::~vec3()
{
}

VectorMath::vec3 VectorMath::vec3::getAbsoluteValueComponentsVector()
{
    return VectorMath::vec3(fabs(x), fabs(y), fabs(z));
}

// for printing out vectors
std::ostream &VectorMath::operator<<(std::ostream &os, const VectorMath::vec3 &v)
{
    os << v.x << "\t" << v.y << "\t" << v.z;
    return os;
}

// addition of two vectors
VectorMath::vec3 VectorMath::vec3::add(const VectorMath::vec3 &v)
{
    return *this + v;
}
VectorMath::vec3 VectorMath::operator+(const VectorMath::vec3 &v1, const VectorMath::vec3 &v2)
{
    return VectorMath::vec3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}
VectorMath::vec3 &VectorMath::operator+=(VectorMath::vec3 &v1, const VectorMath::vec3 &v2)
{
    v1.x += v2.x;
    v1.y += v2.y;
    v1.z += v2.z;
    return v1;
}

// subtraction of two vectors
VectorMath::vec3 VectorMath::vec3::subtract(const VectorMath::vec3 &v)
{
    return *this - v;
}

VectorMath::vec3 VectorMath::operator-(const VectorMath::vec3 &v1, const VectorMath::vec3 &v2)
{
    return VectorMath::vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

VectorMath::vec3 &VectorMath::operator-=(VectorMath::vec3 &v1, const VectorMath::vec3 &v2)
{
    v1.x -= v2.x;
    v1.y -= v2.y;
    v1.z -= v2.z;
    return v1;
}

// multiplies a vector with a number
VectorMath::vec3 VectorMath::vec3::mult(float n)
{
    return n * (*this);
}

// multiplication between number and vector
VectorMath::vec3 VectorMath::operator*(float n, const VectorMath::vec3 &v)
{
    return VectorMath::vec3(v.x * n, v.y * n, v.z * n);
}

// multiplication between vector and number
VectorMath::vec3 VectorMath::operator*(const VectorMath::vec3 &v, float n)
{
    return VectorMath::vec3(v.x * n, v.y * n, v.z * n);
}

// *= operator
VectorMath::vec3 &VectorMath::operator*=(VectorMath::vec3 &v, float n)
{
    v.x *= n;
    v.y *= n;
    v.z *= n;
    return v;
}

// divides a vector by given number
VectorMath::vec3 VectorMath::vec3::divide(float n)
{
    return *this / n;
}

// division operator
VectorMath::vec3 VectorMath::operator/(const VectorMath::vec3 &v, float n)
{
    float inv = 1.0f / n;
    return VectorMath::vec3(v.x * inv, v.y * inv, v.z * inv);
}

// divide by equal operator
VectorMath::vec3 &VectorMath::operator/=(VectorMath::vec3 &v1, float n)
{
    float inv = 1.0f / n;
    v1.x *= inv;
    v1.y *= inv;
    v1.z *= inv;
    return v1;
}

// negates the vector
VectorMath::vec3 VectorMath::vec3::negate()
{
    return -(*this);
}

// returns the negation of the vector
VectorMath::vec3 VectorMath::operator-(const VectorMath::vec3 &v)
{
    return VectorMath::vec3(-v.x, -v.y, -v.z);
}

// returns the value at xyz component of choice
float VectorMath::vec3::get(int i)
{
    CUSTOM_ASSERT(i >= 0 && i <= 2);
    return (&x)[i];
}

// returns xyz component of choice
float VectorMath::vec3::operator[](int i)
{
    CUSTOM_ASSERT(i >= 0 && i <= 2);
    return (&x)[i];
}

// returns dot product of this vector and another vector
float VectorMath::vec3::dot(const VectorMath::vec3 &v)
{
    return VectorMath::dot(*this, v);
}

// returns crossproduct of this vector with another
VectorMath::vec3 VectorMath::vec3::cross(const VectorMath::vec3 &v)
{
    return VectorMath::cross(*this, v);
}

// returns the length squared of the vector
float VectorMath::vec3::lengthsq()
{
    return VectorMath::lengthsq(*this);
}

// returns the length of vector
float VectorMath::vec3::length()
{
    return VectorMath::length(*this);
}

// normalises the vector
VectorMath::vec3 VectorMath::vec3::normalize()
{
    return VectorMath::normalize(*this);
}

// MATRIX DEFINITIONS

// by default the matrix is the identity matrix
VectorMath::mat3::mat3()
{
    m[0] = 1.0;
    m[1] = 0.0;
    m[2] = 0.0;
    m[3] = 0.0;
    m[4] = 1.0;
    m[5] = 0.0;
    m[6] = 0.0;
    m[7] = 0.0;
    m[8] = 1.0;
}

// builds a 3-dim matrix out of 3 vectors
VectorMath::mat3::mat3(const VectorMath::vec3 &v1, const VectorMath::vec3 &v2, const VectorMath::vec3 &v3)
{
    m[0] = v1.x;
    m[1] = v1.y;
    m[2] = v1.z;
    m[3] = v2.x;
    m[4] = v2.y;
    m[5] = v2.z;
    m[6] = v3.x;
    m[7] = v3.y;
    m[8] = v3.z;
}

// builds a 3-dim matrix out of matrix of values
VectorMath::mat3::mat3(const float vals[9])
{
    m[0] = vals[0];
    m[1] = vals[1];
    m[2] = vals[2];
    m[3] = vals[3];
    m[4] = vals[4];
    m[5] = vals[5];
    m[6] = vals[6];
    m[7] = vals[7];
    m[8] = vals[8];
}

// builds a 3-dim matrix out of values given as parameters
VectorMath::mat3::mat3(float v0, float v1, float v2,
                       float v3, float v4, float v5,
                       float v6, float v7, float v8)
{
    m[0] = v0;
    m[1] = v1;
    m[2] = v2;
    m[3] = v3;
    m[4] = v4;
    m[5] = v5;
    m[6] = v6;
    m[7] = v7;
    m[8] = v8;
}

// fills all the cells of the matrix with this value
VectorMath::mat3::mat3(float fillvalue)
{
    m[0] = fillvalue;
    m[1] = fillvalue;
    m[2] = fillvalue;
    m[3] = fillvalue;
    m[4] = fillvalue;
    m[5] = fillvalue;
    m[6] = fillvalue;
    m[7] = fillvalue;
    m[8] = fillvalue;
}

// default destructor
VectorMath::mat3::~mat3()
{
}

// printing out matrix
std::ostream &VectorMath::operator<<(std::ostream &os, const VectorMath::mat3 &m)
{
    os << m.m[0] << "\t" << m.m[3] << "\t" << m.m[6] << std::endl
       << m.m[1] << "\t" << m.m[4] << "\t" << m.m[7] << std::endl
       << m.m[2] << "\t" << m.m[5] << "\t" << m.m[8];
    return os;
}

// addition
VectorMath::mat3 VectorMath::mat3::add(const mat3 &m)
{
    return *this + m;
}

VectorMath::mat3 VectorMath::operator+(const VectorMath::mat3 &m1, const VectorMath::mat3 &m2)
{
    return VectorMath::mat3(m1.m[0] + m2.m[0], m1.m[1] + m2.m[1], m1.m[2] + m2.m[2],
                            m1.m[3] + m2.m[3], m1.m[4] + m2.m[4], m1.m[5] + m2.m[5],
                            m1.m[6] + m2.m[6], m1.m[7] + m2.m[7], m1.m[8] + m2.m[8]);
}

VectorMath::mat3 &VectorMath::operator+=(VectorMath::mat3 &m1, const VectorMath::mat3 &m2)
{
    m1.m[0] += m2.m[0];
    m1.m[1] += m2.m[1];
    m1.m[2] += m2.m[2];
    m1.m[3] += m2.m[3];
    m1.m[4] += m2.m[4];
    m1.m[5] += m2.m[5];
    m1.m[6] += m2.m[6];
    m1.m[7] += m2.m[7];
    m1.m[8] += m2.m[8];
    return m1;
}

// subtraction
VectorMath::mat3 VectorMath::mat3::subtract(const mat3 &m)
{
    return *this - m;
}

VectorMath::mat3 VectorMath::operator-(const VectorMath::mat3 &m1, const VectorMath::mat3 &m2)
{
    return VectorMath::mat3(m1.m[0] - m2.m[0], m1.m[1] - m2.m[1], m1.m[2] - m2.m[2],
                            m1.m[3] - m2.m[3], m1.m[4] - m2.m[4], m1.m[5] - m2.m[5],
                            m1.m[6] - m2.m[6], m1.m[7] - m2.m[7], m1.m[8] - m2.m[8]);
}

VectorMath::mat3 &VectorMath::operator-=(VectorMath::mat3 &m1, const VectorMath::mat3 &m2)
{
    m1.m[0] -= m2.m[0];
    m1.m[1] -= m2.m[1];
    m1.m[2] -= m2.m[2];
    m1.m[3] -= m2.m[3];
    m1.m[4] -= m2.m[4];
    m1.m[5] -= m2.m[5];
    m1.m[6] -= m2.m[6];
    m1.m[7] -= m2.m[7];
    m1.m[8] -= m2.m[8];
    return m1;
}

// multiplication
VectorMath::mat3 VectorMath::mat3::mult(float n)
{
    return *this * n;
}

VectorMath::mat3 VectorMath::mat3::mult(const VectorMath::mat3 &m)
{
    return *this * m;
}

VectorMath::vec3 VectorMath::mat3::mult(const VectorMath::vec3 &v)
{
    return *this * v;
}

VectorMath::mat3 VectorMath::operator*(float n, const VectorMath::mat3 &m)
{
    return VectorMath::mat3(m.m[0] * n, m.m[1] * n, m.m[2] * n,
                            m.m[3] * n, m.m[4] * n, m.m[5] * n,
                            m.m[6] * n, m.m[7] * n, m.m[8] * n);
}

VectorMath::mat3 VectorMath::operator*(const VectorMath::mat3 &m, float n)
{
    return VectorMath::mat3(m.m[0] * n, m.m[1] * n, m.m[2] * n,
                            m.m[3] * n, m.m[4] * n, m.m[5] * n,
                            m.m[6] * n, m.m[7] * n, m.m[8] * n);
}

VectorMath::mat3 VectorMath::operator*(const VectorMath::mat3 &m1, const VectorMath::mat3 &m2)
{
    return VectorMath::mat3(m1.m[0] * m2.m[0] + m1.m[3] * m2.m[1] + m1.m[6] * m2.m[2],
                            m1.m[1] * m2.m[0] + m1.m[4] * m2.m[1] + m1.m[7] * m2.m[2],
                            m1.m[2] * m2.m[0] + m1.m[5] * m2.m[1] + m1.m[8] * m2.m[2],
                            m1.m[0] * m2.m[3] + m1.m[3] * m2.m[4] + m1.m[6] * m2.m[5],
                            m1.m[1] * m2.m[3] + m1.m[4] * m2.m[4] + m1.m[7] * m2.m[5],
                            m1.m[2] * m2.m[3] + m1.m[5] * m2.m[4] + m1.m[8] * m2.m[5],
                            m1.m[0] * m2.m[6] + m1.m[3] * m2.m[7] + m1.m[6] * m2.m[8],
                            m1.m[1] * m2.m[6] + m1.m[4] * m2.m[7] + m1.m[7] * m2.m[8],
                            m1.m[2] * m2.m[6] + m1.m[5] * m2.m[7] + m1.m[8] * m2.m[8]);
}

VectorMath::vec3 VectorMath::operator*(const VectorMath::mat3 &m, const VectorMath::vec3 &v)
{
    return VectorMath::vec3(m.m[0] * v.x + m.m[3] * v.y + m.m[6] * v.z,
                            m.m[1] * v.x + m.m[4] * v.y + m.m[7] * v.z,
                            m.m[2] * v.x + m.m[5] * v.y + m.m[8] * v.z);
}

VectorMath::mat3 &VectorMath::operator*=(VectorMath::mat3 &m, float n)
{
    m.m[0] *= n;
    m.m[1] *= n;
    m.m[2] *= n;
    m.m[3] *= n;
    m.m[4] *= n;
    m.m[5] *= n;
    m.m[6] *= n;
    m.m[7] *= n;
    m.m[8] *= n;
    return m;
}

// division
VectorMath::mat3 VectorMath::mat3::divide(float n)
{
    return *this / n;
}

VectorMath::mat3 VectorMath::operator/(const VectorMath::mat3 &m, float n)
{
    float inv = 1.0f / n;
    return VectorMath::mat3(m.m[0] * inv, m.m[1] * inv, m.m[2] * inv,
                            m.m[3] * inv, m.m[4] * inv, m.m[5] * inv,
                            m.m[6] * inv, m.m[7] * inv, m.m[8] * inv);
}

VectorMath::mat3 &VectorMath::operator/=(VectorMath::mat3 &m, float n)
{
    float inv = 1.0f / n;
    m.m[0] *= inv;
    m.m[1] *= inv;
    m.m[2] *= inv;
    m.m[3] *= inv;
    m.m[4] *= inv;
    m.m[5] *= inv;
    m.m[6] *= inv;
    m.m[7] *= inv;
    m.m[8] *= inv;
    return m;
}

// negation of matrix
VectorMath::mat3 VectorMath::mat3::negate()
{
    return -(*this);
}

VectorMath::mat3 VectorMath::operator-(const VectorMath::mat3 &m)
{
    return VectorMath::mat3(-m.m[0], -m.m[1], -m.m[2],
                            -m.m[3], -m.m[4], -m.m[5],
                            -m.m[6], -m.m[7], -m.m[8]);
}

// return one cross section of matrix
VectorMath::vec3 VectorMath::mat3::get(int i)
{
    CUSTOM_ASSERT(i <= 2 && i >= 0);
    return vec3(m[3 * i], m[3 * i + 1], m[3 * i + 2]);
}

// return one cross section of matrix
VectorMath::vec3 VectorMath::mat3::operator[](int i)
{
    CUSTOM_ASSERT(i <= 2 && i >= 0);
    return vec3(m[3 * i], m[3 * i + 1], m[3 * i + 2]);
}

// return a transposed version of matrix
VectorMath::mat3 VectorMath::mat3::transpose()
{
    return VectorMath::transpose(*this);
}
