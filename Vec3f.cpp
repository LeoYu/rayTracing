#include "Vec3f.h"
#include <math.h>
#include <cstring>
#define LD double

namespace SimpleOBJ
{

//////////////////////////////////////////////////////////////////////////
//  Constructors and Deconstructors
	Vec3f::Vec3f(void) : x(0), y(0), z(0) {}
    
	Vec3f::Vec3f(LD x, LD y, LD z) :x(x), y(y), z(z) {}

	Vec3f::Vec3f(const Vec3f &v) : x(v.x), y(v.y), z(v.z) {}

    Vec3f::~Vec3f(void)
    {

    }

//////////////////////////////////////////////////////////////////////////
// Operators

    Vec3f& Vec3f::operator =( const Vec3f& v)
    {
		x = v.x, y = v.y, z = v.z;
        return (*this);
    }

    void Vec3f::operator +=(const Vec3f& v)
    {
		x += v.x, y += v.y, z += v.z;
    }
    void Vec3f::operator +=(LD f)
    {
		x += f, y += f, z += f;
    }

    void Vec3f::operator -=(const Vec3f& v)
    {
		x -= v.x, y -= v.y, z -= v.z;
    }
    void Vec3f::operator -=(LD f)
    {
        x -= f, y -= f, z -= f;
    }

    void Vec3f::operator *=(const Vec3f& v)
    {
        x *= v.x, y *= v.y, z *= v.z;
    }
    void Vec3f::operator *=(LD f)
    {
        x *= f, y *= f, z *= f;
    }

    void Vec3f::operator /=(const Vec3f& v)
    {
        x /= v.x, y /= v.y, z /= v.z;
    }
    void Vec3f::operator /=(LD f)
    {
        x /= f, y /= f, z /= f;
    }

    Vec3f Vec3f::operator +(const Vec3f&v) const
    {
        return Vec3f(x + v.x, y + v.y, z + v.z);
    }
    Vec3f Vec3f::operator +(LD f) const
    {
        return Vec3f(x + f, y + f, z + f);
    }

    Vec3f Vec3f::operator -(const Vec3f&v) const
    {
        return Vec3f(x - v.x, y - v.y, z - v.z);
    }
    Vec3f Vec3f::operator -(LD f) const
    {
        return Vec3f(x - f, y - f, z - f);
    }

    LD Vec3f::operator *(const Vec3f&v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }
    Vec3f Vec3f::operator *(LD f) const
    {
        return Vec3f(x * f, y * f, z * f);
    }

    Vec3f Vec3f::operator /(const Vec3f&v) const
    {
        return Vec3f(x / v.x, y / v.y, z / v.z);
    }
    Vec3f Vec3f::operator /(LD f) const
    {
        return Vec3f(x / f, y / f, z / f);
    }

    Vec3f Vec3f::operator - () const 
    {
        return Vec3f(- x, - y, - z);
    }

	Vec3f Vec3f::operator %(const Vec3f&v) const
	{
		return Vec3f(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
	}

//////////////////////////////////////////////////////////////////////////
// Other Methods
    void Vec3f::Normalize()
    {
        LD fSqr = L2Norm_Sqr();
        if(fSqr>1e-6)
            (*this) *= 1.0f/sqrt(fSqr);
    }

	LD Vec3f::L2Norm_Sqr()
	{
		return x * x + y * y + z * z;
	}
	LD Vec3f::L2Norm()
	{
		return sqrt(x * x + y * y + z * z);
	}
}


