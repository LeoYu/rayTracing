#ifndef VEC3F_H
#define VEC3F_H
#define LD double

namespace SimpleOBJ
{
    class Vec3f
    {
    public:

        //Constructors
        Vec3f();
        Vec3f(LD x,LD y, LD z);
        Vec3f(const Vec3f& v);
        //Deconstructor
        virtual ~Vec3f();
    public:
        //Operators

        //Operator []
        LD& operator [](int index)
        {
            return _p[index];
        }
        const LD& operator [](int index) const
        {
            return _p[index];
        }
        
        //Operator =
        Vec3f& operator = (const Vec3f& v);

        //Operators +=,-=, *=, /=
        void operator +=(const Vec3f& v);
        void operator +=(LD f);
        void operator -=(const Vec3f& v);
        void operator -=(LD f);
        void operator *=(const Vec3f& v);
        void operator *=(LD f);
        void operator /=(const Vec3f& v);
        void operator /=(LD f);

        //Operators +,-.*,/
        Vec3f operator +(const Vec3f&v) const;
        Vec3f operator +(LD f) const;
        Vec3f operator -(const Vec3f&v) const;
        Vec3f operator -(LD f) const;
		LD operator *(const Vec3f&v) const;
        Vec3f operator *(LD f) const;
        Vec3f operator /(const Vec3f&v) const;
        Vec3f operator /(LD f) const;
		Vec3f operator %(const Vec3f&v) const;

        Vec3f operator -() const;

    public:
        void Normalize();
        LD L2Norm_Sqr();
		LD L2Norm();
     
    public:
        union
        {
            struct
            { LD _p[3]; };
            struct
            { LD x,y,z; };
            struct
            { LD r,g,b; };
        };
        
    };
}

#endif