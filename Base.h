#ifndef BASE_H
#define BASE_H

#include "Vec3f.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
using namespace std;

using namespace SimpleOBJ;
const double PI = acos(-1);
const double EPS = 1e-6;

struct RGB
{
	double r, g, b;
	RGB()   {};
	RGB(double r, double g, double b) : r(r), g(g), b(b)  {}

	RGB operator *(double f)
	{
		return RGB(r * f, g * f, b * f);
	}

	RGB operator +(RGB x)
	{
		return RGB(r + x.r, g + x.g, b + x.b);
	}
	RGB operator -(RGB x)
	{
		return RGB(r - x.r, g - x.g, b - x.b);
	}
	RGB operator *(RGB x)
	{
		return RGB(r * x.r, g * x.g, b * x.b);
	}
	RGB* operator +=(RGB x)
	{
		r += x.r, g += x.g, b += x.b;
		return this;
	}
	double dist()
	{
		return abs(r) + abs(g) + abs(b);
	}
};

struct Image
{
	int height, width;
	RGB	**data;
};

class Object;

struct Light //R0 + Rd
{
	Vec3f R0, Rd;
	Light() {}
	Light(Vec3f R0, Vec3f Rd) : R0(R0), Rd(Rd)	{}
};

struct Aim
{
	Object *insectObj;
	double dist;
	double decay;
	Aim() {}
	Aim(Object *obj, double dist, double decay) : insectObj(obj), dist(dist), decay(decay) {}
};

class Object
{
public:
	RGB	color;
	double Kd; //漫反射系数 diffuse (phong)
	double Ks; //Phong BRDF反射系数 specular
	int Ns; //Phong BRDF反射指数 specular
	double Ka; //环境光反射系数 ambient
	double Kss; // 反射系数 reflect
	double Ktt; // 折射系数 transmit
	double Kdd; // Lambertian diffuse
	bool colorType;
	Image *texture;
	Vec3f L, R;
	int id;
	double refraction;
	bool islight;

	Object() {}
	Object(int id, RGB color, int colorType, double Kd, double Ks, int Ns, double Ka, double Kss, double Ktt, double Kdd, double refraction, Image *texture, bool islight) : id(id), color(color), colorType(colorType), Kd(Kd), Ks(Ks), Ns(Ns), Ka(Ka), Kss(Kss), Ktt(Ktt), Kdd(Kdd), refraction(refraction), texture(texture), islight(islight) {}

	virtual double Insect(Light* light) { return NULL; }
	virtual Vec3f calcNormal(Vec3f pos) { return Vec3f(0, 0, 0); }
	virtual RGB getColor(Vec3f pos) { return RGB(0, 0, 0); }
};

double Rand();

struct Source
{
	Vec3f c;
	double dx, dy, dz;
	RGB I;
	Source() {}
	Source(Vec3f c, double dx, double dy, double dz, RGB I) : c(c), dx(dx), dy(dy), dz(dz), I(I) {}

	Vec3f getPos()
	{
		return Vec3f(c.x + Rand() * dx, c.y + Rand() * dy, c.z + Rand() * dz);
	}
	
};

struct Camera
{
	Vec3f viewPoint; //视点位置
	vector<Source> source; //光源位置
	double alpha, beta, theta; //视角度 alpha:[-pi/2,pi/2] beta:[-pi,pi], theta:[-pi,pi]
	RGB backGroundColor;
	double z; //屏幕到视点距离
	double lx, ly; //视点坐标系中的屏幕左下角位置
	double dx, dy; //像素大小
	int w, h; //像素点个数

	Vec3f getPoint(Vec3f p)
	{
		Vec3f q(p.x, p.z * sin(alpha) + p.y * cos(alpha), p.z * cos(alpha) - p.y * sin(alpha));
		return viewPoint + Vec3f(q.x * cos(beta) - q.z * sin(beta), q.y, q.x * sin(beta) + q.z * cos(beta));
		/*Vec3f q(p.x * cos(theta) - p.y * sin(theta), p.x * sin(theta) + p.y * cos(theta), p.z);
		q = Vec3f(q.x, q.z * sin(PI / 2 - alpha) + q.y * cos(PI / 2 - alpha), q.z * cos(PI / 2 - alpha) - q.y * sin(PI / 2 - alpha));
		return viewPoint + Vec3f(q.x * cos(beta - PI / 2) - q.y * sin(beta - PI / 2), q.x * sin(beta - PI / 2) + q.y * cos(beta - PI / 2), q.z);
		*///return viewPoint + Vec3f(now.x * cos(beta) - now.z * sin(beta), now.y, now.x * sin(beta) + now.z * cos(beta));
	}
	void addSource(Vec3f L, Vec3f R, int X, int Y, int Z, RGB I)
	{
		R -= L;
		double dx = R.x / X, dy = R.y / Y, dz = R.z / Z;
		I = I * (1. / (X * Y * Z));
		for (int i = 0; i < X; ++i)
			for (int j = 0; j < Y; ++j)
				for (int k = 0; k < Z; ++k)
				{
					source.push_back(Source(L + Vec3f(i * dx + dx / 2, j * dy + dy / 2, k * dz + dz / 2), dx, dy, dz, I));
				}
	}
};


class Ball:public Object
{
public:
	double r;
	Vec3f center;

	Ball() {}
	Ball(Vec3f center, double r, int id, RGB color, int colorType,  double Kd, double Ks, int Ns,double Ka, double Kss, double Ktt, double Kdd, double refraction, Image *texture, bool islight = 0) : center(center), r(r), Object(id, color, colorType, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, texture, islight)	
	{
		L = Vec3f(center.x - r, center.y - r, center.z - r);
		R = Vec3f(center.x + r, center.y + r, center.z + r);
	}

	double Insect(Light *light)
	{
		Vec3f l = center - light->R0;
		double p = l * light->Rd;
		double d = l.L2Norm_Sqr() - p * p;
		if (d > r * r - EPS) return 1e99;
		double s = sqrt(r * r - d);
		if (p + s < EPS)	return 1e99;
		if (l.L2Norm_Sqr() > r * r + EPS)
			return p - s;
		else
			return p + s;
	}
	Vec3f calcNormal(Vec3f pos)
	{
		Vec3f x = pos -  center;
		x.Normalize();
		return x;
	}
	RGB getColor(Vec3f pos) { return color; }
};

class Plane:public Object
{
public:
	Vec3f p[3], N, u, v, i, j;
	double uu, vv, uv, D;
	double S;

	Plane() {}
	Plane(Vec3f p1, Vec3f p2, Vec3f p3, int id, RGB color, int colorType, double Kd, double Ks, int Ns, double Ka, double Kss, double Ktt, double Kdd, double refraction, Image *texture, bool islight = 0) : Object(id, color, colorType, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, texture, islight)
	{
		p[0] = p1, p[1] = p2, p[2] = p3;
		u = p[1] - p[0], v = p[2] - p[0];
		uu = u * u, vv = v * v;
		uv = u * v;
		D = uv * uv - uu * vv;
		i = (v * uv - u * vv) / D;
		j = (u * uv - v * uu) / D;
		N = u % v;
		N.Normalize();
		for (int i = 0; i< 3; ++i)
		{
			L[i] = min(p[0][i], min(p[1][i], p[2][i]));
			R[i] = max(p[0][i], max(p[1][i], p[2][i])) + 1.0;
		}
	}

	double Insect(Light *light)
	{
		double k = light->Rd * N;
		if (abs(k) < EPS) return 1e99;
		Vec3f P = light->R0 - p[0];
		double t = -(P * N) / k;
		if (t < EPS) return 1e99;
		P += light->Rd * t;
		double x = P * i, y = P * j;
		if (x + y < 1 + EPS && x > -EPS && y > -EPS)
			return t;
		return 1e99;
	}
	Vec3f calcNormal(Vec3f pos) { return N; };
	RGB getColor(Vec3f pos) { return color;  };
};

Aim *min(Aim *a, Aim *b);

class kdTree
{
public:
	vector<Object*> objList;
	Vec3f L, R;
	int objNum;
	kdTree *son[2];

	kdTree()
	{
		objList.clear();
		L.x = L.y = L.z = 1e99;
		R.x = R.y = R.z = -1e99;
		objNum = 0;
		son[0] = son[1] = NULL;
	}

	void buildTree(int dep);

	double ifInsect(Light *light);

	Aim* calcInsection(Light *light);

	Aim* findInsect(Light* light);

};


#endif