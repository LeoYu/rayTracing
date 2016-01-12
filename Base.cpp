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


double Rand()
{
	int x = rand();
	x = x * rand() + rand();
	x = x * rand() + rand();
	x = x * rand() + rand();
	x = x * rand() + rand();
	return abs(x) % 1000000 / 1000000. - 0.5;
}

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
	Ball(Vec3f center, double r, int id, RGB color, int colorType,  double Kd, double Ks, int Ns,double Ka, double Kss, double Ktt, double Kdd, double refraction, Image *texture, bool islight = 0) : center(center), r(r), Object(id, color, colorType, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, texture, islight)	{}

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

Camera camera;

vector<Object*> objList;
int TcalcInsection, TcalcShadow;

Aim *min(Aim *a, Aim *b)
{
	if (!a)	return b;
	if (!b)	return a;
	if (a->dist < b->dist)	return a;
	return b;
}

Aim* calcInsection(Light *light, int nowObj)
{
	TcalcInsection -= clock();
	Aim *res = new Aim(NULL, 1e10, 0);
	for (int i = 0; i < objList.size(); ++i)
	{
		double d = objList[i]->Insect(light);
		if (d < res->dist)
		{
			res->insectObj = objList[i];
			res->dist = d;
			res->decay = 0.8 * objList[i]->Ktt;
		}
	}
	TcalcInsection += clock();
	if (res->dist < 1e9)
		return res;
	delete res;
	return NULL;
}

double calcShadow(SimpleOBJ::Vec3f sPos, SimpleOBJ::Vec3f tPos)
{
	TcalcShadow -= clock();
	Light l(sPos, tPos - sPos);
	double threshold = sqrt(l.Rd.L2Norm_Sqr());
	l.Rd.Normalize();

	double res = 1.0;

	for (int i = 0; i< objList.size(); ++i)
	{
		double d = objList[i]->Insect(&l);
		if (d < threshold - EPS)
			res *= 0.8 * objList[i]->Ktt;
		if (res < 0.001)
		{
			TcalcShadow += clock();
			return res;
		}
	}

	TcalcShadow += clock();
	return res;

}

RGB rayTracing(Light *light, int dep, double &dist, int nowObj, double decayRate, double nowRefraction)
{
	Aim* aim = calcInsection(light, nowObj);

	if (aim == NULL)
		return RGB(0, 0, 0);
	if (dep == 0)
		dist = aim->dist;

	Vec3f now;
	Object* insectObj = aim->insectObj;

	if (insectObj->islight)	return insectObj->color;
	now = light->R0 + light->Rd * aim->dist;
	RGB res = camera.backGroundColor * insectObj->Ka;
	delete aim;

	Vec3f N = insectObj->calcNormal(now);
	Vec3f V = light->Rd;// camera.viewPoint - now; V.Normalize();
	if (V * N > 0)
		N = -N;

	if ((insectObj->Kd + insectObj->Ks) * decayRate > 0.001)
	{
		for (int i = 0; i < camera.source.size(); ++i)
		{
			Vec3f source = camera.source[i].getPos();
			Vec3f L = source - now;
			L.Normalize();
			double decay = calcShadow(now, source);

			Vec3f H = L + V;
			H.Normalize();
			double D = decay * (insectObj->Kd * abs(L * N)) /** abs(V * N)*/;
			double S = decay * (insectObj->Ks * pow(abs(H * N), insectObj->Ns));
			res += insectObj->getColor(now) * camera.source[i].I * D + camera.source[i].I * S;
		}
	}

	if (decayRate > 0.01 && dep < 5)
	{
		RGB Ip;
		if (decayRate * insectObj->Kss > 1e-3) // reflection
		{
			Light newlight(now, light->Rd - N * 2 * (light->Rd * N));
			newlight.Rd.Normalize();
			Ip = rayTracing(&newlight, dep + 1, dist, nowObj, decayRate * insectObj->Kss, nowRefraction);
			res += Ip * insectObj->Kss;
		}
		if (decayRate * insectObj->Ktt > 1e-3) // transmit
		{
			double cosi = - light->Rd * N;
			if (cosi < 0)
			{
				cosi = -cosi;
				N = -N;
			}
			double rate, newRefraction;
			int newObj;
			if (insectObj->id != nowObj)
			{
				if (nowObj != -1)
					rate = nowRefraction / insectObj->refraction;
				else
					rate = 1.0 / insectObj->refraction;
				newObj = insectObj->id;
				newRefraction = insectObj->refraction;
			}
			else
			{
				rate = nowRefraction;
				newObj = -1;
				newRefraction = 1.0;
			}
			double T = 1.0 - rate * rate * (1.0 - cosi * cosi);
			if (T > - EPS)
			{
				double cosT = sqrt(T);
				Light newlight(now, light->Rd * rate + N * (rate * cosi - cosT));
				newlight.Rd.Normalize();
				Ip = rayTracing(&newlight, dep + 1, dist, newObj, decayRate * insectObj->Ktt, newRefraction);
				res += Ip * insectObj->Ktt;
			}
			else
			{
				Light nlight(now, light->Rd - N * 2 * (light->Rd * N));
				nlight.Rd.Normalize();
				Ip = rayTracing(&nlight, dep + 1, dist, nowObj, decayRate * insectObj->Ktt, nowRefraction);
				res += Ip * insectObj->Ktt;
			}
		}
		if (decayRate * insectObj->Kdd > 1e-3) // diffus
		{
			double alpha = sqrt(Rand() + 0.5) * PI / 2, beta = Rand() * PI * 2;
			Vec3f I(0, N.z, -N.y);
			if (abs(N.x) > 0.57)	I = Vec3f(N.y, -N.x, 0);
			I.Normalize();
			Vec3f J = N % I;
			Light newlight(now, (I * cos(beta) + J * sin(beta)) * sin(alpha) + N * cos(alpha));
			Ip = rayTracing(&newlight, dep + 1, dist, nowObj, decayRate * insectObj->Kdd, nowRefraction);
			res += Ip * insectObj->color * insectObj->Kdd;
		}
	}

	//res.r = min(1., res.r);
	//res.g = min(1., res.g);
	//res.b = min(1., res.b);

	return res;
}


const int dirX[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
const int dirY[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
const double sampleX[4] = { 0.125, 0.375, -0.125, -0.375 };
const double sampleY[4] = { 0.375, -0.125, -0.375, 0.125 };

int main()
{
	srand(time(0));

	double Kd = 0, Ks = 0, Ns = 1000, Ka = 0.5, Kss = 0, Ktt = 0, Kdd = 0.5, refraction = 1;
	//floor
	objList.push_back(new Plane(Vec3f(556.0, 0.0, 0.0), Vec3f(0.0, 0.0, 0.0), Vec3f(0.0, 0.0, 559.2), 1, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(556.0, 0.0, 0.0), Vec3f(0.0, 0.0, 559.2), Vec3f(556.0, 0.0, 559.2), 1, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//ceil
	objList.push_back(new Plane(Vec3f(556.0, 548.8, 0.0), Vec3f(556.0, 548.8, 559.2), Vec3f(0.0, 548.8, 559.2), 1, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(0.0, 548.8, 0.0), Vec3f(556.0, 548.8, 0.0), Vec3f(0.0, 548.8, 559.2), 1, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//back
	objList.push_back(new Plane(Vec3f(556.0, 0.0, 559.2), Vec3f(0.0, 0.0, 559.2), Vec3f(0.0, 548.8, 559.2), 1, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(556.0, 548.8, 559.2), Vec3f(556.0, 0.0, 559.2), Vec3f(0.0, 548.8, 559.2), 1, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//right Green
	objList.push_back(new Plane(Vec3f(0.0, 0.0, 559.2), Vec3f(0.0, 0.0, 0.0), Vec3f(0.0, 548.8, 0.0), 1, RGB(0, 1, 0), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(0.0, 548.8, 559.2), Vec3f(0.0, 0.0, 559.2), Vec3f(0.0, 548.8, 0.0), 1, RGB(0, 1, 0), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//left Red
	//Kss = 0.5, Kdd = 0.5;
	objList.push_back(new Plane(Vec3f(556.0, 0.0, 0.0), Vec3f(556.0, 0.0, 559.2), Vec3f(556.0, 548.8, 559.2), 1, RGB(1, 0, 0), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(556.0, 548.8, 0.0), Vec3f(556.0, 0.0, 0.0), Vec3f(556.0, 548.8, 559.2), 1, RGB(1, 0, 0), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));

	Ks = 0, Ns = 1000, Ka = 0.5, Kss = 0, Ktt = 0, Kdd = 0.7, refraction = 1;
	//Short block
	objList.push_back(new Plane(Vec3f(130.0, 165.0, 65.0), Vec3f(82.0, 165.0, 225.0), Vec3f(240.0, 165.0, 272.0), 2, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(290.0, 165.0, 114.0), Vec3f(130.0, 165.0, 65.0), Vec3f(240.0, 165.0, 272.0), 2, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(290.0, 0.0, 114.0), Vec3f(290.0, 165.0, 114.0), Vec3f(240.0, 165.0, 272.0), 2, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(240.0, 0.0, 272.0), Vec3f(290.0, 0.0, 114.0), Vec3f(240.0, 165.0, 272.0), 2, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(130.0, 0.0, 65.0), Vec3f(130.0, 165.0, 65.0), Vec3f(290.0, 165.0, 114.0), 2, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(290.0, 0.0, 114.0), Vec3f(130.0, 0.0, 65.0), Vec3f(290.0, 165.0, 114.0), 2, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(82.0, 0.0, 225.0), Vec3f(82.0, 165.0, 225.0), Vec3f(130.0, 165.0, 65.0), 2, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(130.0, 0.0, 65.0), Vec3f(82.0, 0.0, 225.0), Vec3f(130.0, 165.0, 65.0), 2, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(240.0, 0.0, 272.0), Vec3f(240.0, 165.0, 272.0), Vec3f(82.0, 165.0, 225.0), 2, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(82.0, 0.0, 225.0), Vec3f(240.0, 0.0, 272.0), Vec3f(82.0, 165.0, 225.0), 2, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//Tall block
	objList.push_back(new Plane(Vec3f(423.0, 330.0, 247.0), Vec3f(265.0, 330.0, 296.0), Vec3f(314.0, 330.0, 456.0), 3, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(472.0, 330.0, 406.0), Vec3f(423.0, 330.0, 247.0), Vec3f(314.0, 330.0, 456.0), 3, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(423.0, 0.0, 247.0), Vec3f(423.0, 330.0, 247.0), Vec3f(472.0, 330.0, 406.0), 3, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(472.0, 0.0, 406.0), Vec3f(423.0, 0.0, 247.0), Vec3f(472.0, 330.0, 406.0), 3, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(472.0, 0.0, 406.0), Vec3f(472.0, 330.0, 406.0), Vec3f(314.0, 330.0, 456.0), 3, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(314.0, 0.0, 456.0), Vec3f(472.0, 0.0, 406.0), Vec3f(314.0, 330.0, 456.0), 3, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(314.0, 0.0, 456.0), Vec3f(314.0, 330.0, 456.0), Vec3f(265.0, 330.0, 296.0), 3, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(265.0, 0.0, 296.0), Vec3f(314.0, 0.0, 456.0), Vec3f(265.0, 330.0, 296.0), 3, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(265.0, 0.0, 296.0), Vec3f(265.0, 330.0, 296.0), Vec3f(423.0, 330.0, 247.0), 3, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(423.0, 0.0, 247.0), Vec3f(265.0, 0.0, 296.0), Vec3f(423.0, 330.0, 247.0), 3, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//Blue Ball
	//Kd = 0.8, Ks = 0.7, Ns = 100, Kss = 0.05, Ktt = 0, refraction = 1;
	//objList.push_back(new Ball(Vec3f(120, 300, 400), 100, 4, RGB(0, 0, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//Tranmissive Ball
	//Kd = 0.8, Ks = 0.7, Ns = 100, Kss = 0.01, Ktt = 0.8, refraction = 1.5;
	//objList.push_back(new Ball(Vec3f(278, 274.4, -100), 100, 4, RGB(0, 0, 0), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//Light
	Kd = 0, Ks = 0, Ns = 2000, Ka = 0, Kss = 0, Ktt = 0, Kdd = 0, refraction = 1;
	double I = 70;
	objList.push_back(new Plane(Vec3f(343.0, 548.79, 227.0), Vec3f(343.0, 548.79, 332.0), Vec3f(213.0, 548.79, 332.0), 5, RGB(I, I, I), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL, true));
	objList.push_back(new Plane(Vec3f(213.0, 548.79, 227.0), Vec3f(343.0, 548.79, 227.0), Vec3f(213.0, 548.79, 332.0), 5, RGB(I, I, I), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL, true));

	//Kd = 0.8, Ks = 0.3, Ns = 100, Ka = 0, Kss = 0.3, Ktt = 0, refraction = 1;
	//objList.push_back(new Ball(Vec3f(278, 279.0, 276.0), 200, 4, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));

	camera.viewPoint = Vec3f(278, 273, -800);
	camera.alpha = 0;
	camera.beta = 0;
	camera.theta = 0;
	camera.backGroundColor = RGB(0.2, 0.2, 0.2);
	double X = 0.025, Y = 0.025;
	camera.w = 512, camera.h = 512;
	camera.dx = X / camera.w, camera.dy = Y / camera.h;
	camera.lx = -X / 2, camera.ly = -Y / 2;
	camera.z = 0.035;

	/*343.0 548.8 227.0
	343.0 548.8 332.0
	213.0 548.8 332.0
	213.0 548.8 227.0*/

	camera.addSource(Vec3f(213.0, 548.8, 227.0), Vec3f(343.0, 548.8, 332.0), 4, 1, 4, RGB(1, 1, 1));
	//camera.source.push_back(Vec3f(0.5, 0.5, 1));
	//camera.source.push_back(Vec3f(0.5, 0.5, 1));

	RGB **ans = new RGB*[camera.w];
	double **dist = new double*[camera.w];
	for (int i = 0; i < camera.w; ++ i)
	{
		ans[i] = new RGB[camera.h];
		dist[i] = new double[camera.h];
		for (int j = 0; j < camera.h; ++j)
		{
			dist[i][j] = -1;
			ans[i][j] = RGB(0, 0, 0);
		}
	}

	TcalcShadow = 0;
	TcalcInsection = 0;

	int TT = 1000;
	
	for (int tt = 0; tt < TT; ++tt)
	{
		cout << tt << endl;
		cout << clock() << endl;
#pragma omp parallel for num_threads(16)
		for (int i = 0; i < camera.w; ++i)
		{
			for (int j = 0; j < camera.h; ++j)
			{
				Vec3f R = camera.getPoint(Vec3f(camera.lx + i * camera.dx + camera.dx / 2, camera.ly + j * camera.dy + camera.dy / 2, camera.z));
				R -= camera.viewPoint;
				R.Normalize();
				int T = 128;
				for (int t = 0; t < T; ++t)
				{
					Light *light = new Light(camera.viewPoint, R);
					ans[i][j] += rayTracing(light, 0, dist[i][j], -1, 1, 1) * (1. / T);
					delete light;
				}
			}
		}
		FILE *output = fopen("1.txt", "w");
		fprintf(output, "%d %d\n", camera.h, camera.w);
		for (int j = camera.h - 1; j >= 0; --j)
		{
			for (int i = camera.w - 1; i >= 0; --i)
			{/*
				ans[i][j].r = min(1., ans[i][j].r);
				ans[i][j].g = min(1., ans[i][j].g);
				ans[i][j].b = min(1., ans[i][j].b);*/
				fprintf(output, "%d %d %d ", int(255 * min(1., ans[i][j].b / (tt + 1))), int(255 * min(1., ans[i][j].g / (tt + 1))), int(255 * min(1., ans[i][j].r / (tt + 1))));
			}
			fprintf(output, "\n");
		}
		fclose(output);
	}

	cout << TcalcInsection << endl;
	cout << TcalcShadow << endl;
	cout << clock() << endl;

	/*double SSpara = 0.02;
	for (int i = 0; i < camera.w; ++i)
		for (int j = 0; j < camera.h; ++j)
			for (int dir = 0; dir < 8; ++dir)
			{
				int x = i + dirX[dir], y = j + dirY[dir];
				if (x >= 0 && x < camera.w && y >= 0 && y < camera.h && (ans[i][j] - ans[x][y]).dist() > SSpara)
				{
					ans[i][j] = ans[i][j] * 0.2;
					for (int c = 0; c < 4; ++c)
					{
						Vec3f R = camera.getPoint(Vec3f(camera.lx + i * camera.dx + camera.dx / 2 + camera.dx * sampleX[c], camera.ly + j * camera.dy + camera.dy / 2 + camera.dy * sampleY[c], camera.z));
						R -= camera.viewPoint;
						R.Normalize();
						dist[i][j] = -1;
						ans[i][j] += rayTracing(new Light(camera.viewPoint, R), 0, dist[i][j], -1, 1, 1) * 0.2;
					}
					break;
				}
			}
	cout << TcalcInsection << endl;
	cout << TcalcShadow << endl;
	cout << clock() << endl;*/


	return 0;
}