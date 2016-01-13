#include "Vec3f.h"
#include "SimpleObject.h"
#include "Base.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
using namespace std;

using namespace SimpleOBJ;

bool useKDTree = 0;
vector<Object*> objList;
int TcalcInsection, TcalcShadow;
Camera camera;
kdTree *root;


Aim* calcInsection(Light *light, int nowObj)
{
	TcalcInsection -= clock();
	if (useKDTree)
	{
		Aim *tmp = root->findInsect(light);
		TcalcInsection += clock();
		return tmp;
	}

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
			double cosi = -light->Rd * N;
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
			if (T > -EPS)
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


const int dirX[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };
const int dirY[8] = { -1, -1, -1, 0, 0, 1, 1, 1 };
const double sampleX[4] = { 0.125, 0.375, -0.125, -0.375 };
const double sampleY[4] = { 0.375, -0.125, -0.375, 0.125 };
int ID;


class Mat_3
{
public:
	double a[3][4];

	Vec3f operator*(Vec3f B)
	{
		Vec3f C;
		for (int i = 0; i<3; ++i)
		{
			C[i] = a[i][3];
			for (int j = 0; j<3; ++j)
				C[i] += a[i][j] * B[j];
		}
		return C;
	}
};

void initialObj(char *a, Mat_3 mat, int id, RGB color, int colorType, double Kd, double Ks, int Ns, double Ka, double Kss, double Ktt, double Kdd, double refraction, Image *texture, bool islight = 0)
{
	CSimpleObject x;
	x.LoadFromObj(a);

	for (int i = 0; i < x.m_nTriangles; i++)
	{
		objList.push_back(new Plane(mat * x.m_pVertexList[x.m_pTriangleList[i][0]], mat * x.m_pVertexList[x.m_pTriangleList[i][1]], mat * x.m_pVertexList[x.m_pTriangleList[i][2]], ID, color, colorType, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	}
}

int main()
{
	srand(time(0));

	useKDTree = 1;
	ID = 0;
	double Kd = 0, Ks = 0, Ns = 1000, Ka = 0.5, Kss = 0, Ktt = 0, Kdd = 0.5, refraction = 1;
	ID ++;
	//floor
	objList.push_back(new Plane(Vec3f(556.0, 0.0, 0.0), Vec3f(0.0, 0.0, 0.0), Vec3f(0.0, 0.0, 559.2), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(556.0, 0.0, 0.0), Vec3f(0.0, 0.0, 559.2), Vec3f(556.0, 0.0, 559.2), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//ceil
	objList.push_back(new Plane(Vec3f(556.0, 548.8, 0.0), Vec3f(556.0, 548.8, 559.2), Vec3f(0.0, 548.8, 559.2), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(0.0, 548.8, 0.0), Vec3f(556.0, 548.8, 0.0), Vec3f(0.0, 548.8, 559.2), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//back
	objList.push_back(new Plane(Vec3f(556.0, 0.0, 559.2), Vec3f(0.0, 0.0, 559.2), Vec3f(0.0, 548.8, 559.2), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(556.0, 548.8, 559.2), Vec3f(556.0, 0.0, 559.2), Vec3f(0.0, 548.8, 559.2), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//right Green
	objList.push_back(new Plane(Vec3f(0.0, 0.0, 559.2), Vec3f(0.0, 0.0, 0.0), Vec3f(0.0, 548.8, 0.0), 1, RGB(0, 1, 0), ID, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(0.0, 548.8, 559.2), Vec3f(0.0, 0.0, 559.2), Vec3f(0.0, 548.8, 0.0), 1, RGB(0, 1, 0), ID, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//left Red
	//Kss = 0.5, Kdd = 0.5;
	objList.push_back(new Plane(Vec3f(556.0, 0.0, 0.0), Vec3f(556.0, 0.0, 559.2), Vec3f(556.0, 548.8, 559.2), ID, RGB(1, 0, 0), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	objList.push_back(new Plane(Vec3f(556.0, 548.8, 0.0), Vec3f(556.0, 0.0, 0.0), Vec3f(556.0, 548.8, 559.2), ID, RGB(1, 0, 0), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));

	//Ks = 0.05, Ns = 1000, Ka = 0.5, Kss = 0.05, Ktt = 0, Kdd = 0, refraction = 1;
	//ID ++;
	////Short block
	//objList.push_back(new Plane(Vec3f(130.0, 165.0, 65.0), Vec3f(82.0, 165.0, 225.0), Vec3f(240.0, 165.0, 272.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(290.0, 165.0, 114.0), Vec3f(130.0, 165.0, 65.0), Vec3f(240.0, 165.0, 272.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(290.0, 0.0, 114.0), Vec3f(290.0, 165.0, 114.0), Vec3f(240.0, 165.0, 272.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(240.0, 0.0, 272.0), Vec3f(290.0, 0.0, 114.0), Vec3f(240.0, 165.0, 272.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(130.0, 0.0, 65.0), Vec3f(130.0, 165.0, 65.0), Vec3f(290.0, 165.0, 114.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(290.0, 0.0, 114.0), Vec3f(130.0, 0.0, 65.0), Vec3f(290.0, 165.0, 114.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(82.0, 0.0, 225.0), Vec3f(82.0, 165.0, 225.0), Vec3f(130.0, 165.0, 65.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(130.0, 0.0, 65.0), Vec3f(82.0, 0.0, 225.0), Vec3f(130.0, 165.0, 65.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(240.0, 0.0, 272.0), Vec3f(240.0, 165.0, 272.0), Vec3f(82.0, 165.0, 225.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(82.0, 0.0, 225.0), Vec3f(240.0, 0.0, 272.0), Vec3f(82.0, 165.0, 225.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	////Tall block
	//ID = 3;
	//objList.push_back(new Plane(Vec3f(423.0, 330.0, 247.0), Vec3f(265.0, 330.0, 296.0), Vec3f(314.0, 330.0, 456.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(472.0, 330.0, 406.0), Vec3f(423.0, 330.0, 247.0), Vec3f(314.0, 330.0, 456.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(423.0, 0.0, 247.0), Vec3f(423.0, 330.0, 247.0), Vec3f(472.0, 330.0, 406.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(472.0, 0.0, 406.0), Vec3f(423.0, 0.0, 247.0), Vec3f(472.0, 330.0, 406.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(472.0, 0.0, 406.0), Vec3f(472.0, 330.0, 406.0), Vec3f(314.0, 330.0, 456.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(314.0, 0.0, 456.0), Vec3f(472.0, 0.0, 406.0), Vec3f(314.0, 330.0, 456.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(314.0, 0.0, 456.0), Vec3f(314.0, 330.0, 456.0), Vec3f(265.0, 330.0, 296.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(265.0, 0.0, 296.0), Vec3f(314.0, 0.0, 456.0), Vec3f(265.0, 330.0, 296.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(265.0, 0.0, 296.0), Vec3f(265.0, 330.0, 296.0), Vec3f(423.0, 330.0, 247.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
	//objList.push_back(new Plane(Vec3f(423.0, 0.0, 247.0), Vec3f(265.0, 0.0, 296.0), Vec3f(423.0, 330.0, 247.0), ID, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));

	//Blue Ball
	//ID++;
	//Kd = 0.8, Ks = 0.7, Ns = 100, Kss = 0.05, Ktt = 0, refraction = 1;
	//objList.push_back(new Ball(Vec3f(120, 300, 400), 100, ID, RGB(0, 0, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));

	//Tranmissive Ball
	//ID ++;
	//Kd = 0.8, Ks = 0.7, Ns = 100, Kss = 0.01, Ktt = 0.8, refraction = 1.5;
	//objList.push_back(new Ball(Vec3f(278, 274.4, -100), 100, ID, RGB(0, 0, 0), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));

	/*Kd = 0.8, Ks = 0.7, Ns = 100, Kss = 0.2, Ktt = 0, refraction = 1;

	for (int i = 0; i < 50; ++i)
		for (int j = 0; j < 50; ++j)
			for (int k = 0; k < 50; ++k)
			{
				ID++;
				objList.push_back(new Ball(Vec3f(10 + i * 10, 10 + j * 10, 10 + k * 10), 3, ID, RGB(0, 0, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));
			}*/

	/*ID ++;
	char obj[100] = "obj\\kitten.50k.obj";
	Mat_3 mat;
	mat.a[0][0] = 2.5;	mat.a[0][1] = 0;	mat.a[0][2] = 0;	mat.a[0][3] = 400;
	mat.a[1][0] = 0;	mat.a[1][1] = 2.5;	mat.a[1][2] = 0;	mat.a[1][3] = 0;
	mat.a[2][0] = 0;	mat.a[2][1] = 0;	mat.a[2][2] = -2.5;	mat.a[2][3] = 400;
	Kd = 0.8, Ks = 0.8, Ns = 100, Ka = 0.2, Kss = 0.1, Ktt = 0, refraction = 1;
	initialObj(obj, mat, ID, RGB(0.753, 0.753, 0.753), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL);*/

	{

		ID++;
		char obj[100] = "obj\\fixed.perfect.dragon.100K.0.0701.obj";
		Mat_3 mat;
		mat.a[0][0] = -100;	mat.a[0][1] = 0;	mat.a[0][2] = 0;	mat.a[0][3] = 400;
		mat.a[1][0] = 0;	mat.a[1][1] = 100;	mat.a[1][2] = 0;	mat.a[1][3] = 70;
		mat.a[2][0] = 0;	mat.a[2][1] = 0;	mat.a[2][2] = 100;	mat.a[2][3] = 400;
		Kd = 0, Ks = 0, Ns = 100, Ka = 0.5, Kss = 0.1, Kdd = 0.7, refraction = 1;
		initialObj(obj, mat, ID, RGB(0.753, 0.753, 0.753), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL);
	}

	{
		ID++;
		char obj[100] = "obj\\fixed.perfect.dragon.100K.0.0701.obj";
		Mat_3 mat;
		mat.a[0][0] = -100;	mat.a[0][1] = 0;	mat.a[0][2] = 0;	mat.a[0][3] = 150;
		mat.a[1][0] = 0;	mat.a[1][1] = 100;	mat.a[1][2] = 0;	mat.a[1][3] = 70;
		mat.a[2][0] = 0;	mat.a[2][1] = 0;	mat.a[2][2] = 100;	mat.a[2][3] = 200;
		initialObj(obj, mat, ID, RGB(0.753, 0.753, 0.753), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL);
	}
	
	//Light
	ID++;
	Kd = 0, Ks = 0, Ns = 2000, Ka = 0, Kss = 0, Ktt = 1.2, Kdd = 0, refraction = 1;
	double I = 50;
	objList.push_back(new Plane(Vec3f(343.0, 548.79, 227.0), Vec3f(343.0, 548.79, 332.0), Vec3f(213.0, 548.79, 332.0), 5, RGB(I, I, I), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL, true));
	objList.push_back(new Plane(Vec3f(213.0, 548.79, 227.0), Vec3f(343.0, 548.79, 227.0), Vec3f(213.0, 548.79, 332.0), 5, RGB(I, I, I), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL, true));

	//Kd = 0.8, Ks = 0.3, Ns = 100, Ka = 0, Kss = 0.3, Ktt = 0, refraction = 1;
	//objList.push_back(new Ball(Vec3f(278, 279.0, 276.0), 200, 4, RGB(1, 1, 1), 1, Kd, Ks, Ns, Ka, Kss, Ktt, Kdd, refraction, NULL));



	if (useKDTree)
	{
		root = new kdTree();
		for (int i = 0; i < objList.size(); ++i)
			root->objList.push_back(objList[i]);
		root->buildTree(1);
	}
	
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
	for (int i = 0; i < camera.w; ++i)
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
				int T = 128;
				for (int t = 0; t < T; ++t)
				{
					Vec3f R = camera.getPoint(Vec3f(camera.lx + i * camera.dx + camera.dx / 2 + Rand() * camera.dx, camera.ly + j * camera.dy + camera.dy / 2 + Rand() * camera.dy, camera.z));
					R -= camera.viewPoint;
					R.Normalize();
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
		fprintf(output, "%d\n", tt);
		fclose(output);
	}

	cout << TcalcInsection << endl;
	cout << TcalcShadow << endl;
	cout << clock() << endl;
//
//	double SSpara = 0.1;
//#pragma omp parallel for num_threads(16)
//	for (int i = 0; i < camera.w; ++i)
//		for (int j = 0; j < camera.h; ++j)
//			for (int dir = 0; dir < 8; ++dir)
//			{
//				int x = i + dirX[dir], y = j + dirY[dir];
//				if (x >= 0 && x < camera.w && y >= 0 && y < camera.h && (ans[i][j] - ans[x][y]).dist() > SSpara)
//				{
//					ans[i][j] = ans[i][j] * 0.2;
//					for (int c = 0; c < 4; ++c)
//					{
//						Vec3f R = camera.getPoint(Vec3f(camera.lx + i * camera.dx + camera.dx / 2 + camera.dx * sampleX[c], camera.ly + j * camera.dy + camera.dy / 2 + camera.dy * sampleY[c], camera.z));
//						R -= camera.viewPoint;
//						R.Normalize();
//						dist[i][j] = -1;
//						ans[i][j] += rayTracing(new Light(camera.viewPoint, R), 0, dist[i][j], -1, 1, 1) * 0.2;
//				}
//				break;
//			}
//	}
//	FILE *output = fopen("1.txt", "w");
//	fprintf(output, "%d %d\n", camera.h, camera.w);
//	for (int j = camera.h - 1; j >= 0; --j)
//	{
//		for (int i = camera.w - 1; i >= 0; --i)
//		{/*
//		 ans[i][j].r = min(1., ans[i][j].r);
//		 ans[i][j].g = min(1., ans[i][j].g);
//		 ans[i][j].b = min(1., ans[i][j].b);*/
//			fprintf(output, "%d %d %d ", int(255 * min(1., ans[i][j].b)), int(255 * min(1., ans[i][j].g)), int(255 * min(1., ans[i][j].r)));
//		}
//		fprintf(output, "\n");
//	}
//	fclose(output);
//	cout << TcalcInsection << endl;
//	cout << TcalcShadow << endl;
//	cout << clock() << endl;


	return 0;
}