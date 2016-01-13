#include "Base.h"

double Rand()
{
	return 1. * rand() / RAND_MAX - 0.5;
}

Aim *min(Aim *a, Aim *b)
{
	if (!a)	return b;
	if (!b)	return a;
	if (a->dist < b->dist)	return a;
	return b;
}
const int KDdep = 16;

bool cmp0(Object *x, Object *y)
{
	return x->R[0] < y->R[0];
}
bool cmp1(Object *x, Object *y)
{
	return x->R[1] < y->R[1];
}
bool cmp2(Object *x, Object *y)
{
	return x->R[2] < y->R[2];
}


void kdTree::buildTree(int dep)
{
	int type = dep % 3;
	objNum = objList.size();
	for (int i = 0; i< objNum; ++i)
		for (int j = 0; j< 3; ++j)
		{
			L[j] = min(L[j], objList[i]->L[j]);
			R[j] = max(R[j], objList[i]->R[j]);
		}

	if (objNum <= 1 || dep > KDdep)
		return;

	if (type == 0) sort(objList.begin(), objList.end(), cmp0);
	if (type == 1) sort(objList.begin(), objList.end(), cmp1);
	if (type == 2) sort(objList.begin(), objList.end(), cmp2);

	son[0] = new kdTree();
	son[1] = new kdTree();
	for (int i = 0; i < objNum; ++i)
	{
		if (i < objNum / 2)
			son[0]->objList.push_back(objList[i]);
		else
			son[1]->objList.push_back(objList[i]);
	}

	son[0]->buildTree(dep + 1);
	son[1]->buildTree(dep + 1);
}

double kdTree::ifInsect(Light *light)
{
	double tmin = 0;
	double tmax = 1e10;
	for (int i = 0; i < 3; ++i)
	{
		if (fabs(light->Rd[i]) < EPS) continue;
		double t1 = (L[i] - light->R0[i]) / light->Rd[i];
		double t2 = (R[i] - light->R0[i]) / light->Rd[i];
		if (t1 > t2)	swap(t1, t2);
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;
	}
	if (tmin > tmax - EPS) return -1e10;
	return tmin;
}


Aim* kdTree::calcInsection(Light *light)
{
	Aim *res = new Aim(NULL, 1e10, 0);
	for (int i = 0; i < objNum; ++i)
	{
		double d = objList[i]->Insect(light);
		if (d < res->dist)
		{
			res->insectObj = objList[i];
			res->dist = d;
			res->decay = 0.8 * objList[i]->Ktt;
		}
	}
	if (res->dist < 1e9)
		return res;
	delete res;
	return NULL;
}



Aim* kdTree::findInsect(Light* light)
{
	if (objNum <= 2)
		return calcInsection(light);
	if (son[0] == NULL || son[1] == NULL)
		return calcInsection(light);

	Aim* res = NULL;

	double ll = son[0]->ifInsect(light);
	double rr = son[1]->ifInsect(light);
	int fir = 0;
	if (ll > rr)
	{
		swap(ll, rr);
		fir = 1;
	}
	if (ll > -1e9) res = son[fir]->findInsect(light);
	if (res != NULL && res->dist < rr) return res;
	if (rr > -1e9) res = min(res, son[fir ^ 1]->findInsect(light));
	return res;
}

