// UncertainRel.cpp: implementation of the CUncertainRel class.
// Author: Luis Pedraza
//////////////////////////////////////////////////////////////////////
#include "UncertainRel.h"
#include <math.h>

using namespace NEWMAT;

CUncertainRel Compound(CUncertainRel &xij, CUncertainRel &xjk, NEWMAT::Matrix &J1, NEWMAT::Matrix &J2)
{
	CUncertainRel xik;
	NEWMAT::Real sinang = (Real)sin(xij.ang);
	NEWMAT::Real cosang = (Real)cos(xij.ang);
	xik.x = xjk.x * cosang - xjk.y * sinang + xij.x;
	xik.y = xjk.x * sinang + xjk.y * cosang + xij.y;
	xik.ang = xij.ang + xjk.ang;
	J1.ReSize(3,3); J2.ReSize(3,3);
	J1 << 1 << 0 << -(xik.y - xij.y)
		<< 0 << 1 << (xik.x - xij.x)
		<< 0 << 0 << 1;
	J2 << cosang << -sinang << 0
		<< sinang << cosang << 0
		<< 0 << 0 << 1;
	xik.Sigma << J1 * xij.Sigma * J1.t() + J2 * xjk.Sigma * J2.t();
	return xik;
}

/** Default constructor **/
CUncertainRel::CUncertainRel()
{
	Sigma.ReSize(3);
	Sigma = 0.0F;
	x = y = ang = 0.0f;
}

CUncertainRel::~CUncertainRel()
{

}

CUncertainRel CUncertainRel::operator+(const CUncertainRel &xjk) const
{
	CUncertainRel xik;
	Real sinang = (Real)sin(ang);
	Real cosang = (Real)cos(ang);
	xik.x = xjk.x * cosang - xjk.y * sinang + x;
	xik.y = xjk.x * sinang + xjk.y * cosang + y;
	xik.ang = ang + xjk.ang;
	NEWMAT::Matrix J1(3,3);
	NEWMAT::Matrix J2(3,3);
	J1 << 1 << 0 << -(xik.y - y)
		<< 0 << 1 << (xik.x - x)
		<< 0 << 0 << 1;
	J2 << cosang << -sinang << 0
		<< sinang << cosang << 0
		<< 0 << 0 << 1;
	xik.Sigma << J1 * Sigma * J1.t() + J2 * xjk.Sigma * J2.t();
	return xik;
}

CUncertainRel CUncertainRel::operator-()const
{
	CUncertainRel xji;
	Real sinang = (Real)sin(ang);
	Real cosang = (Real)cos(ang);
	xji.x = -x * cosang -y * sinang;
	xji.y = x * sinang - y * cosang;
	xji.ang = -ang;
	NEWMAT::Matrix J(3,3);
	J << -cosang << -sinang << xji.y
		<< sinang << -cosang << -xji.x
		<< 0 << 0 << -1;
	return xji;
}

CUncertainRel CUncertainRel::operator-(const CUncertainRel &xkj) const
{
	CUncertainRel xsol;
	xsol = *this + (-xkj);
	return xsol;
}
