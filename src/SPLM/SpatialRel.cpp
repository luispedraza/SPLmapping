// SpatialRel.cpp: implementation of the CSpatialRel class.
// Author: Luis Pedraza
//////////////////////////////////////////////////////////////////////

#include "SpatialRel.h"
#include <math.h>	// For math operation

/** Constructor with initial values
 * 	\param xval x value
 * 	\param yval y value
 * 	\param angval angular value
 **/
CSpatialRel::CSpatialRel(float xval, float yval, float angval)
{
	x = xval; y = yval; ang = angval;
}

/** Default constructor **/
CSpatialRel::CSpatialRel()
{
	x = y = ang = 0.0F;
}

CSpatialRel::~CSpatialRel()
{

}

/** Compounding **/
CSpatialRel CSpatialRel::operator+(const CSpatialRel &xjk) const
{
	CSpatialRel xik;
	float sinang = (float)sin(ang);
	float cosang = (float)cos(ang);
	xik.x = xjk.x * cosang - xjk.y * sinang + x;
	xik.y = xjk.x * sinang + xjk.y * cosang + y;
	xik.ang = ang + xjk.ang;
	return xik;
}

/** Inverse relationship **/
CSpatialRel CSpatialRel::operator-() const
{
	CSpatialRel xji;
	float sinang = (float)sin(ang);
	float cosang = (float)cos(ang);
	xji.x = -x * cosang - y * sinang;
	xji.y = x * sinang - y * cosang;
	xji.ang = -ang;
	return xji;
}

CSpatialRel CSpatialRel::operator-(const CSpatialRel &xkj) const
{
	CSpatialRel xsol;
	xsol = (*this) + (-xkj);
	return xsol;
}
