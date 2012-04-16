// Map.cpp: implementation of the CMap class.
// Author: Luis Pedraza
//////////////////////////////////////////////////////////////////////

#include "Map.h"
#include <math.h>

#ifndef PI
	#define PI 3.14159265358979
#endif
#ifndef DEG2RAD
	#define DEG2RAD 0.0174532925199432
#endif

/** Constructor for a CMapSP class **/
CMapSP::CMapSP()
{
	m_nCP = 0;		// initial number of control points
	m_nSP = 0;		// initial number of splines
	next = NULL;
	P.ReSize(3);	// Size of initial covariance matrix
	P = 0.0F;		// Covariance init.	
	P.element(0,0) = 0.01*0.01;
	P.element(1,1) = P.element(0,0);
	P.element(2,2) = 0.2 * 0.2;
	x.ReSize(3);
	x = 0.0F;		// State init.
	idMap = 0;
}

CMapSP::~CMapSP()
{

}

/** Adds a new spline to the map **/
int CMapSP::AddFeature(CBspline* spline)
{
	m_Splines.push_back(spline);	// new spline
	m_nSP++;						// inc # splines
	m_nCP += spline->m_iNumber;		// inc # CP
	return 0;
}


/** Clears the map **/
int CMapSP::Clear()
{
	for (int i = 0; i < m_nSP; i++) delete m_Splines[i];
	m_Splines.clear();
	m_nSP = 0;
	return 0;
}

/** Defines robot's pose and orientation **/
int CMapSP::SetRobotPose(PoseStruct& pose)
{
	x.element(0) = pose.x;
	x.element(1) = pose.y;
	x.element(2) = pose.ang;
	RobotPose = pose;
	return 0;
}

/** Retrieves robot's pose and orientation **/
int CMapSP::GetRobotPose(PoseStruct& pose)
{
	pose.x = x.element(0);
	pose.y = x.element(1);
	pose.ang = x.element(2);
	return 0;
}

/** Function for saving a map to a file **/
int CMapSP::Save(std::ofstream& file)
{
	file << "Number_of_objects: " << m_nSP << std::endl;
	int i, j;
	for (i = 0; i < m_nSP; i++)	// for all splines in the map...
	{
		file << "SPLINE_"<< i << ": " << m_Splines[i]->m_iNumber << std::endl;
		file << "T: ";
		int limite = m_Splines[i]->m_iNumber;
		for (j = 0; j < limite + 4; j++)
		{
			file << m_Splines[i]->m_Knots[j];
			if (j==limite+3)
				file << std::endl;
			else
				file << ", ";
		}
		file << "X: ";
		for (j = 0; j < limite; j++)
		{
			file << m_Splines[i]->m_Bx[j];
			if (j==limite-1)
				file << std::endl;
			else
				file << ", ";
		}
		file << "Y: ";
		for (j = 0; j < limite; j++)
		{
			file << m_Splines[i]->m_By[j];
			if (j==limite-1)
				file << std::endl;
			else
				file << ", ";
		}
	}
	return 0;
}

/** Function for loading a map from a file **/
int CMapSP::Load(std::ifstream& file)
{
	Clear();
	int nmaps = 0;		// number of maps
	int numbersp = 0;	// number of objects
	int numbercp = 0;	// number of control points of an object
	char str[20];
	file.get(str, 20, ' ');
	file >> numbersp;	
	int i, j;
	for (i = 0; i < numbersp; i++)
	{
		CBspline* spline = new CBspline;		
		file.get(str, 20, ' ');
		file >> numbercp;
		spline->m_iNumber = numbercp;
		// T
		float value;
		for (j = 0; j < numbercp+4; j++)
		{
			file.get(str, 20, ' ');
			file >> value;
			spline->m_Knots.push_back(value);
			if (j==0) spline->m_fRange[0] = value;
			if (j==numbercp+3) spline->m_fRange[1] = value;
		}
		// X
		for (j = 0; j < numbercp; j++)
		{			
			file.get(str, 20, ' ');
			file >> value;
			spline->m_Bx.push_back(value);
		}
		// Y
		for (j = 0; j < numbercp; j++)
		{		
			file.get(str, 20, ' ');
			file >> value;
			spline->m_By.push_back(value);
		}
		AddFeature(spline);
	}
	
	return 0;
}

int CMapSP::SetRobotPose(float &xx, float &yy, float &ang)
{
	x.element(0) = xx;
	x.element(1) = yy;
	x.element(2) = ang;
	RobotPose.x = xx;
	RobotPose.y = yy;
	RobotPose.ang = ang;
	return 0;
}

int CMapSP::GetRobotPose(float &xx, float &yy, float &ang)
{
	xx = x.element(0);
	yy = x.element(1);
	ang = x.element(2);
	return 0;
}

/** Adapted from http://www.cas.kth.se/toolbox/ (matlab toolbox)**/
int CMapSP::ProbEllipseParams(float sxx, float syy, float sxy, float &a, float &b, float &ang, float conf)
{
	a = sqrt(0.5*(sxx+syy+sqrt((sxx-syy)*(sxx-syy)+4*sxy*sxy)));	// always greater
	b = sqrt(0.5*(sxx+syy-sqrt((sxx-syy)*(sxx-syy)+4*sxy*sxy)));	// always smaller
	// Remove imaginary parts in case of neg. definite C
	// if ~isreal(a), a = real(a); end;
	//if ~isreal(b), b = real(b); end;
	// Scaling in order to reflect specified probability
	//a = a*2.448;
	//b = b*2.448;
	a = a*10;
	b = b*10;
	// Look where the greater half axis belongs to
	if (sxx < syy)
	{ float swap = a; a = b; b = swap;};
	// Calculate inclination (numerically stable)
	if (sxx != syy)
		ang = 0.5*atan(2*sxy/(sxx-syy));	
	else if (sxy == 0)
		ang = 0;     // angle doesn't matter 
	else if (sxy > 0)
		ang =  PI/4;
	else if (sxy < 0)
		ang = -PI/4;	
	return 0;
}

/** From map reference frame to robot reference frame **/
int CMapSP::Global2Robot(float xgf, float ygf, float &xrf, float &yrf)
{
	float xr = RobotPose.x;
	float yr = RobotPose.y;
	float thr = RobotPose.ang;
	float cos_a = cos(thr);
	float sen_a = sin(thr);
	float xi = -xr*cos_a-yr*sen_a;
	float yi = xr*sen_a-yr*cos_a;
	float thi =- thr;
	float cos_ai = cos(thi);
	float sen_ai = sin(thi);
	xrf = xi + xgf*cos_ai - ygf*sen_ai;
	yrf = yi + xgf*sen_ai + ygf*cos_ai;					
	return 0;	
}
