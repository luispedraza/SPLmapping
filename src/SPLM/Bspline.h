// Bspline.h: interface for the CBspline class.
// Author: Luis Pedraza
//////////////////////////////////////////////////////////////////////

#if !defined(BSPLINE_H_)
#define BSPLINE_H_

#include <vector>
#include "Point.h"
#include <newmat/newmat.h>
  #include <iostream>
   #include <iomanip>
   #include <newmat/newmatio.h>

#ifdef use_namespace
	using namespace NEWMAT;              // access NEWMAT namespace
#endif

/**
	Basic B-Spline curve class
**/

#define NEAREST_TOL	0.001	///< Default tolerance for nearest point calculation
#define NEAREST_DIV	3		///< Spline subdivision for nearest point calculation

class CBspline;
/** Aux structure for a  link with another spline **/
struct Link
{
	CBspline* spline;
	float Keys[4];	// ini_o, fin_o, ini_m, fin_m
};

class CBspline
{
public:
	bool m_bCPderiv;			///< Indicates whether CP of derivatives need to be calculated or not (performance variable)
	bool m_bCPderiv2;			///< Indicates whether CP of second derivatives need to be calculated or not (performance variable)
	std::vector<float> tempt;

	int Rotate(float th);						///< Spline rotation
	int Translate(float x, float y);			///< Spline translation
	int TransRot(float x, float y, float th);	///< Spline translation and rotation
	bool isnew;			///< The spline has just been added to the map
	bool isextended;	///< The spline has been extended
	static float nearest_tol;	///< Tolerance for nearest point detection
	int Fit(const std::vector<float> *x, const std::vector<float> *y, std::vector<float> *param = NULL, float spacing = 2.0f, int method = 0);
	int Fit(const std::vector<Point>* data, std::vector<float>* param = NULL, float spacing = 2.0f, int method = 0);
	int EvalD2(float t, float* x, float* y);///< Second derivative evaluation
	int Curvature(float t, float& c);		///< CUrvature evaluation
	int EvalD(float t, float* x, float* y);	///< First derivative evaluation
	float GetLength(void);	///< Retrievecs the total length of the Bspline curve
	int InitUnifKnots2(float tmin, float tmax, float spacing); 
	int Nearest(float x, float y, float& ts, float& xs, float& ys, float& d);
	int Eval(float t, float* x, float* y);
	std::vector<float> m_By;	///< Control points - y coord
	std::vector<float> m_Bx;	///< Control points - x coord
	std::vector<float> m_Knots;	///< The knots vector
	int m_iNumber;				///< Number of control points
	int InitUnifKnots(float tmin, float tmax, int numberCP);
	float Basis(float t, int i, int k);
	float BasisD(float t, int i, int k);
	float BasisD2(float t, int i, int k);
	float m_fRange[2];			///< Range interval of definition for the B-spline
	
	int m_iOrder;	///< Order of the B-spline
	CBspline();
	virtual ~CBspline();
	NEWMAT::Matrix* PHI;	// FITTING MATRIX
private:
	std::vector<float> m_KnotsD;	///< The knots vector
	std::vector<float> m_KnotsD2;	///< The knots vector
	std::vector<float> m_BxD;	///< Control points of the derivative (x)
	std::vector<float> m_ByD;	///< Control points of the derivative (y)
	std::vector<float> m_BxD2;	///< Control points of the derivative (x)
	std::vector<float> m_ByD2;	///< Control points of the derivative (y)

protected:
	int ComputeCPD2(void);
	int ComputeCPD(void);
	int FindSpan(float t);
};

#endif // !defined(BSPLINE_H_)
