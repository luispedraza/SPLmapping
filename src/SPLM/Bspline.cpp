// Bspline.cpp: implementation of the CBspline class.
// Autho: Luis Pedraza
//////////////////////////////////////////////////////////////////////
#include "Bspline.h"
#include <math.h>
#include "../Utils/Utils.h"

float CBspline::nearest_tol = NEAREST_TOL;

CBspline::CBspline() {
	isnew = true;
	isextended = false;
	m_iOrder = 4; // Default order is 4 (cubic B-spline)
	m_iNumber = 0;
	m_bCPderiv = false;
	m_bCPderiv2 = false;
	PHI = NULL;
}

CBspline::~CBspline() {
	if (PHI)
		PHI->ReleaseAndDelete();
}

/**
 *	Evaluates the i-th basis function of order k at point t
 *	Calculations are performed implementing the Cox-de Boor formula
 *	as it is shown in the de Boor's article "B(asic)-spline Basics"
 *	\param t is the value of the parameter at which the B-spline is evaluated
 *	\param i in the index of the B-spline base function
 *	\param k is the order of the B-spline (degree k-1)
 **/
float CBspline::Basis(float t, int i, int k) {
	if ((t == m_fRange[1]) && (i == m_iNumber - 1))
		return 1.0F;
	// B-spline of degree 0 (order 1)
	if (k == 1) {
		return ((t >= m_Knots[i]) && (t < m_Knots[i + 1])) ? 1 : 0;
	} else {
		// Calculation of higher degree B-splines (recursion)
		Real w_i, w_iplus1; // Coefficients for the recursion formula
		float result;
		(m_Knots[i] == m_Knots[i + k - 1]) ? (w_i = 0) : (w_i
				= (t - m_Knots[i]) / (m_Knots[i + k - 1] - m_Knots[i]));
		(m_Knots[i + 1] == m_Knots[i + k]) ? (w_iplus1 = 0) : (w_iplus1
				= (m_Knots[i + k] - t) / (m_Knots[i + k] - m_Knots[i + 1]));
		result = w_i * Basis(t, i, k - 1) + w_iplus1 * Basis(t, i + 1, k - 1);
		return result;
	}
}
/**
 *	Evaluates the i-th basis function of order k at point t
 *	Calculations are performed implementing the formula Cox-de Boor
 *	as it is shown in the de Boor's article "B(asic)-spline Basics"
 *	\param t is the value of the parameter at which the B-spline is evaluated
 *	\param i in the index of the B-spline base function
 *	\param k is the order of the B-spline (degree k-1)
 **/
float CBspline::BasisD(float t, int i, int k) {
	// B-spline of degree 0 (order 1)
	if (k == 1) {
		return ((t >= m_KnotsD[i]) && (t < m_KnotsD[i + 1])) ? 1 : 0;
	} else {
		// Calculation of higher degree B-splines (recursion)
		Real w_i, w_iplus1; // Coeficients for the recursion formula
		float result;
		(m_KnotsD[i] == m_KnotsD[i + k - 1]) ? (w_i = 0) : (w_i = (t
				- m_KnotsD[i]) / (m_KnotsD[i + k - 1] - m_KnotsD[i]));
		(m_KnotsD[i + 1] == m_KnotsD[i + k]) ? (w_iplus1 = 0) : (w_iplus1
				= (m_KnotsD[i + k] - t) / (m_KnotsD[i + k] - m_KnotsD[i + 1]));
		result = w_i * BasisD(t, i, k - 1) + w_iplus1 * BasisD(t, i + 1, k - 1);
		return result;
	}
}

float CBspline::BasisD2(float t, int i, int k) {
	// B-spline of degree 0 (order 1)
	if (k == 1) {
		return ((t >= m_KnotsD2[i]) && (t < m_KnotsD2[i + 1])) ? 1 : 0;
	} else {
		// Calculation of higher degree B-splines (recursion)
		Real w_i, w_iplus1; // Coeficients for the recursion formula
		float result;
		(m_KnotsD2[i] == m_KnotsD2[i + k - 1]) ? (w_i = 0) : (w_i = (t
				- m_KnotsD2[i]) / (m_KnotsD2[i + k - 1] - m_KnotsD2[i]));
		(m_KnotsD2[i + 1] == m_KnotsD2[i + k]) ? (w_iplus1 = 0) : (w_iplus1
				= (m_KnotsD2[i + k] - t)
						/ (m_KnotsD2[i + k] - m_KnotsD2[i + 1]));
		result = w_i * BasisD2(t, i, k - 1) + w_iplus1 * BasisD2(t, i + 1, k
				- 1);
		return result;
	}
}
/**
 Knot vector initialization
 \param tmin Minimum value of the parameter t
 \param tmax Maximum value of the parameter t
 \param numberCP Number of control polygon vertices
 **/
int CBspline::InitUnifKnots(float tmin, float tmax, int numberCP) {
	int i; // counter
	// Number of control points
	m_iNumber = (numberCP < m_iOrder) ? m_iOrder : numberCP;
	int knumber = m_iNumber + m_iOrder; // Total number of knots
	float step = (tmax - tmin) / (knumber - 2 * m_iOrder + 1);
	// spline parameter range
	m_fRange[0] = tmin;
	m_fRange[1] = tmax;
	m_Knots.clear();
	m_Knots.push_back(tmin);
	for (i = 1; i < knumber; i++) {
		if ((i >= m_iOrder) && (i < knumber - m_iOrder + 1))
			m_Knots.push_back(m_Knots[i - 1] + step);
		else
			m_Knots.push_back(m_Knots[i - 1]);
	}
	m_Bx.clear();
	m_By.clear();

	return 0;
}

/**
 Evaluates the Bspline
 \param t : Value of the parameter
 \param x : Resulting x value
 \param y : Resulting y value
 **/
int CBspline::Eval(float t, float* x, float* y) {
	if (t == m_fRange[0]) {
		if (x != NULL)
			*x = m_Bx[0];
		if (y != NULL)
			*y = m_By[0];
		return 0;
	}
	if (t == m_fRange[1]) {
		if (x != NULL)
			*x = m_Bx.back();
		if (y != NULL)
			*y = m_By.back();
		return 0;
	}
	if ((t < m_fRange[0]) || (t > m_fRange[1]))
		return 1;

	Real xval = 0.0F, yval = 0.0F, basis = 0.0F;
	if (x == NULL) {
		for (int i = 0; i < m_iNumber; i++) {
			basis = Basis(t, i, m_iOrder);
			yval += (m_By[i] * basis);
		}
		*y = yval;
		return 0;
	}
	if (y == NULL) {
		for (int i = 0; i < m_iNumber; i++) {
			basis = Basis(t, i, m_iOrder);
			xval += (m_Bx[i] * basis);
		}
		*x = xval;
		return 0;
	}
	for (int i = 0; i < m_iNumber; i++) {
		basis = Basis(t, i, m_iOrder);
		xval += (m_Bx[i] * basis);
		yval += (m_By[i] * basis);
	}
	*x = xval;
	*y = yval;
	return 0;
}

/**
 Calculates the nearest point on the spline to the point (\param x, \param y)
 Return values:
 \param ts : value of the parameter at the nearst point to (x,y)
 \param xs : x value of the nearest point to (x,y)
 \param ys : y value of the nearest point to (x,y)
 **/
int CBspline::Nearest(float x, float y, float &ts, float &xs, float &ys,
		float &d) {
	float ini = m_fRange[0];
	float fin = m_fRange[1];
	double interv = fin - ini;
	double step = interv / NEAREST_DIV;
	float tlocal, xlocal, ylocal, dist, oldist;
	float difx, dify;
	while (interv > nearest_tol) {
		for (int i = 0; i <= NEAREST_DIV; i++) {
			tlocal = ini + i * step;
			Eval(tlocal, &xlocal, &ylocal);
			difx = xlocal - x;
			dify = ylocal - y;
			dist = difx * difx + dify * dify;
			if (i == 0) {
				oldist = dist;
				ts = tlocal;
				xs = xlocal;
				ys = ylocal;
				d = oldist;
			} else if (dist < oldist) {
				ts = tlocal;
				xs = xlocal;
				ys = ylocal;
				d = dist;
				oldist = dist;
			}
		}
		ini = max(m_fRange[0], ts-step);
		fin = min(m_fRange[1], ts+step);
		interv = fin - ini;
		step = interv / NEAREST_DIV;
	}
	return 0;
}

/**
 Knot vector initialization
 \param tmin Minimum value of the parameter t
 \param tmax Maximum value of the parameter t
 \param spacing Length of each polynomial piece (aprox)
 **/
int CBspline::InitUnifKnots2(float tmin, float tmax, float spacing) {
	int i; // counter
	int pieces = ceil((tmax - tmin) / spacing);
	spacing = (tmax - tmin) / pieces;
	int knumber = (pieces + 1) + 2 * (m_iOrder - 1); // Total number of knots
	m_iNumber = knumber - m_iOrder; // Number of control points
	m_fRange[0] = tmin; // spline parameter range
	m_fRange[1] = tmax;
	m_Knots.clear();
	for (i = 0; i < knumber; i++) {
		if (i < m_iOrder)
			m_Knots.push_back(tmin);
		if ((i >= m_iOrder) && (i < knumber - m_iOrder))
			m_Knots.push_back(m_Knots[i - 1] + spacing);
		if (i >= knumber - m_iOrder)
			m_Knots.push_back(tmax);
	}
	m_Bx.clear();
	m_By.clear();

	return 0;
}

/**	Retrieves the total length of the B-spline
 as the value of the range of the parameter t
 **/
float CBspline::GetLength() {
	return ((float) fabs(m_fRange[1] - m_fRange[0]));
}

/**
 Evaluates the first derivative of the Bspline
 \param t : Value of the parameter
 \param x : First derivative in x coord
 \param y : First derivative in y coord
 **/
int CBspline::EvalD(float t, float* x, float* y) {
	if (!m_bCPderiv)
		ComputeCPD();

	if (t == m_fRange[0]) {
		if (x != NULL)
			*x = m_BxD[0];
		if (y != NULL)
			*y = m_ByD[0];
		return 0;
	}
	if (t == m_fRange[1]) {
		if (x != NULL)
			*x = m_BxD.back();
		if (y != NULL)
			*y = m_ByD.back();
		return 0;
	}

	if ((t < m_fRange[0]) || (t > m_fRange[1]))
		return 1;

	Real xval = 0.0F, yval = 0.0F, basis = 0.0F;
	if (x == NULL) {
		for (int i = 0; i < m_iNumber - 1; i++) {
			basis = BasisD(t, i, m_iOrder - 1);
			yval += m_ByD[i] * basis;
		}
		*y = yval;
		return 0;
	}
	if (y == NULL) {
		for (int i = 0; i < m_iNumber - 1; i++) {
			basis = BasisD(t, i, m_iOrder - 1);
			xval += m_BxD[i] * basis;
		}
		*x = xval;
		return 0;
	}
	for (int i = 0; i < m_iNumber - 1; i++) {
		basis = BasisD(t, i, m_iOrder - 1);
		xval += m_BxD[i] * basis;
		yval += m_ByD[i] * basis;
	}

	*x = xval;
	*y = yval;
	return 0;
}
/**
 Evaluates the curvature of the B-spline at a given location
 \param t : Value of the parameter
 \param c : Curvature
 **/

int CBspline::Curvature(float t, float &c) {
	if ((t < m_fRange[0]) || (t > m_fRange[1]))
		return 1;

	float xd, yd, xd2, yd2;
	EvalD(t, &xd, &yd);
	EvalD2(t, &xd2, &yd2);
	c = (xd * yd2 - yd * xd2) / pow((xd * xd + yd * yd), 1.5);

	return 0;
}
/**
 Evaluates the second derivative of the Bspline
 \param t : Value of the parameter
 \param x : Second derivative in x coord
 \param y : Second derivative in y coord
 **/
int CBspline::EvalD2(float t, float* x, float* y) {
	if (!m_bCPderiv)
		ComputeCPD();
	if (!m_bCPderiv2)
		ComputeCPD2();

	if (t == m_fRange[0]) {
		if (x != NULL)
			*x = m_BxD2[0];
		if (y != NULL)
			*y = m_ByD2[0];
		return 0;
	}
	if (t == m_fRange[1]) {
		if (x != NULL)
			*x = m_BxD2.back();
		if (y != NULL)
			*y = m_ByD2.back();
		return 0;
	}

	if ((t < m_fRange[0]) || (t > m_fRange[1]))
		return 1;

	Real xval = 0.0F, yval = 0.0F, basis = 0.0F;
	if (x == NULL) {
		for (int i = 0; i < m_iNumber - 2; i++) {
			basis = BasisD2(t, i, m_iOrder - 2);
			yval += m_ByD2[i] * basis;
		}
		*y = yval;
		return 0;
	}
	if (y == NULL) {
		for (int i = 0; i < m_iNumber - 2; i++) {
			basis = BasisD2(t, i, m_iOrder - 2);
			xval += m_BxD2[i] * basis;
		}
		*x = xval;
		return 0;
	}
	for (int i = 0; i < m_iNumber - 2; i++) {
		basis = BasisD2(t, i, m_iOrder - 2);
		xval += m_BxD2[i] * basis;
		yval += m_ByD2[i] * basis;
	}

	*x = xval;
	*y = yval;
	return 0;
}

int CBspline::Fit(const std::vector<Point>* data, std::vector<float>* param,
		float spacing, int method) {
	int n = data->size(); // Number of data points
	std::vector<float>* t = param;
	if (t == NULL) {
		t = new std::vector<float>(n);
		t->at(0) = 0.0; // The first data point is at t=0
		float length = 0.0; // Total length of the spline (approximated by arcs)
		for (int j = 1; j < n; j++) {
			length += (data->at(j) - data->at(j - 1)).modulo();
			t->at(j) = length;
		}
	}
	InitUnifKnots2(0.0f, t->back(), spacing);

	// Now comes the spline approximation		
	NEWMAT::Matrix N(n, m_iNumber); // The matrix N of B-splines values
	NEWMAT::Matrix D(n, 2); // Data points
	NEWMAT::Matrix B(m_iNumber, 2); // B-spline coeficients (control polygon)

	// Matrix N construction
	for (int iN = 0; iN < n; iN++) {
		for (int jN = 0; jN < m_iNumber; jN++) {
			Real value = Basis(t->at(iN), jN, 4);
			N.element(iN, jN) = value;
		}
		D.element(iN, 0) = data->at(iN).x;
		D.element(iN, 1) = data->at(iN).y;
	}
	NEWMAT::Matrix T1 = N.t() * N;
	T1 = T1.i();
	NEWMAT::Matrix T2 = N.t() * D;
	B = T1 * T2;
	for (int j = 0; j < m_iNumber; j++) {
		m_Bx.push_back(B.element(j, 0));
		m_By.push_back(B.element(j, 1));
	}

	return 0;
}

int CBspline::Fit(const std::vector<float> *x, const std::vector<float> *y,
		std::vector<float> *param, float spacing, int method) {
	int n = x->size();

	std::vector<float>* t = param;
	if (t == NULL) {
		t = new std::vector<float>(n);
		t->at(0) = 0.0; // The first data point is at t=0
		float length = 0.0; // Total length of the spline (approximated by arcs)
		for (int j = 1; j < n; j++) {
			float difx = x->at(j) - x->at(j - 1);
			float dify = y->at(j) - y->at(j - 1);
			length += sqrt(difx * difx + dify * dify);
			t->at(j) = length;
		}
	}
	InitUnifKnots2(t->front(), t->back(), spacing);
	// Now comes the spline approximation		
	NEWMAT::Matrix D(n, 2); // Data points
	NEWMAT::Matrix B(m_iNumber, 2); // B-spline coefficients (control polygon)
	try {
		if (PHI == NULL) {
			PHI = new NEWMAT::Matrix(m_iNumber, n);
			NEWMAT::Matrix N(n, m_iNumber); // The matrix N of B-splines values
			// Matrix N construction
			for (int iN = 0; iN < n; iN++) {
				for (int jN = 0; jN < m_iNumber; jN++) {
					N.element(iN, jN) = Basis(t->at(iN), jN, m_iOrder);

				}
				D.element(iN, 0) = x->at(iN);
				D.element(iN, 1) = y->at(iN);
			}
			NEWMAT::CroutMatrix NTN = N.t() * N;
			if (NTN.IsSingular() == true)
				return -1;
			*PHI = NTN.i() * N.t();
		} else {
			for (int iN = 0; iN < n; iN++) {
				D.element(iN, 0) = x->at(iN);
				D.element(iN, 1) = y->at(iN);
			}

		}

		B = (*PHI) * D;
		m_Bx.clear();
		m_By.clear();
		for (int j = 0; j < m_iNumber; j++) {
			m_Bx.push_back(B.element(j, 0));
			m_By.push_back(B.element(j, 1));
		}
	} catch (Exception) {
		cout << Exception::what() << endl;
	}
	return 0;
}

/** Translates and rotates the spline **/
int CBspline::TransRot(float x, float y, float th) {
	float Bx, By;
	Real costh = cos(th);
	Real sinth = sin(th);
	for (int i = 0; i < m_iNumber; i++) {
		Bx = costh * (m_Bx[i] - x) + sinth * (m_By[i] - y);
		By = -sinth * (m_Bx[i] - x) + costh * (m_By[i] - y);
		m_Bx[i] = Bx;
		m_By[i] = By;
	}
	m_bCPderiv = false;
	m_bCPderiv2 = false;
	return 0;
}
/** Translates the spline **/
int CBspline::Translate(float x, float y) {
	for (int i = 0; i < m_iNumber; i++) {
		m_Bx[i] = m_Bx[i] - x;
		m_By[i] = m_By[i] - y;
	}
	m_bCPderiv = false;		// need to be computed
	m_bCPderiv2 = false;	// need to be computed
	return 0;
}
/** Rotates the spline **/
int CBspline::Rotate(float th) {
	float Bx, By;
	Real costh = cos(th);
	Real sinth = sin(th);
	for (int i = 0; i < m_iNumber; i++) {
		Bx = costh * m_Bx[i] + sinth * m_By[i];
		By = -sinth * m_Bx[i] + costh * m_By[i];
		m_Bx[i] = Bx;
		m_By[i] = By;
	}
	m_bCPderiv = false;		// need to be computed
	m_bCPderiv2 = false;	// need to be computed
	return 0;
}
int CBspline::FindSpan(float t) {
	return 0;
}
/** Control points of first derivative **/
int CBspline::ComputeCPD() {
	m_BxD.clear();
	m_ByD.clear();
	m_KnotsD.clear();
	int c;
	for (c = 1; c < m_Knots.size() - 1; c++)
		m_KnotsD.push_back(m_Knots[c]);
	float fact = m_iOrder - 1;
	for (c = 1; c < m_iNumber; c++) {
		m_BxD.push_back(fact * (m_Bx[c] - m_Bx[c-1]) / (m_Knots[c+m_iOrder-1] - m_Knots[c]));
		m_ByD.push_back(fact * (m_By[c] - m_By[c-1]) / (m_Knots[c+m_iOrder-1] - m_Knots[c]));
	}
	m_bCPderiv = true;		// have been computed
	m_bCPderiv2 = false;	// need to be computed
	return 0;
}

/** Control points of second derivative **/
int CBspline::ComputeCPD2() {
	m_BxD2.clear();
	m_ByD2.clear();
	m_KnotsD2.clear();
	int c;
	for (c = 1; c < m_KnotsD.size()-1; c++)
		m_KnotsD2.push_back(m_KnotsD[c]);
	float fact = m_iOrder - 2;
	for (c = 1; c < m_iNumber-1; c++) {
		m_BxD2.push_back(fact * (m_BxD[c] - m_BxD[c-1]) / (m_KnotsD[c+m_iOrder-2] - m_KnotsD[c]));
		m_ByD2.push_back(fact * (m_ByD[c] - m_ByD[c-1]) / (m_KnotsD[c+m_iOrder-2] - m_KnotsD[c]));
	}
	m_bCPderiv2 = true;		// have been computed
	return 0;
}
