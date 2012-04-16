// Map.h: interface for the CMap class.
// Author: Luis Pedraza
//////////////////////////////////////////////////////////////////////

#if !defined(MAPSP_H_)
#define MAPSP_H_

#include "Bspline.h"
#include "UncertainRel.h"
#include <vector>
#include <fstream>
#include <stdlib.h>

struct PoseStruct
{
	double x; 
	double y; 
	double ang;
	PoseStruct(double xx, double yy, double aa) : x(xx), y(yy), ang(aa) {};
	PoseStruct() : x(0.0), y(0.0), ang(0.0) {};
};

/** 
	Class for a map
	Contains a list of splines
**/
class CMapSP  
{
public:
	int Global2Robot(float xgf, float ygf, float &xrf, float &yrf);
	int ProbEllipseParams(float sxx, float syy, float sxy, float &a, float &b, float &ang, float conf=0.95);
	int GetRobotPose(float &xx, float &yy, float &ang);
	int SetRobotPose(float &xx, float &yy, float &ang);
	NEWMAT::ColumnVector x;				///< State vector
	NEWMAT::SymmetricMatrix P;			///< Convariance matrix
	std::vector<CBspline*> m_Splines;	///< Vector containing map features	
	CUncertainRel PosAbs;				///< Map position in global coordinates
	CUncertainRel PosRel;				///< Map pose relative to its parent
	CMapSP* next;						///< Next map in the list
	int Load(std::ifstream& file);		///< Loads map from a file
	int Save(std::ofstream& file);		///< Saves map to a file

	std::vector<PoseStruct> m_Sigmas;	///< Sigmas (for consistency)
	std::vector<PoseStruct> m_Error;	///< Error (for consistency)
	std::vector<PoseStruct> m_Estim;	///< Estimated trajectory	
	std::vector<PoseStruct> m_Odom;		///< Odom. trajectory
	std::vector<PoseStruct> m_Sim;		///< Simulation trajectory (ground truth)
	std::vector<double> m_dMahala;		///< Mahalanobis distance to ground truth (robot pose)
	std::vector<double> precision;		///< Trajectory precision
	std::vector<NEWMAT::Matrix> P_values;			///< Elements of P(3,3)
	std::vector<NEWMAT::Matrix> Pinv_values;			///< Elements of P(3,3)

	std::vector<int> m_Tpred;				///< Prediction stage processing time
	std::vector<int> m_Tupdt;				///< Update stage processing time
	std::vector<int> m_N;					///< Number of control points in the map
	
	int GetRobotPose(PoseStruct& pose); ///< Retrieves robot's pose
	int SetRobotPose(PoseStruct& pose); ///< Establishes robot's pose
	int Clear(void);					///< Clears the map (removes all splines)
	int AddFeature(CBspline* spline);	///< Adds a new spline to the map
	int m_nSP;							///< Number of features contained in the map (splines)
	int m_nCP;							///< Number of control points in the map
	CMapSP();							///< Default constructor
	virtual ~CMapSP();					///< Default destructor
	int idMap;							///< Map id
private:
	PoseStruct RobotPose;
};

#endif // !defined(MAPSP_H_)
