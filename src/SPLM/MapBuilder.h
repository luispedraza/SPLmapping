// MapBuilder.h: interface for the CMapBuilder class.
// Author: Luis Pedraza
//////////////////////////////////////////////////////////////////////

#if !defined(MAP_BUILDER_H_)
#define MAP_BUILDER_H_

#include <vector>
#include <newmat/newmat.h>

#include <iostream>
#include "../Random/RandomLib.h"				// Random numbers generation lib 
#include "../Random/Rand.h"						// Random numbers generation class 

#include "SpatialRel.h"
#include "Map.h"
#include <sys/timeb.h>
#include "Robot.h"


#ifdef use_namespace
	using namespace NEWMAT;              // access NEWMAT namespace
#endif

///////////////////////////////////
// METHODS
#define M_NONE	0	// ONLY ODOMETRY IS USED (DEAD RECK.)
#define M_LOCAL	1	// Only position is stochastic
#define	M_SLAM	2	// Stochastic map - FULL SLAM

/** Object data structure (useful info) **/
struct SLAMfeature
{
	// Kalman state extension
	NEWMAT::Matrix GExr;	///< Extension submatrix (d/dxr)
	NEWMAT::Matrix GExm;	///< Extension submatrix (d/dxm)
	NEWMAT::Matrix GEz;	///< Extension submatrix (d/dz) 
	NEWMAT::Matrix GAx;	///< New spline submatrix (d/dx)
	NEWMAT::Matrix GAz;	///< New spline submatrix (d/dz)
	NEWMAT::Matrix PHIext;	///< Extension auxiliar matrix 
	// Kalman filter related variables
	std::vector<Real> tsol;	///< t* solution of newton raphson
	std::vector<float> res;		///< Residuals
	std::vector<float> dhdth;	///< dh/dth
	std::vector<float> dhdy;	///< dh/dy
	std::vector<float> dhdx;	///< dh/dx
	std::vector<float> dhdBx;	///< dh/dbx
	std::vector<float> dhdBy;	///< dh/dby
	// Feature related and data acquisition variables
	std::vector<float> y;		///< data point y coordinate
	std::vector<float> x;		///< data point x coordinate
	std::vector<float> l;		///< range measurement
	std::vector<float> t;		///< parameter t (estimation)
	std::vector<float> tau;		///< tau angle
	std::vector<float> mu;		///< mu angle (tau + robot orientation)
	std::vector<float> sinmu;	///< sin(mu)
	std::vector<float> cosmu;	///< cos(mu)
	CBspline* spline;			///< Spline feature	
	bool isnew;					///< The feature is new and has to be added to the map
	std::vector<struct Link> links;		///< Kalman filter links
	struct Link* linkRE;				///< right extension link
	struct Link* linkLE;				///< left extension link

	SLAMfeature() : isnew(false), spline(NULL), linkRE(NULL), linkLE(NULL) {};
};

/** Options structure for map building **/
struct BuildOptions
{
	bool savelog;		///< save log files of slam process
	bool addnoise;		///< add noise to simulated data
	bool datasim;		///< data is imulated
	int method;			///< Building method (M_NONE, M_LOCAL, M_SLAM)
	bool submaps;		///< Build or not submaps
	int submapSize;		///< Number of CP for each submap	
	bool extension;		///< Perform splines extension
	float minext;		///< Min length for extension	
	float span;			///< Span for polynomial pieces	
	float limdist;		///< Distance limit for segmentation
	float limang;		///< Angular limit for segmentation
	float minlo;		///< Min length of an object
	int minmo;			///< Min measurement for an object
	float offset;		///< Robot's sensor offset
	float laser_min;	///< Min angular range of laser sensor
	float laser_max;	///< Max angular range of laser sensor
	float laser_step;	///< Angle between measurements
	float range;		///< Laser range
	int nlaser;			///< Laser measurements per scan
	/****************/
	/* Noise values */
	/****************/
	float sigmaD;		///< Drive noise %
	float sigmaS;		///< Steer noise %	
	float sigmaDmin;
	float sigmaSmin;

	float sigmaX;		///< Odometry x noise %
	float sigmaY;		///< Odometry y noise %
	float sigmaXmin;	///< Min value of \f$ \sigma_x \f$
	float sigmaYmin;	///< Min value of \f$ \sigma_y \f$	

	float sigmaL;		///< Laser noise sigma (m)
	float matchmin;		///< Min matched length
	float matchtol;		///< Matching tolerance
};


/** Map Building class **/
class CMapBuilder  
{
public:
	CMapSP* m_Map;					///< Pointer to the map being built (current map)
	CMapSP* m_MatchMap;
	CRobot m_Robot;
	float MSE;						///< mean squared error of residuals (for TRo paper)
	NEWMAT::ColumnVector xSM;		///< State vector for submaps
	NEWMAT::SymmetricMatrix PSM;		///< COvariance matrix for submaps
	PoseStruct poseZero;
	int FindMatch(CMapSP* map, bool usecp=false);
	

	int GetSplinePos(CBspline* spline);
	int timeUpdt;	///< Update stage time in millisec
	int timePred;	///< Prediction stage time in millisec
	CMapBuilder(BuildOptions opt);
	int SetOptions(BuildOptions opt);		///< Sets building options
	BuildOptions GetOptions(void);			///< Retrieves building options
	std::vector<SLAMfeature*> m_Features;	///< Observed Features
	int ProcessOdometry(float x, float y, float ang);
	CMapSP* Init(int type, char *source);

	CMapBuilder();						///< Default constructor
	virtual ~CMapBuilder();				///< Default destructor
	BuildOptions options;				///< Options for map building


	std::vector<CMapSP*> m_Mlist;		///< List of submaps when submaps are built
	unsigned int buildtime;				///< Total map building time

	void ProcessLaser(std::vector<float> &scan, float &rmin, float &rmax, float &amin, float &amax, float &step);
private:
	void LogError(void);
	CRand randX, randY, randTH, randD;
	int EKF_PredictXY(void);
	int OdometryXY(void);
	void CopyMap2State(void);
	int ndata_ext;	///< Number of data points used in extensions
	int ndata_new;	///< Number of data points used in new features	
	int dim_new;	///< State dim after new features >= dim_ext
	int dim_ext;	///< State dim after extension >= dim_old
	int dim_old;	///< State dim befor extension / new features
	int EKF_Enlarge(void);	///< Map enlargement (covariance update)
	void LogSymMatrix(NEWMAT::SymmetricMatrix &m, const char* filename);
	void LogMatrix(NEWMAT::Matrix &m, const char* filename);
	void CopyState2Map(void);
	void LogKalmanUpdate(int Iter, int Samples, NEWMAT::Matrix &Gain);
	FILE* logfile;
	int Clean(void);
	float m_fSigmaD;	///< Sigma drive (odometry) 
	float m_fSigmaS;	///< Sigma steer (odometry)
	float m_fSigmaX;	///< Sigma x (odometry)
	float m_fSigmaY;	///< Sigma y (odometry)
	int ProcessSubmaps(void);
	// Auxiliary functions for spline extension ////////////////////////
	Real Lambda(const std::vector<float> &t, int k, int i, int j);
	Real lambda(const std::vector<float> &t, int k, int i, int j);
	Real Omega(const std::vector<float> &t, int k, int i, int j);
	Real omega(const std::vector<float> &t, int k, int i, int j);
	/////////////////////////////////////////// En spline extension
	int ProcessExt(void);
	int Match(void);			///< Match observation against he map. Keys construction
	int Aprox(void);			///< Aproximates a set of observations
	PoseStruct odomOld;			///< Stores last odometry lecture (for deltas and because of Urbano offset)
	float deltaTH;				///< Odometric delta steer
	float deltaD;				///< Odometric delta drive
	float deltaX;				///< Odometry delta X
	float deltaY;				///< Odometry delta Y
	int m_iSamples;				///< Number of processed samples
	int Prepro(void);			///< Laser preprocessing (segmentation)
	int EKF_Update(void);		///< EKF Update stage
	int EKF_PredictDS(void);	///< EKF Predict stage
	int OdometryDS(void);		///< Odometry processing
	int ProcessError(void);
	int ProcessNew(void);		///< Process new splines
protected:
};

#endif // !defined(MAP_BUILDER_H_)
