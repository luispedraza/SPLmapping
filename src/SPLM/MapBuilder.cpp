// MapBuilder.cpp: implementation of the CMapBuilder class.
// Author: Luis Pedraza
//////////////////////////////////////////////////////////////////////
#include "MapBuilder.h"
#include <stdlib.h>
#include "../Utils/Utils.h"

#ifndef DEG2RAD
#define DEG2RAD 0.017453293f
#endif
#ifndef RAD2DEG
#define RAD2DEG 57.295779513f
#endif

float ForceInRange(float angle) {
	float inrange = angle;
	while (inrange >= TWOPI)
		inrange -= TWOPI;
	while (inrange < 0)
		inrange += TWOPI;
	return inrange;
}

float AngError(float ang2, float ang1) {
	float delta = ang2 - ang1;
	return delta;
}

/** Default constructor for a CMapBuilder object **/
CMapBuilder::CMapBuilder() {
	// Matching variables ///////////
	m_Map = new CMapSP();
	m_MatchMap = NULL;
	/////////////////////////////////
	buildtime = 0;
	odomOld = PoseStruct(0.0, 0.0, 0.0); // robot pose init.
	deltaD = 0.0f;
	deltaTH = 0.0f;
	m_iSamples = 0; // num samples initialization

	///////////////////////////////////////////////////////////////
	// Default OPTIONS ////////////////////////////////////////////
	options.addnoise = false;
	options.datasim = false;
	//options.method = M_SLAM;
	//options.method = M_LOCAL;
	options.method = M_NONE;
	options.limang = cos(PI / 8); // For CORNER DETECTION pi/10
	options.limdist = 0.2F; // 0.2
	options.minmo = 10; // min object measurements	(8)
	options.minlo = 0.4F; // min object length	(0.2) 0.15
	options.matchmin = 0.1F; // 0.1
	options.matchtol = 0.2F; // matching tolerance

	options.sigmaD = 10.0F; // % noise
	options.sigmaDmin = 0.002F; // 0.001F;

	options.sigmaS = 10.0F; // % noise
	options.sigmaSmin = 0.02F; // 0.02F;
	options.sigmaX = 10.0F;
	options.sigmaY = 10.0F;
	options.sigmaXmin = 0.001F;
	options.sigmaYmin = 0.001F;

	/*	options.sigmaS = 5.0F;		// % noise
	 options.sigmaSmin = 0.02F;		// 0.02F;
	 options.sigmaX = 5.0F;
	 options.sigmaY = 5.0F;
	 options.sigmaXmin = 0.05F;
	 options.sigmaYmin = 0.05F;
	 */
	options.sigmaL = 0.005F; // m 0.008

	options.offset = 0.00F; //OFFSET;
	options.span = 1.0F;
	// EXTENSION OPTIONS
	options.minext = 1.0F;
	options.extension = false;
	options.submaps = false;
	options.submapSize = 80; // default submap size
	m_fSigmaD = m_fSigmaS = m_fSigmaX = m_fSigmaY = 0.0F;
	options.savelog = false;
	timePred = timeUpdt = 0;
	logfile = fopen("log.txt", "w");
	///////// Map enlarging options
	ndata_new = 0;
	ndata_ext = 0;
	dim_old = 0;
	dim_new = 0;
	dim_ext = 0;
}

CMapBuilder::~CMapBuilder() {
	fclose(logfile);
}

/** Map builder initialization (data file setting) **/
CMapSP* CMapBuilder::Init(int type, char *source) {
	m_Map = new CMapSP;
	m_Mlist.push_back(m_Map);
	return m_Map;
}

/***************************************************************
 Odometry Processing
 ****************************************************************/
int CMapBuilder::ProcessOdometry(float x, float y, float ang) {
	m_Robot.x = x;
	m_Robot.y = y;
	m_Robot.ang = ForceInRange(ang);

	/*
	 struct timeb t1, t2;
	 ftime(&t1); // initial time acquisition
	 //OdometryDS();			// Odometry data => odometry processing (drive steer)
	 //EKF_PredictDS();		// Kalman filter PREDICTION stage (drive steer)
	 OdometryXY(); 			// Odometry data => odometry processing (rel. transf)
	 EKF_PredictXY(); 		// Kalman filter PREDICTION stage (rel. transf)
	 ftime(&t2);
	 timePred = (t2.time - t1.time) * 1000 + (t2.millitm - t1.millitm); // elapsed time for predeiction
	 buildtime += timePred; // total time update
	 m_Map->m_Tpred.push_back(timePred);
	 */
	return 0;
}

/** Process Robot odometry 
 - Odometry acquisition
 - CMapBuilder::deltaD and CMapBuilder::deltaTH calculation
 - m_Map->m_Odom update
 - odomOld update
 **/
int CMapBuilder::OdometryDS() {
	// Offset calculations (LASER POSITION in Urbano robot)
	float ang = ForceInRange(m_Robot.ang);
	float x = m_Robot.x + options.offset * cos(ang);
	float y = m_Robot.y + options.offset * sin(ang);
	// Odometric trajectory //////////////////////////////////////
	PoseStruct odompos(x, y, ang);
	if (m_iSamples > 0) // not the first sample
	{
		float dX = m_Robot.x - odomOld.x;
		float dY = m_Robot.y - odomOld.y;
		deltaD = (float) sqrt(dX * dX + dY * dY);
		deltaTH = AngError(odomOld.ang, ang);
		m_fSigmaD = options.sigmaDmin + (options.sigmaD / 100) * deltaD;
		m_fSigmaS = options.sigmaSmin + (options.sigmaS / 100) * deltaTH;
		if (options.addnoise) {
			deltaD = deltaD + randD.RandomNormal(0, m_fSigmaD, true);
			deltaTH = deltaTH + randTH.RandomNormal(0, m_fSigmaS, true);
		}
	}
	odomOld.x = m_Robot.x;
	odomOld.y = m_Robot.y;
	odomOld.ang = ang;
	if (options.datasim) /// If simulated data...
	{
		m_Mlist[0]->m_Sim.push_back(odompos); // New simulated sample added
		if (m_iSamples > 0) {
			float deltaTH_2 = deltaTH / 2;
			PoseStruct oldpose = m_Mlist[0]->m_Odom.back();
			float oldTH = oldpose.ang;
			float newTH = ForceInRange(oldTH + deltaTH);
			// Odometry calculation
			odompos.x = oldpose.x + options.offset * (cos(newTH) - cos(oldTH))
					+ deltaD * cos(oldTH + deltaTH_2);
			odompos.y = oldpose.y + options.offset * (sin(newTH) - sin(oldTH))
					+ deltaD * sin(oldTH + deltaTH_2);
			odompos.ang = newTH;
		}
	}
	m_Mlist[0]->m_Odom.push_back(odompos); // New odometry sample added
	m_iSamples++;
	return 0;
}

/** Kalman filter prediction **/
int CMapBuilder::EKF_PredictDS() {
	if (m_iSamples == 1) // First odometry sample
	{
		PoseStruct newpose(m_Mlist[0]->m_Odom[0].x, m_Mlist[0]->m_Odom[0].y,
				m_Mlist[0]->m_Odom[0].ang);
		m_Map->m_Estim.push_back(newpose);
		m_Map->SetRobotPose(newpose); // Pose initialization
		return 0;
	}

	Real deltaD_2 = deltaD / 2;
	Real deltaTH_2 = deltaTH / 2;
	/////////////////////
	PoseStruct oldpose;
	m_Map->GetRobotPose(oldpose);
	float oldTH = oldpose.ang;
	float newTH = ForceInRange(oldTH + deltaTH);
	////////////////////////////////////////////////
	// State prediction ////////////////////////////
	PoseStruct newpose(oldpose.x + options.offset * (cos(newTH) - cos(oldTH))
			+ deltaD * cos(oldTH + deltaTH_2), oldpose.y + options.offset
			* (sin(newTH) - sin(oldTH)) + deltaD * sin(oldTH + deltaTH_2),
			newTH);

	m_Map->SetRobotPose(newpose);
	m_Map->m_Estim.push_back(newpose);
	////////////////////////////////////////////////
	// Covariance matrix calculation ///////////////
	NEWMAT::Matrix Fx = NEWMAT::IdentityMatrix(m_Map->x.Nrows());
	Fx.element(0, 2) = oldpose.y - newpose.y;
	Fx.element(1, 2) = newpose.x - oldpose.x;

	NEWMAT::Matrix Fu(m_Map->x.Nrows(), 2);
	Fu = 0.0;
	Fu.element(0, 0) = cos(oldTH + deltaTH_2);
	Fu.element(1, 0) = sin(oldTH + deltaTH_2);

	Fu.element(0, 1) = -options.offset * sin(newTH) - deltaD_2 * Fu.element(1,
			0);
	Fu.element(1, 1) = options.offset * cos(newTH) + deltaD_2
			* Fu.element(0, 0);
	Fu.element(2, 1) = 1.0;

	NEWMAT::DiagonalMatrix Qds(2);
	Qds.element(0) = m_fSigmaD * m_fSigmaD;
	Qds.element(1) = m_fSigmaS * m_fSigmaS;

	m_Map->P << (Fx * m_Map->P * Fx.t() + Fu * Qds * Fu.t());
	if (options.savelog)
		LogSymMatrix(m_Map->P, "Ppre.log");

	return 0;
}

/** Kalman filter update (robot only)**/
int CMapBuilder::EKF_Update() {
	try {
		double Rcoef = 1.0; // inflation coef for measure noise
		NEWMAT::Matrix Hx(options.nlaser, m_Map->x.Nrows());
		Hx = 0.0;
		NEWMAT::ColumnVector res(options.nlaser);
		res = 0.0;

		int nKalman = 0; // number of valid measures for Kalman filter
		for (int iobs = 0; iobs < m_Features.size(); iobs++) // for all observed features ...
		{
			SLAMfeature* feat = m_Features[iobs];
			if (feat->isnew)
				continue; // nothing to do, as there is no association
			CBspline* spo = feat->spline; // current observed spline
			for (int ilink = 0; ilink < feat->links.size(); ilink++) // for all links
			{
				CBspline* spm = feat->links[ilink].spline; // map spline
				//spm->tempt.clear();
				Real t0 = feat->links[ilink].Keys[2]; // NewtonRaphson initial value

				for (int imed = 0; imed < feat->l.size(); imed++) {
					if (feat->t[imed] < feat->links[ilink].Keys[0])
						continue;
					if (feat->t[imed] > feat->links[ilink].Keys[1])
						break;
					CBspline spaux = *spm; // auxiliar spline to be rotated and traslated
					spaux.TransRot(m_Map->x.element(0), m_Map->x.element(1),
							feat->mu[imed]);
					float x, y, xd, yd, tsol;
					bool solfound = false; // NR has found a solution
					////////////////////////////////////////////////////////////
					// NEWTON-RAPHSON //////////////////////////////////////////
					for (int inr = 0; inr < 10/*max iter*/; inr++) {
						spaux.Eval(t0, NULL, &y);
						spaux.EvalD(t0, NULL, &yd);
						tsol = t0 - (y / yd);
						if (tsol < spaux.m_fRange[0])
							//{t0 = splineaux.m_fRange[0]; continue;}
							break;
						if (tsol > spaux.m_fRange[1])
							//{t0 = splineaux.m_fRange[1]; continue;}
							break;
						if (fabs(tsol - t0) < 0.001) {
							solfound = true;
							break;
						} else
							t0 = tsol;
					}
					// END Newton Raphson	//////////////////////////////////////////
					//////////////////////////////////////////////////////////////////
					if (!solfound)
						continue;

					spaux.Eval(tsol, &x, &y);
					spaux.EvalD(tsol, &xd, &yd);
					Real tgth_ = (Real) yd / (Real) xd;
					if (tgth_ == 0)
						continue;

					//feat->tsol.push_back(tsol);
					//feat->res.push_back(feat->l[imed] - x);	// residual
					//feat->dhdx.push_back(-feat->cosmu[imed] - feat->sinmu[imed]/tgth_);
					//feat->dhdy.push_back(-feat->sinmu[imed] + feat->cosmu[imed]/tgth_); // + o - ?
					//feat->dhdth.push_back(x/tgth_);
					//feat->links[ilink].spline->tempt.push_back(tsol);
					Hx.element(nKalman, 0) = -(Real) feat->cosmu[imed]
							- (Real) feat->sinmu[imed] / tgth_; //dh/dx
					Hx.element(nKalman, 1) = -(Real) feat->sinmu[imed]
							+ (Real) feat->cosmu[imed] / tgth_; //dh/dy
					Hx.element(nKalman, 2) = (Real) x / tgth_; //dh/df
					res.element(nKalman) = feat->l[imed] - x;
					//////////////////////////////////////////////////////////
					if (options.method == M_SLAM) {
						int pos = GetSplinePos(spm);
						int nCP = spm->m_iNumber;
						for (int ijacob = 0; ijacob < nCP; ijacob++) {
							Real basis = feat->links[ilink].spline->Basis(tsol,
									ijacob, 4);
							Hx.element(nKalman, pos + ijacob) = -basis
									* Hx.element(nKalman, 0);
							Hx.element(nKalman, pos + ijacob + nCP) = -basis
									* Hx.element(nKalman, 1);
						}
					}
					nKalman++; // New Kalman data point
				} // for imed
			} // for ilink
		} // for iobs

		if (nKalman == 0)
			return 1; // nothing to update ////// return 1 ///////

		NEWMAT::IdentityMatrix R(nKalman);

		R = options.sigmaL * options.sigmaL * Rcoef;
		Hx = Hx.Rows(1, nKalman);
		res = res.Rows(1, nKalman);

		//NEWMAT::Matrix S = Hx * m_Map->P * Hx.t() + R;
		NEWMAT::SymmetricMatrix S;
		S << Hx * m_Map->P * Hx.t() + R;
		NEWMAT::Matrix W = m_Map->P * Hx.t() * S.i();

		m_Map->x = m_Map->x + W * res;
		//	m_Map->P << m_Map->P - W * S * W.t();
		//m_Map->P << (NEWMAT::IdentityMatrix(m_Map->x.Nrows()) - W*Hx) * m_Map->P;

		// Joseph
		NEWMAT::IdentityMatrix I(m_Map->x.Nrows());
		NEWMAT::Matrix IKH(m_Map->x.Nrows(), m_Map->x.Nrows());
		IKH << I - (W * Hx);
		m_Map->P << IKH * m_Map->P * IKH.t() + W * R * W.t();
		// end joseph

		m_Map->x.element(2) = (Real) ForceInRange(m_Map->x.element(2));
		m_Map->m_Estim.push_back(PoseStruct(m_Map->x.element(0),
				m_Map->x.element(1), m_Map->x.element(2)));

		CopyState2Map();

		// MSE = res.t() * res;
	}

	catch (Exception) {
		std::ofstream error("exceptions.log", ios::app);
		error << "sample num: " << m_iSamples << endl;
		error << Exception::what();
		error << endl;
		error.close();
	}

	return 0;
}

/** Laser measurements preprocessing **/
int CMapBuilder::Prepro() {
	int i; // counter variable
	//PoseStruct poseR;
	//m_Map->GetRobotPose(poseR);

	int nData = m_Robot.m_Laser.nData;
	std::vector<float> mu(nData);
	std::vector<float> cosmu(nData);
	std::vector<float> sinmu(nData);
	float rMax = m_Robot.m_Laser.rangeMax;
	float out = rMax * 2; // out of range

	for (i = 0; i < nData; i++) {
		mu[i] = m_Robot.m_Laser.tau[i] + m_Robot.ang;
		cosmu[i] = cos(mu[i]);
		sinmu[i] = sin(mu[i]);
		if (options.addnoise) // add noise to laser
		{
			if (m_Robot.m_Laser.ranges[i] < rMax) {
				float lnoise = m_Robot.m_Laser.ranges[i] + RandomNormal(0,
						options.sigmaL, true);
				if (lnoise < rMax)
					m_Robot.m_Laser.ranges[i] = lnoise;
				else
					m_Robot.m_Laser.ranges[i] = rMax;
			}
		}
	}

	float Difx[360];
	float Dify[360];
	float PSt[361];
	float Pcos[361];

	for (i = 0; i < nData; i++) // for all laser measurements
	{
		if (i == 0) {
			PSt[0] = 0.0f;
			Pcos[0] = 1.0f;
			Difx[0] = 0.0f;
			Dify[0] = 0.0f;
		} else {
			Difx[i] = m_Robot.m_Laser.x[i] - m_Robot.m_Laser.x[i - 1];
			Dify[i] = m_Robot.m_Laser.y[i] - m_Robot.m_Laser.y[i - 1];
			PSt[i] = (float) sqrt(Difx[i] * Difx[i] + Dify[i] * Dify[i]);
			if (i > 1) {
				Pcos[i - 1] = (Difx[i - 1] * Difx[i] + Dify[i - 1] * Dify[i])
						/ (PSt[i - 1] * PSt[i]);
			}
		}
	}

	Pcos[i - 1] = 1.0f;
	int iseg = 0;
	int mseg = 0;
	SLAMfeature* feat = NULL;
	while (true) {
		float z = m_Robot.m_Laser.ranges[iseg];
		if (mseg == 0) {
			feat = new SLAMfeature;
			mseg = 1; // first measurement
			feat->x.push_back(m_Robot.x + z * cosmu[iseg]);
			feat->y.push_back(m_Robot.y + z * sinmu[iseg]);
			feat->l.push_back(z);
			feat->t.push_back(0.0f);
			feat->tau.push_back(m_Robot.m_Laser.tau[iseg]);
			feat->mu.push_back(mu[iseg]);
			feat->sinmu.push_back(sinmu[iseg]);
			feat->cosmu.push_back(cosmu[iseg]);
		} else if (mseg == 1) // second measurement of current object
		{
			if ((PSt[iseg] <= options.limdist) && (Pcos[iseg] > options.limang)
					&& (m_Robot.m_Laser.ranges[iseg] < rMax)) {
				mseg = 2;
				feat->x.push_back(m_Robot.x + z * cosmu[iseg]);
				feat->y.push_back(m_Robot.y + z * sinmu[iseg]);
				feat->l.push_back(z);
				feat->t.push_back(PSt[iseg]);
				feat->tau.push_back(m_Robot.m_Laser.tau[iseg]);
				feat->mu.push_back(mu[iseg]);
				feat->sinmu.push_back(sinmu[iseg]);
				feat->cosmu.push_back(cosmu[iseg]);
			} else {
				mseg = 0;
				iseg--;
				delete feat;
			}
		} else if ((Pcos[iseg] > options.limang)
				&& (m_Robot.m_Laser.ranges[iseg] < rMax)
				&& (max(PSt[iseg], PSt[iseg - 1])< min(
						PSt[iseg], PSt[iseg - 1]) * 1.5))
				{
					mseg++;
					feat->x.push_back(m_Robot.x + z * cosmu[iseg]);
					feat->y.push_back(m_Robot.y + z * sinmu[iseg]);
					feat->l.push_back(z);
					feat->t.push_back(PSt[iseg] + feat->t.back()); // cumulated length
					feat->tau.push_back(m_Robot.m_Laser.tau[iseg]);
					feat->mu.push_back(mu[iseg]);
					feat->sinmu.push_back(sinmu[iseg]);
					feat->cosmu.push_back(cosmu[iseg]);
					if (iseg == nData - 1) // last measurement
					{
						if ((mseg >= options.minmo) && (feat->t.back() >= options.minlo))
						m_Features.push_back(feat);
						else
						delete feat;
					}
				} else if ((mseg >= options.minmo) && (feat->t.back() >= options.minlo)) // Valid object
				{
					mseg = 0;
					iseg--;
					m_Features.push_back(feat);
				} else {
					mseg = 0;
					iseg--;
					delete feat;
				}
				iseg = iseg + 1;
				if (iseg == nData)
				break;
			}
			return 0;
		}

		/** Aproximates the detected features with B-splines **/
int CMapBuilder::Aprox() {
	cout << m_Features.size() << endl;
	for (int i = 0; i < m_Features.size(); i++) // for all detected features...
	{
		m_Features[i]->spline = new CBspline;
		if(m_Features[i]->spline->Fit(&m_Features[i]->x, &m_Features[i]->y,
				&m_Features[i]->t, options.span) == -1)
		{
			cout << "que mal\n";
			delete m_Features[i]->spline;
			m_Features[i]->spline = NULL;
		}
	}
	return 0;
}

/** Matching of current observation against the map **/
int CMapBuilder::Match() {
	if (m_Map == NULL) {
		cout << "no map" << endl;
		return 0;
	}
	float dmincp = 4.0f; // min distance for CP comparison
	CBspline* splineo; // observed spline
	CBspline* splinem; // map spline
	float IniO[2], FinO[2], IniM[2], FinM[2]; // beginning and end points matched span
	bool match1, match2; // match found / not found
	float d1, d2; // distances between matched points
	// LOOP for the list of detected objects (observation)
	int iobs, imap;
	int nObs = m_Features.size();
	int nMap = m_Map->m_nSP;
	int nNew = 0;
	for (iobs = 0; iobs < nObs; iobs++) // for all observations
	{

		splineo = m_Features[iobs]->spline; // current observed spline
		if (splineo == NULL) continue;
		m_Features[iobs]->isnew = true; // supposed to be a new spline
		// Find matching with map elements
		for (imap = 0; imap < nMap; imap++) // for all map splines
		{
			splinem = m_Map->m_Splines[imap]; // current map spline
			float difx, dify, dmin;
			// First, splines proximity (CP comparison)
			int ii, jj;
			for (ii = 0; ii < splineo->m_iNumber; ii++) // observation control points
			{
				for (jj = 0; jj < splinem->m_iNumber; jj++) // map feature control points
				{
					difx = splineo->m_Bx[ii] - splinem->m_Bx[jj];
					dify = splineo->m_By[ii] - splinem->m_By[jj];
					dmin = difx * difx + dify * dify;
					if (dmin < dmincp) {
						ii = splineo->m_iNumber;
						break; // near enough
					}
				}
			}
			float ini_m, fin_m, ini_o, fin_o; // parameters (t)
			match1 = match2 = false;

			ini_o = splineo->m_fRange[0];
			fin_o = splineo->m_fRange[1];
			splineo->Eval(ini_o, IniO, IniO + 1);
			splineo->Eval(fin_o, FinO, FinO + 1);
			if (dmin < dmincp) {
				//////////////////////////////////////
				// Min distance from IniO ////////////
				splinem->Nearest(IniO[0], IniO[1], ini_m, IniM[0], IniM[1], d1);
				// Minimum distance at the beginning or end of map curve
				if ((ini_m == splinem->m_fRange[0]) || (ini_m
						== splinem->m_fRange[1]))
					splineo->Nearest(IniM[0], IniM[1], ini_o, IniO[0], IniO[1],
							d1);
				if (d1 < options.matchtol)
					match1 = true;
				else
					continue; // continue imap
				//////////////////////////////////////
				// Min distance from FinO ////////////
				splinem->Nearest(FinO[0], FinO[1], fin_m, FinM[0], FinM[1], d2);
				// Minimum distance at the beginning or end of map curve
				if ((fin_m == splinem->m_fRange[0]) || (fin_m
						== splinem->m_fRange[1]))
					splineo->Nearest(FinM[0], FinM[1], fin_o, FinO[0], FinO[1],
							d2);
				if (d2 < options.matchtol)
					match2 = true;
				else
					continue; // continue imap

				if (match1 && match2) // both extremes matched
				{
					if ((fin_o > ini_o) && (fin_m > ini_m)) // valid matching
					{
						m_Features[iobs]->isnew = false; // correct matching
						if ((fin_o - ini_o > options.matchmin) && (fin_m
								- ini_m > options.matchmin)) // Kalman filter update
						{
							Link link;
							link.spline = splinem;
							link.Keys[0] = ini_o;
							link.Keys[1] = fin_o;
							link.Keys[2] = ini_m;
							link.Keys[3] = fin_m;
							m_Features[iobs]->links.push_back(link);
						}
						// RIGHT EXTENSION //////////////////////////////////////////////////////////
						if (m_Features[iobs]->linkLE == NULL) // left extension => no right extension
						{

							if (m_Features[iobs]->linkRE == NULL) // for right extension
							{
								if ((splineo->m_fRange[1] - fin_o)
										> options.minext) {
									m_Features[iobs]->linkRE = new Link;
									m_Features[iobs]->linkRE->Keys[0] = ini_o;
									m_Features[iobs]->linkRE->Keys[1] = fin_o;
									m_Features[iobs]->linkRE->Keys[2] = ini_m;
									m_Features[iobs]->linkRE->Keys[3] = fin_m;
									m_Features[iobs]->linkRE->spline = splinem;
								}
							} else {
								if (fin_o > m_Features[iobs]->linkRE->Keys[1]) {
									if ((splineo->m_fRange[1] - fin_o)
											> options.minext) {
										m_Features[iobs]->linkRE->Keys[0]
												= ini_o;
										m_Features[iobs]->linkRE->Keys[1]
												= fin_o;
										m_Features[iobs]->linkRE->Keys[2]
												= ini_m;
										m_Features[iobs]->linkRE->Keys[3]
												= fin_m;
										m_Features[iobs]->linkRE->spline
												= splinem;
									} else {
										delete m_Features[iobs]->linkRE;
										m_Features[iobs]->linkRE = NULL;
									}
								}
							}
						}
						if (m_Features[iobs]->linkRE)
							continue; // right extension => no lefet extension

						// LEFT EXTENSION //////////////////////////////////////////////////////////////
						if (m_Features[iobs]->linkLE == NULL) // for left extension
						{
							if ((ini_o - splineo->m_fRange[0]) > options.minext) {
								m_Features[iobs]->linkLE = new Link;
								m_Features[iobs]->linkLE->Keys[0] = ini_o;
								m_Features[iobs]->linkLE->Keys[1] = fin_o;
								m_Features[iobs]->linkLE->Keys[2] = ini_m;
								m_Features[iobs]->linkLE->Keys[3] = fin_m;
								m_Features[iobs]->linkLE->spline = splinem;
							}
						} else if (ini_o < m_Features[iobs]->linkLE->Keys[0]) {
							if ((ini_o - splineo->m_fRange[0]) > options.minext) {
								m_Features[iobs]->linkLE->Keys[0] = ini_o;
								m_Features[iobs]->linkLE->Keys[1] = fin_o;
								m_Features[iobs]->linkLE->Keys[2] = ini_m;
								m_Features[iobs]->linkLE->Keys[3] = fin_m;
								m_Features[iobs]->linkLE->spline = splinem;
							} else {
								delete m_Features[iobs]->linkLE;
								m_Features[iobs]->linkLE = NULL;
							}

						}
					}
				}
			}
		} // for imap


		if (m_Features[iobs]->isnew)
			nNew++;
		if (m_Features[iobs]->linkRE != NULL) {
			if ((splineo->m_fRange[1] - m_Features[iobs]->linkRE->Keys[1])
					< options.minext) {
				delete m_Features[iobs]->linkRE;
				m_Features[iobs]->linkRE = NULL;
			}
		}
		if (m_Features[iobs]->linkLE != NULL) {
			if ((m_Features[iobs]->linkLE->Keys[0] - splineo->m_fRange[0])
					< options.minext) {
				delete m_Features[iobs]->linkLE;
				m_Features[iobs]->linkLE = NULL;
			}
		}
	} // for iobs

	cout << nNew << " new features detected" << endl;
	return 0;
}

/** Process new splines which are added to the map **/
int CMapBuilder::ProcessNew() {
	SLAMfeature* feat = NULL;
	PoseStruct pose;
	pose.x = m_Robot.x;
	pose.y = m_Robot.y;
	pose.ang = m_Robot.ang;
	//m_Map->GetRobotPose(pose);
	ndata_new = 0;
	int iobs = 0; // counter for feature
	int idata = 0; // counter for data
	int nFeatures = m_Features.size();
	int nData = 0;
	for (iobs = 0; iobs < nFeatures; iobs++) // for all detected features
	{
		feat = m_Features[iobs];
		if (feat->spline == NULL) continue;
		if (feat->isnew) // feature to be added to the map
		{
			cout << "adding new feature\n";
			nData = feat->x.size();
			for (idata = 0; idata < nData; idata++) // data update
			{
				float z = feat->l[idata];
				feat->mu[idata] = pose.ang + feat->tau[idata];
				feat->cosmu[idata] = cos(feat->mu[idata]);
				feat->sinmu[idata] = sin(feat->mu[idata]);
				feat->x[idata] = pose.x + z * feat->cosmu[idata];
				feat->y[idata] = pose.y + z * feat->sinmu[idata];
			}
			//feat->spline->Fit(&feat->x, &feat->y, &feat->t, options.span);
			feat->spline->isnew = true;
			m_Map->AddFeature(feat->spline);

			if ((options.method == M_NONE) || (options.method == M_LOCAL)) {
				feat->spline = NULL;
				cout << "saliendo\n";
				continue; // AND NOTHING ELSE TO BE DONE WITH THIS FEATURE
			}
			ndata_new += nData;
			///////////////////////////////////////////////////////////////////////////////
			// STATE EXTENSION ////////////////////////////////////////////////////////////
			int numcp = feat->spline->m_iNumber; // number of new control points

			NEWMAT::ColumnVector xnew(2 * numcp);
			for (int icp = 0; icp < numcp; icp++) {
				xnew.element(icp) = feat->spline->m_Bx[icp];
				xnew.element(icp + numcp) = feat->spline->m_By[icp];
			}
			dim_new += 2 * numcp;
			// Covariance update ////////////////////////////////////////////////////////
			NEWMAT::Matrix Mx(nData, 3);
			Mx.Column(1) = 1.0;
			Mx.Column(2) = 0.0;
			NEWMAT::Matrix My(nData, 3);
			My.Column(1) = 0.0;
			My.Column(2) = 1.0;
			NEWMAT::DiagonalMatrix Zx(nData);
			NEWMAT::DiagonalMatrix Zy(nData);
			for (idata = 0; idata < nData; idata++) {
				Mx.element(idata, 2) = pose.y - feat->y[idata];
				My.element(idata, 2) = feat->x[idata] - pose.x;
				Zx.element(idata) = feat->cosmu[idata];
				Zy.element(idata) = feat->sinmu[idata];
			}
			feat->GAx = ((*feat->spline->PHI) * Mx) & ((*feat->spline->PHI)
					* My);
			feat->GAz = ((*feat->spline->PHI) * Zx) & ((*feat->spline->PHI)
					* Zy);

			////////////////////////////////////////////////////////////
			feat->spline = NULL;
		}
	}
	return 0;
}

/** Map extensions processing **/
int CMapBuilder::ProcessExt() {
	if (!options.extension)
		return -1; // nothing to do
	int iobs = 0; // detected features counter
	int ndata = 0; // data points counter
	PoseStruct pose;
	m_Map->GetRobotPose(pose); // current robot pose
	for (iobs = 0; iobs < m_Features.size(); iobs++) // for all observed features...
	{
		SLAMfeature* feat = m_Features[iobs]; // current feature
		if (feat->linkLE) // LEFT EXTENSION (LOWER T)
		{
			CBspline* spo = feat->spline; // current detected spline
			CBspline* spm = feat->linkLE->spline; // map spline to be enlarged (left)
			float ini_o = feat->linkLE->Keys[0];
			float fin_o = feat->linkLE->Keys[1];
			float ini_m = feat->linkLE->Keys[2];
			float fin_m = feat->linkLE->Keys[3];

			float length = ini_o + spm->m_Knots[4];
			int n1L = 4; // number of initial left knots
			int n2L = ceil(length / options.span) + 3; // number of final left knots
			int nnew = n2L - n1L; // number of new knots (to be inserted)

			for (int toff = 0; toff < spm->m_Knots.size(); toff++)
				spm->m_Knots[toff] += ini_o;

			for (int tnew = 0; tnew < nnew; tnew++)
				spm->m_Knots.insert(spm->m_Knots.begin() + 4, spm->m_Knots[4]
						- options.span);

			spm->m_Knots[0] = spm->m_Knots[1] = spm->m_Knots[2]
					= spm->m_Knots[3] = 0.0F;
			spm->m_fRange[1] = spm->m_Knots.back();

			std::vector<float> xdata;
			std::vector<float> ydata; // x and y data points
			// new data to be added to the spline
			for (ndata = 0; ndata < feat->x.size(); ndata++) {
				if (feat->t[ndata] < ini_o) {
					feat->mu[ndata] = pose.ang + feat->tau[ndata];
					feat->cosmu[ndata] = cos(feat->mu[ndata]);
					feat->sinmu[ndata] = sin(feat->mu[ndata]);
					float xval = pose.x + feat->l[ndata] * feat->cosmu[ndata];
					float yval = pose.y + feat->l[ndata] * feat->sinmu[ndata];
					xdata.push_back(xval); // extending data (x)
					ydata.push_back(yval); // extending data (y)
				} else
					break;
			}

			std::vector<float> tu(spm->m_Knots.begin() + nnew,
					spm->m_Knots.end()); // unclamped knot vector

			int n1 = spm->m_iNumber; // initial number of cp
			int n2 = n1 + nnew; // final number of cp
			m_Map->m_nCP += nnew;
			spm->m_iNumber = n2;
			spm->m_bCPderiv = false;
			spm->m_bCPderiv2 = false;

			NEWMAT::Matrix L(ndata + n1, ndata + n1); // L matrix (square ndata+n1 x ndata+n1)
			L = 0.0F;
			for (int il = 0; il < ndata + n1; il++)
				L.element(il, il) = 1.0F;
			L.element(ndata, ndata) = 1 / (omega(tu, 4, 0, 1) * omega(tu, 4, 0,
					2));
			L.element(ndata, ndata + 1) = -(Omega(tu, 4, 0, 2) / omega(tu, 4,
					1, 2) + Omega(tu, 4, 0, 1) / omega(tu, 4, 0, 2));
			L.element(ndata, ndata + 2) = Omega(tu, 4, 0, 2) * Omega(tu, 4, 1,
					2);
			L.element(ndata + 1, ndata) = 0.0f;
			L.element(ndata + 1, ndata + 1) = 1 / omega(tu, 4, 1, 2);
			L.element(ndata + 1, ndata + 2) = -Omega(tu, 4, 1, 2);

			NEWMAT::Matrix N(ndata + n1, n2);
			N = 0.0F;
			int iN;
			int jN;
			for (iN = 0; iN < ndata; iN++) {
				for (jN = 0; jN < n2; jN++) {
					N.element(iN, jN) = spm->Basis(feat->t[iN], jN, 4);
				}
			}
			for (iN = 0; iN < n1; iN++)
				N.element(ndata + iN, nnew + iN) = 1.0F;

			feat->PHIext = (N.t() * N).i() * N.t() * L; // PHIext calculation
			NEWMAT::ColumnVector data(ndata + n1);
			int id;
			for (id = 0; id < ndata; id++)
				data.element(id) = xdata[id]; // first data points
			for (id = 0; id < n1; id++)
				data.element(ndata + id) = spm->m_Bx[id]; // second control points
			NEWMAT::ColumnVector xsol = feat->PHIext * data;

			for (id = 0; id < ndata; id++)
				data.element(id) = ydata[id]; // first data points
			for (id = 0; id < n1; id++)
				data.element(ndata + id) = spm->m_By[id]; // second control points
			NEWMAT::ColumnVector ysol = feat->PHIext * data;
			// SPLINE UPDATE ///////////////////////////////////////////////
			spm->m_Bx.clear();
			spm->m_By.clear();
			int isol;
			for (isol = 0; isol < n2; isol++) {
				spm->m_Bx.push_back(xsol.element(isol));
				spm->m_By.push_back(ysol.element(isol));
			}

			if (options.method != M_SLAM)
				//	return 0;
				continue; // nothing else to be done !!!
			/////////////////////////////////////////////////////////////////
			// Covariance update ////////////////////////////////////////////
			dim_ext += nnew * 2; // new state dim
			ndata_ext += ndata; // total number of extending data points

			NEWMAT::Matrix Mx(ndata, 3);
			Mx.Column(1) = 1.0;
			Mx.Column(2) = 0.0;
			NEWMAT::Matrix My(ndata, 3);
			My.Column(1) = 0.0;
			My.Column(2) = 1.0;
			NEWMAT::DiagonalMatrix Zx(ndata);
			NEWMAT::DiagonalMatrix Zy(ndata);
			for (int idata = 0; idata < ndata; idata++) {
				Mx.element(idata, 2) = pose.y - ydata[idata];
				My.element(idata, 2) = xdata[idata] - pose.x;
				Zx.element(idata) = feat->cosmu[idata];
				Zy.element(idata) = feat->sinmu[idata];
			}
			feat->GExr = (feat->PHIext.Columns(1, ndata) * Mx)
					& (feat->PHIext.Columns(1, ndata) * My);
			feat->GExm = NEWMAT::Matrix(n2 * 2, n1 * 2);
			feat->GExm.SubMatrix(1, n2, 1, n1) = feat->PHIext.Columns(
					ndata + 1, ndata + n1);
			feat->GExm.SubMatrix(1, n2, n1 + 1, n1 * 2) = 0.0;
			feat->GExm.SubMatrix(n2 + 1, n2 * 2, 1, n1) = 0.0;
			feat->GExm.SubMatrix(n2 + 1, n2 * 2, n1 + 1, n1 * 2)
					= feat->PHIext.Columns(ndata + 1, ndata + n1);
			feat->GEz = (feat->PHIext.Columns(1, ndata) * Zx)
					& (feat->PHIext.Columns(1, ndata) * Zy);
			spm->isextended = true;
			continue;
		} // if linkLE

		if (feat->linkRE != NULL) // RIGHT EXTENSION (HIGHER T)
		{
			CBspline* spo = feat->spline; // current detected spline
			CBspline* spm = feat->linkRE->spline; // map spline to be enlarged (right)
			float ini_o = feat->linkRE->Keys[0];
			float fin_o = feat->linkRE->Keys[1];
			float ini_m = feat->linkRE->Keys[2];
			float fin_m = feat->linkRE->Keys[3];
			int nt = spm->m_Knots.size(); // initial number of knots

			float length = (spo->m_fRange[1] - fin_o) + (spm->m_Knots[nt - 1]
					- spm->m_Knots[nt - 5]);
			int n1R = 4; // number of initial right knots
			int n2R = ceil(length / options.span) + 3; // number of final right knots
			int nnew = n2R - n1R; // number of new knots (to be inserted)

			int tnew;
			for (tnew = 0; tnew < nnew; tnew++)
				spm->m_Knots.insert(spm->m_Knots.end() - 4, spm->m_Knots[nt - 5
						+ tnew] + options.span);

			float finval = fin_m + (spo->m_fRange[1] - fin_o);
			nt = nt + nnew; // new number of knots
			spm->m_Knots[nt - 1] = spm->m_Knots[nt - 2] = spm->m_Knots[nt - 3]
					= spm->m_Knots[nt - 4] = finval;
			spm->m_fRange[1] = finval;

			std::vector<float> xdata;
			std::vector<float> ydata;
			std::vector<float> param;
			// new data to be added to the spline
			int numdata = feat->x.size();
			for (ndata = 0; ndata < numdata; ndata++) {
				int thisdata = numdata - 1 - ndata;
				if (feat->t[thisdata] > fin_o) {
					float tnew = feat->t[thisdata] - fin_o + fin_m;
					feat->mu[thisdata] = pose.ang + feat->tau[thisdata];
					feat->cosmu[thisdata] = cos(feat->mu[thisdata]);
					feat->sinmu[thisdata] = sin(feat->mu[thisdata]);
					float xval = pose.x + feat->l[thisdata]
							* feat->cosmu[thisdata];
					float yval = pose.y + feat->l[thisdata]
							* feat->sinmu[thisdata];
					xdata.push_back(xval); // x data (y)
					ydata.push_back(yval); // x data (y)
					param.push_back(tnew);
				} else
					break;
			}
			ndata = param.size();

			std::vector<float> tu(spm->m_Knots.begin(), spm->m_Knots.end()
					- nnew); // unclamped knot vector

			int n1 = spm->m_iNumber; // initial number of cp
			int n2 = n1 + nnew; // final number of cp
			spm->m_iNumber = n2;
			m_Map->m_nCP += nnew;
			spm->m_bCPderiv = false;
			spm->m_bCPderiv2 = false;

			int n1m1 = n1 - 1;
			NEWMAT::Matrix R(ndata + n1, ndata + n1); // R matrix (square ndata+n1 x ndata+n1)
			R = 0.0F;
			for (int il = 0; il < ndata + n1; il++)
				R.element(il, il) = 1.0F;
			R.element(ndata + n1 - 1, ndata + n1 - 1) = 1 / (lambda(tu, 4,
					n1m1, 1) * lambda(tu, 4, n1m1, 2));
			R.element(ndata + n1 - 1, ndata + n1 - 2)
					= -(Lambda(tu, 4, n1m1, 2) / lambda(tu, 4, n1m1 - 1, 2)
							+ Lambda(tu, 4, n1m1, 1) / lambda(tu, 4, n1m1, 2));
			R.element(ndata + n1 - 1, ndata + n1 - 3) = Lambda(tu, 4, n1m1, 2)
					* Lambda(tu, 4, n1m1 - 1, 2);
			R.element(ndata + n1 - 2, ndata + n1 - 1) = 0.0f;
			R.element(ndata + n1 - 2, ndata + n1 - 2) = 1 / lambda(tu, 4, n1m1
					- 1, 2);
			R.element(ndata + n1 - 2, ndata + n1 - 3) = -Lambda(tu, 4,
					n1m1 - 1, 2);

			NEWMAT::Matrix N(ndata + n1, n2);
			N = 0.0F;
			for (int iN = 0; iN < ndata; iN++) {
				for (int jN = 0; jN < n2; jN++) {
					float tt = param[iN];
					N.element(iN, jN) = spm->Basis(param[iN], jN, 4);
				}
			}
			int iN;
			for (iN = 0; iN < n1; iN++)
				N.element(ndata + iN, iN) = 1.0F;

			feat->PHIext = (N.t() * N).i() * N.t() * R; // PHIext calculation
			NEWMAT::ColumnVector data(ndata + n1);
			int id;
			for (id = 0; id < ndata; id++)
				data.element(id) = xdata[id]; // first data points
			for (id = 0; id < n1; id++)
				data.element(ndata + id) = spm->m_Bx[id]; // second control points
			NEWMAT::ColumnVector xsol = feat->PHIext * data;

			for (id = 0; id < ndata; id++)
				data.element(id) = ydata[id]; // first data points
			for (id = 0; id < n1; id++)
				data.element(ndata + id) = spm->m_By[id]; // second control points
			NEWMAT::ColumnVector ysol = feat->PHIext * data;
			// SPLINE UPDATE ///////////////////////////////////////////////
			spm->m_Bx.clear();
			spm->m_By.clear();
			int isol;
			for (isol = 0; isol < n2; isol++) {
				spm->m_Bx.push_back(xsol.element(isol));
				spm->m_By.push_back(ysol.element(isol));
			}

			if (options.method != M_SLAM)
				//	return 0;
				continue; // nothing else to be done !!!
			/////////////////////////////////////////////////////////////////
			// Covariance update ////////////////////////////////////////////
			dim_ext += nnew * 2; // new state dim
			ndata_ext += ndata; // total number of extending data points

			NEWMAT::Matrix Mx(ndata, 3);
			Mx.Column(1) = 1.0;
			Mx.Column(2) = 0.0;
			NEWMAT::Matrix My(ndata, 3);
			My.Column(1) = 0.0;
			My.Column(2) = 1.0;
			NEWMAT::DiagonalMatrix Zx(ndata);
			NEWMAT::DiagonalMatrix Zy(ndata);
			int idata;
			for (idata = 0; idata < ndata; idata++) {
				int thisdata = numdata - 1 - idata;
				Mx.element(idata, 2) = pose.y - ydata[idata];
				My.element(idata, 2) = xdata[idata] - pose.x;
				Zx.element(idata) = feat->cosmu[thisdata];
				Zy.element(idata) = feat->sinmu[thisdata];
			}
			feat->GExr = (feat->PHIext.Columns(1, ndata) * Mx)
					& (feat->PHIext.Columns(1, ndata) * My);
			feat->GExm = NEWMAT::Matrix(n2 * 2, n1 * 2);
			feat->GExm.SubMatrix(1, n2, 1, n1) = feat->PHIext.Columns(
					ndata + 1, ndata + n1);
			feat->GExm.SubMatrix(1, n2, n1 + 1, n1 * 2) = 0.0;
			feat->GExm.SubMatrix(n2 + 1, n2 * 2, 1, n1) = 0.0;
			feat->GExm.SubMatrix(n2 + 1, n2 * 2, n1 + 1, n1 * 2)
					= feat->PHIext.Columns(ndata + 1, ndata + n1);
			feat->GEz = (feat->PHIext.Columns(1, ndata) * Zx)
					& (feat->PHIext.Columns(1, ndata) * Zy);
			spm->isextended = true;
		} // if linkRE
	} // for iobs

	return 0;
}

/** omega function for spline extension **/
Real CMapBuilder::omega(const std::vector<float> &t, int k, int i, int j) {
	return (t[k - 1] - t[i + k]) / (t[i + k - j - 1] - t[i + k]);
}

/** Omega function for spline extension **/
Real CMapBuilder::Omega(const std::vector<float> &t, int k, int i, int j) {
	float w = omega(t, k, i, j);
	return (1 - w) / w;
}

/** lambda function for spline extension **/
Real CMapBuilder::lambda(const std::vector<float> &t, int k, int i, int j) {
	int n = t.size() - k - 1;
	return (t[n + 1] - t[i]) / (t[i + j + 1] - t[i]);
}

/** Lambda function for spline extension **/
Real CMapBuilder::Lambda(const std::vector<float> &t, int k, int i, int j) {
	float l = lambda(t, k, i, j);
	return (1 - l) / l;
}

/** Retrieves map building options **/
BuildOptions CMapBuilder::GetOptions() {
	return options;
}

/** Configures map building options **/
int CMapBuilder::SetOptions(BuildOptions opt) {
	options = opt;
	return 0;
}

/** Submaps Management **/
int CMapBuilder::ProcessSubmaps() {
	if (!options.submaps)
		return -1;
	int dim1 = xSM.Nrows();
	if (dim1 == 0) // first submap in the state => init.
	{
		xSM.ReSize(3);
		xSM.element(0) = m_Map->PosAbs.x;
		xSM.element(1) = m_Map->PosAbs.y;
		xSM.element(2) = m_Map->PosAbs.ang;
		PSM.ReSize(3);
		PSM = m_Map->PosAbs.Sigma;
	}

	if (m_Map->m_nCP >= options.submapSize) // new submaps is created
	{
		PoseStruct pose;
		m_Map->GetRobotPose(pose);
		CUncertainRel posRelMap; // pose relativa del nuevo mapa
		posRelMap.x = pose.x;
		posRelMap.y = pose.y;
		posRelMap.ang = pose.ang;
		posRelMap.Sigma = m_Map->P.SymSubMatrix(1, 3);
		/////////////////////////////////////////////////////////////////////////////
		NEWMAT::Matrix J1(3, 3), J2(3, 3);
		CMapSP* newMap = new CMapSP;
		newMap->idMap = m_Map->idMap + 1; // id of the new submap
		newMap->PosRel = posRelMap;
		newMap->PosAbs = Compound(m_Map->PosAbs, posRelMap, J1, J2);
		//newMap->PosAbs = m_Map->PosAbs + posRelMap;
		newMap->m_Sim.push_back(PoseStruct(0, 0, 0));
		m_Map->next = newMap;
		m_Map = newMap;
		m_Mlist.push_back(m_Map);
		/////////////////////////////////////////////////////////////////////////////
		// state enlargement
		xSM.Release();
		NEWMAT::ColumnVector xtemp = xSM;
		xSM.ReSize(dim1 + 3);
		xSM.Rows(1, dim1) = xtemp;
		xSM.element(dim1) = m_Map->PosAbs.x;
		xSM.element(dim1 + 1) = m_Map->PosAbs.y;
		xSM.element(dim1 + 2) = m_Map->PosAbs.ang;

		NEWMAT::Matrix Gx(dim1 + 3, dim1);
		Gx = 0.0f;
		for (int ii = 0; ii < dim1; ii++)
			Gx.element(ii, ii) = 1.0f;
		Gx.SubMatrix(dim1 + 1, dim1 + 3, dim1 - 2, dim1) = J1;
		NEWMAT::Matrix Gz(dim1 + 3, 3);
		Gz = 0.0f;
		Gz.SubMatrix(dim1 + 1, dim1 + 3, 1, 3) = J2;

		NEWMAT::SymmetricMatrix Ma;
		NEWMAT::SymmetricMatrix Mb;
		Ma << Gx * PSM * Gx.t();
		Mb << Gz * posRelMap.Sigma * Gz.t();
		PSM << (Ma + Mb);
		///////////// map init
		Clean();
		//ProcessLaser();
	}
	return 0;
}

CMapBuilder::CMapBuilder(BuildOptions opt) {
	options = opt;
}

int CMapBuilder::ProcessError() {
	if (!options.datasim) // only when using data from simulation
		return -1;
	PoseStruct error;
	CUncertainRel posRel;
	posRel.x = m_Map->x.element(0);
	posRel.y = m_Map->x.element(1);
	posRel.ang = m_Map->x.element(2);
	CUncertainRel posAbs = m_Map->PosAbs + posRel;

	error.x = posAbs.x - m_Mlist[0]->m_Sim.back().x;
	error.y = posAbs.y - m_Mlist[0]->m_Sim.back().y;
	error.ang = AngError(m_Mlist[0]->m_Sim.back().ang, posAbs.ang);
	m_Map->m_Error.push_back(error); // localization error
	PoseStruct sigmas(sqrt(m_Map->P.element(0, 0)),
			sqrt(m_Map->P.element(1, 1)), sqrt(m_Map->P.element(2, 2)));
	m_Map->m_Sigmas.push_back(sigmas); // sigma_x, sigma_y, sigma_th
	/////////////////////////////////////////////////////
	// NEES /////////////////////////////////////////////
	if (m_iSamples > 1) {
		NEWMAT::SymmetricMatrix Ptemp(3);
		Ptemp = m_Map->P.SymSubMatrix(1, 3);
		/*		NEWMAT::Matrix P_(3,3);
		 P_ = 0.0f;
		 P_.element(0,0) = m_Map->P.element(0,0);
		 P_.element(1,1) = m_Map->P.element(1,1);
		 P_.element(2,2) = m_Map->P.element(2,2);
		 P_.element(0,1) = P_.element(1,0) = m_Map->P.element(0,1);
		 P_.element(0,2) = P_.element(2,0) = m_Map->P.element(0,2);
		 P_.element(1,2) = P_.element(2,1) = m_Map->P.element(1,2);
		 */
		NEWMAT::ColumnVector errorMahala(3);
		errorMahala.element(0) = error.x;
		errorMahala.element(1) = error.y;
		errorMahala.element(2) = error.ang;
		NEWMAT::ColumnVector em(1);
		em = errorMahala.t() * (Ptemp.i() * errorMahala);
		m_Map->m_dMahala.push_back((float) em.element(0));
		// alternative
		//float emmm = error.x * error.x / m_Map->P.element(0,0) + error.y * error.y / m_Map->P.element(1,1) + error.ang * error.ang / m_Map->P.element(2,2);
		//m_Map->m_dMahala.push_back(emmm);

		em = errorMahala.t() * errorMahala;
		m_Map->precision.push_back(em.element(0));
		//		Ptemp = 0.0;
		//		Ptemp.element(0,0) = m_Mlist[0]->P.element(0,0);
		//		Ptemp.element(1,1) = m_Mlist[0]->P.element(1,1);
		//		Ptemp.element(2,2) = m_Mlist[0]->P.element(2,2);
		//		em = errorMahala.t() * (Ptemp.i() * errorMahala);
		//		m_Mlist[0]->m_dMahala.push_back(em.element(0));
		//		m_Map->P_values.push_back(P_);
		//		m_Map->Pinv_values.push_back(P_inv);
	}
	return 0;
}

/** Cleaning function (observations) **/
int CMapBuilder::Clean() {
	int cont;
	for (cont = 0; cont < m_Features.size(); cont++) {
		if (m_Features[cont]->spline)
			delete m_Features[cont]->spline;
		if (m_Features[cont]->linkRE)
			delete m_Features[cont]->linkRE;
		if (m_Features[cont]->linkLE)
			delete m_Features[cont]->linkLE;
		delete m_Features[cont];
	}
	m_Features.clear();
	return 0;
}

void CMapBuilder::LogKalmanUpdate(int Iter, int Samples, NEWMAT::Matrix &Gain) {
	if (!options.savelog)
		return;
	fprintf(
			logfile,
			"%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n");
	fprintf(logfile,
			"%i Update Iteration, %i Samples used in Kalman Filter\n\n", Iter,
			Samples);
	fprintf(logfile, "mu \t\t\tdh/dx \t\tdh/dy \t\tdh/dth \t\tresidual \n");
	for (int io = 0; io < m_Features.size(); io++) {
		SLAMfeature* feat = m_Features[io];
		for (int imed = 0; imed < feat->res.size(); imed++) {
			fprintf(logfile, "%.5f \t\t%.5f \t\t%.5f \t\t%.5f \t\t%.5f\n",
					feat->mu[imed], feat->dhdx[imed], feat->dhdy[imed],
					feat->dhdth[imed], feat->res[imed]);
		}
	}
	fprintf(logfile, "W MAX VALUE: %f, \t\tW MIN VALUE: %f\n", Gain.Maximum(),
			Gain.Minimum());
}

/** Position of the spline on the state vector (starting from zero) **/
int CMapBuilder::GetSplinePos(CBspline *spline) {
	int result = 3; // position of first spline
	int spindex = 0;
	CBspline* sp = m_Map->m_Splines[spindex];
	while (sp != spline) {
		result += (2 * sp->m_iNumber);
		spindex++;
		sp = m_Map->m_Splines[spindex];
	}
	return result;
}

/* Copies map state vector to splines control points */
void CMapBuilder::CopyState2Map() {
	if (options.method != M_SLAM)
		return; // nothing needs to be done

	int index = 3;
	for (int iobj = 0; iobj < m_Map->m_Splines.size(); iobj++) {
		CBspline* sp = m_Map->m_Splines[iobj];
		int nCP = sp->m_iNumber;
		for (int icp = 0; icp < nCP; icp++) {
			sp->m_Bx[icp] = m_Map->x.element(index);
			sp->m_By[icp] = m_Map->x.element(index + nCP);
			sp->m_bCPderiv = false;
			sp->m_bCPderiv2 = false;
			index++;
		}
		index += nCP;
	}
}

void CMapBuilder::LogMatrix(NEWMAT::Matrix &m, const char *filename) {
	ofstream file(filename, ios::out);
	for (int i = 0; i < m.Nrows(); i++) {
		for (int j = 0; j < m.Ncols(); j++) {
			file << m.element(i, j) << " ";
		}
		file << "\n";
	}
	file.close();
}

void CMapBuilder::LogSymMatrix(NEWMAT::SymmetricMatrix &m, const char* filename) {
	ofstream file(filename, ios::out);
	int dim = m.Nrows();
	for (int i = 0; i < dim; i++) {
		for (int j = 0; j < dim; j++) {
			file << m.element(i, j) << " ";
		}
		file << "\n";
	}
	file.close();
}

int CMapBuilder::EKF_Enlarge() {
	if (options.method != M_SLAM)
		return -1;

	int numdata = ndata_ext + ndata_new;
	if (numdata == 0)
		return -1; // nothing to do
	NEWMAT::Matrix Gz(dim_new, numdata);
	Gz = 0.0;
	NEWMAT::Matrix Gx(dim_new, dim_old);
	Gx = 0.0;
	Gx.SubMatrix(1, 3, 1, 3) = NEWMAT::IdentityMatrix(3); // robot

	int imap = 0; // counter variable
	int iobs = 0; // counter variable
	SLAMfeature* feat = NULL;
	CBspline *sp = NULL;
	//////////////////////////////////////////////////////////////////////////
	// NEW FEATURES and EXTENSIONS PROCESSING ////////////////////////////////
	int posrow = 4;
	int poscolGx = 4;
	int poscolGz = 1;
	// extended features processing //////////////////////////////////////////
	for (imap = 0; imap < m_Map->m_Splines.size(); imap++) {
		sp = m_Map->m_Splines[imap];
		if (sp->isnew) {
			sp->isnew = false;
			continue;
		}
		if (sp->isextended) {
			iobs = 0;
			while (true) {
				if (m_Features[iobs]->linkLE) {
					if (m_Features[iobs]->linkLE->spline == sp)
						break;
				}
				if (m_Features[iobs]->linkRE) {
					if (m_Features[iobs]->linkRE->spline == sp)
						break;
				}
				iobs++;
			}
			SLAMfeature *feat = m_Features[iobs];
			Gx.SubMatrix(posrow, posrow + feat->GExr.Nrows() - 1, 1, 3)
					= feat->GExr;
			Gx.SubMatrix(posrow, posrow + feat->GExm.Nrows() - 1, poscolGx,
					poscolGx + feat->GExm.Ncols() - 1) = feat->GExm;
			Gz.SubMatrix(posrow, posrow + feat->GEz.Nrows() - 1, poscolGz,
					poscolGz + feat->GEz.Ncols() - 1) = feat->GEz;
			posrow += feat->GExr.Nrows();
			poscolGx += feat->GExm.Ncols();
			poscolGz += feat->GEz.Ncols();
			sp->isextended = false;
		} else {
			int dim = sp->m_iNumber * 2;
			Gx.SubMatrix(posrow, posrow + dim - 1, poscolGx, poscolGx + dim - 1)
					= NEWMAT::IdentityMatrix(dim);
			posrow += dim;
			poscolGx += dim;
		}
	}
	// new features processing ////////////////////////////////////////
	for (iobs = 0; iobs < m_Features.size(); iobs++) // for all detected features
	{
		feat = m_Features[iobs];
		if (feat->isnew) // NEW FEATURES
		{
			Gx.SubMatrix(posrow, posrow + feat->GAx.Nrows() - 1, 1, 3)
					= feat->GAx;
			Gz.SubMatrix(posrow, posrow + feat->GAz.Nrows() - 1, poscolGz,
					poscolGz + feat->GAz.Ncols() - 1) = feat->GAz;
			posrow += feat->GAx.Nrows();
			poscolGz += feat->GAz.Ncols();
		}
	}
	//////////////////////////////////////////////////////////
	// matrix update /////////////////////////////////////////
	NEWMAT::IdentityMatrix R(numdata);
	R = options.sigmaL * options.sigmaL;
	NEWMAT::SymmetricMatrix Ma;
	NEWMAT::SymmetricMatrix Mb;
	Ma << Gx * m_Map->P * Gx.t();
	Mb << Gz * R * Gz.t();
	m_Map->P << (Ma + Mb);
	// Cleaning 
	ndata_ext = 0;
	ndata_new = 0;
	dim_old = 0;
	dim_ext = 0;
	dim_new = 0;
	return 0;
}

void CMapBuilder::CopyMap2State() {
	if (options.method != M_SLAM)
		return; // nothing needs to be done
	int iobj = 0;
	int dim = 0;

	for (iobj = 0; iobj < m_Map->m_Splines.size(); iobj++)
		dim += m_Map->m_Splines[iobj]->m_iNumber;
	m_Map->m_nCP = dim; // Number of control points in the map
	dim = 2 * dim + 3; // state vector dim.
	float x = m_Map->x.element(0);
	float y = m_Map->x.element(1);
	float fi = m_Map->x.element(2);
	m_Map->x.ReSize(dim);
	m_Map->x.element(0) = x;
	m_Map->x.element(1) = y;
	m_Map->x.element(2) = fi;

	int index = 3;
	for (iobj = 0; iobj < m_Map->m_Splines.size(); iobj++) {
		CBspline* sp = m_Map->m_Splines[iobj];
		int nCP = sp->m_iNumber;
		for (int icp = 0; icp < nCP; icp++) {
			m_Map->x.element(index) = sp->m_Bx[icp];
			m_Map->x.element(index + nCP) = sp->m_By[icp];
			index++;
		}
		index += nCP;
	}
}

void CMapBuilder::ProcessLaser(std::vector<float> &scan, float &rmin,
		float &rmax, float &amin, float &amax, float &step) {
	//	if (m_iSamples == 0) return; // nothing to be done
	m_Robot.m_Laser.SetScan(scan, rmin, rmax, amin, amax, step);
	struct timeb t1, t2;
	ftime(&t1); // initial time acquisition
	Prepro(); // Laser data pre-processing
	Aprox();
	Match();
	switch (options.method) {
	case M_NONE:
		break;
	default:
		EKF_Update(); // Stochastic map update
	}
	// MAP ENLARGEMENT ///////////////////////////////
	dim_old = m_Map->x.Nrows(); // old state dimension
	dim_ext = dim_old;
	//ProcessExt(); // Object extensions
	dim_new = dim_ext;

	ProcessNew(); // New objects to be added
	//	 CopyMap2State();
	//	 EKF_Enlarge();

	//m_Map->m_N.push_back(m_Map->m_nCP);
	//	 ProcessError(); // For consistency tests
	//	 ProcessSubmaps();
	Clean();
	/*
	 ftime(&t2); // final time acquisition
	 timeUpdt = (t2.time - t1.time) * 1000 + (t2.millitm - t1.millitm); // elapsed time for update
	 buildtime += timeUpdt; // total time update
	 m_Map->m_Tupdt.push_back(timeUpdt);
	 */

}

int CMapBuilder::OdometryXY() {
	// Offset calculations (LASER POSITION in Urbano robot)
	float ang = m_Robot.ang;
	float x = m_Robot.x + options.offset * cos(ang);
	float y = m_Robot.y + options.offset * sin(ang);
	if (m_iSamples == 0) {
		poseZero.x = x;
		poseZero.y = y;
		poseZero.ang = ang;
	}
	x -= poseZero.x;
	y -= poseZero.y; //ang = ForceInRange(ang - poseZero.ang);
	// Odometric trajectory //////////////////////////////////////
	PoseStruct odompos(x, y, ang);
	if (m_iSamples > 0) // not the first sample
	{
		CSpatialRel xik(m_Robot.x, m_Robot.y, m_Robot.ang);
		CSpatialRel xij(odomOld.x, odomOld.y, odomOld.ang);
		CSpatialRel xjk = -xij + xik;
		deltaX = xjk.x;
		deltaY = xjk.y;
		deltaTH = ForceInRange(xjk.ang);
		m_fSigmaX = options.sigmaXmin + (options.sigmaX / 100) * deltaX; // x sigma
		m_fSigmaY = options.sigmaYmin + (options.sigmaY / 100) * deltaY; // y sigma
		m_fSigmaS = options.sigmaSmin + (options.sigmaS / 100) * deltaTH; // ang sigma
		if (options.addnoise) {
			deltaX = deltaX + randX.RandomNormal(0, m_fSigmaX, true);
			deltaY = deltaY + randY.RandomNormal(0, m_fSigmaY, true);
			deltaTH = deltaTH + randTH.RandomNormal(0, m_fSigmaS, true);
		}
	}
	odomOld.x = m_Robot.x;
	odomOld.y = m_Robot.y;
	odomOld.ang = m_Robot.ang;
	if (options.datasim) /// If simulated data...
	{
		m_Mlist[0]->m_Sim.push_back(odompos); // New simulated sample added
		if (m_iSamples > 0) {
			PoseStruct oldpose = m_Mlist[0]->m_Odom.back();
			float oldTH = oldpose.ang;
			float newTH = ForceInRange(oldTH + deltaTH);
			// Odometry calculation
			odompos.x = oldpose.x + options.offset * (cos(newTH) - cos(oldTH))
					+ deltaX * cos(oldTH) - deltaY * sin(oldTH);
			odompos.y = oldpose.y + options.offset * (sin(newTH) - sin(oldTH))
					+ deltaX * sin(oldTH) + deltaY * cos(oldTH);
			odompos.ang = newTH;
		}
	}
	m_Mlist[0]->m_Odom.push_back(odompos); // New odometry sample added
	m_iSamples++;
	return 0;
}

int CMapBuilder::EKF_PredictXY() {
	if (m_iSamples == 1) // First odometry sample
	{
		PoseStruct newpose(m_Mlist[0]->m_Odom[0].x, m_Mlist[0]->m_Odom[0].y,
				m_Mlist[0]->m_Odom[0].ang);
		m_Map->m_Estim.push_back(newpose);
		m_Map->SetRobotPose(newpose); // Pose initialization
		return 0;
	}

	PoseStruct oldpose;
	m_Map->GetRobotPose(oldpose);
	float oldTH = oldpose.ang;
	float newTH = ForceInRange(oldTH + deltaTH);
	////////////////////////////////////////////////
	// State prediction ////////////////////////////
	PoseStruct newpose(oldpose.x + options.offset * (cos(newTH) - cos(oldTH))
			+ deltaX * cos(oldTH) - deltaY * sin(oldTH), oldpose.y
			+ options.offset * (sin(newTH) - sin(oldTH)) + deltaX * sin(oldTH)
			+ deltaY * cos(oldTH), newTH);

	m_Map->SetRobotPose(newpose);
	m_Map->m_Estim.push_back(newpose);
	////////////////////////////////////////////////
	// Covariance matrix calculation ///////////////
	NEWMAT::Matrix Fx = NEWMAT::IdentityMatrix(m_Map->x.Nrows());
	Fx.element(0, 2) = oldpose.y - newpose.y;
	Fx.element(1, 2) = newpose.x - oldpose.x;

	NEWMAT::Matrix Fu(m_Map->x.Nrows(), 3);
	Fu = 0.0;
	Fu.element(0, 0) = cos(oldTH);
	Fu.element(0, 1) = -sin(oldTH);
	Fu.element(1, 0) = sin(oldTH);
	Fu.element(1, 1) = cos(oldTH);

	Fu.element(0, 2) = -options.offset * sin(newTH);
	Fu.element(1, 2) = options.offset * cos(newTH);

	Fu.element(2, 2) = 1.0;

	NEWMAT::DiagonalMatrix Qds(3);
	Qds.element(0) = m_fSigmaX * m_fSigmaX;
	Qds.element(1) = m_fSigmaY * m_fSigmaY;
	Qds.element(2) = m_fSigmaS * m_fSigmaS;

	NEWMAT::SymmetricMatrix Ma;
	Ma << Fx * m_Map->P * Fx.t();
	NEWMAT::SymmetricMatrix Mb;
	Mb << Fu * Qds * Fu.t();

	m_Map->P << Ma + Mb;
	return 0;
}

void CMapBuilder::LogError() {
	/*
	 CString stre;
	 stre = CString(Exception::what());
	 std::ofstream error("c:/error.log", ios::app);
	 error << stre;
	 error << endl;
	 error.close();
	 */
}
