/*
 * Laser.h
 *
 *  Created on: Apr 19, 2011
 *      Author: Luis Pedraza
 */

#ifndef LASER_H_
#define LASER_H_

#include <vector>

class CLaser {
public:
	// Functions:
	int SetScan(std::vector<float> &scan, float &rmin, float &rmax, float &amin, float &amax, float &astep);
	CLaser();
	virtual ~CLaser();
	// Data
	int nData;
	bool isConfigured;
	float rangeMax;
	float rangeMin;
	float angMax;
	float angMin;
	float step;
	std::vector<float> ranges;
	std::vector<float> tau;
	std::vector<float> costau;
	std::vector<float> sintau;
	//std::vector<float> alfa;
	std::vector<float> x;
	std::vector<float> y;
};

#endif /* LASER_H_ */
