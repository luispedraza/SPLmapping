/*
 * Laser.cpp
 *
 *  Created on: Apr 19, 2011
 *      Author: Luis Pedraza
 */

#include "Laser.h"
#include <math.h>
#include <iostream>

CLaser::CLaser() {
	nData = 0;
	isConfigured = false;
	rangeMax = 0.0f;
	rangeMin = 0.0f;
	angMax = 0.0f;
	angMin = 0.0f;
	step = 0.0f;
}

CLaser::~CLaser() {
	// TODO Auto-generated destructor stub
}

int CLaser::SetScan(std::vector<float> &scan, float &rmin, float &rmax,
		float &amin, float &amax, float &astep) {
	int i;
	if (!isConfigured) {
		nData = scan.size();
		rangeMin = rmin;
		rangeMax = rmax;
		angMin = amin;
		angMax = amax;
		step = astep;
		// Pre-compute tau angles, sin, and cos...
		tau.resize(nData);
		costau.resize(nData);
		sintau.resize(nData);
		for (i = 0; i < nData; i++) {
			tau[i] = amin + i * astep;
			costau[i] = cos(tau[i]);
			sintau[i] = sin(tau[i]);
		}
		x.resize(nData);
		y.resize(nData);
		ranges.resize(nData);
		isConfigured = true;
	}

	ranges = scan;
	for (i = 0; i < nData; i++)
	{
		x[i] = ranges[i] * costau[i];
		y[i] = ranges[i] * sintau[i];
	}
}
