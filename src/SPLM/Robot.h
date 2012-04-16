/*
 * Robot.h
 *
 *  Created on: Apr 19, 2011
 *      Author: luispedraza
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "Laser.h"

class CRobot {
public:
	CRobot();
	virtual ~CRobot();
	CLaser m_Laser;
	float x;
	float y;
	float ang;
private:

};

#endif /* ROBOT_H_ */
