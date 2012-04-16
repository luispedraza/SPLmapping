/*
 * Utils.h
 *
 *  Created on: Apr 17, 2011
 *      Author: Luis Pedraza
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <time.h>
#include <stdio.h>

#ifndef PI
	#define PI 3.141592654
#endif
#ifndef TWOPI
	#define TWOPI 6.283185307
#endif


#define TRACE(ARG) cout << ARG << endl
#define max(a, b) (a>b) ? a : b
#define min(a, b) (a<b) ? a : b




/** Forces an angular variable in the range 0-2pi **/
/*
float ForceInRange(float angle)
{
	float inrange = angle;
	while(inrange >= TWOPI) inrange-= TWOPI;
	while(inrange < 0) inrange += TWOPI;
	return 	inrange;
}

float AngError (float ang2, float ang1)
{
	float delta = ang2-ang1;
	return delta;
}
*/

/*
clock_t launch;

#define TIC() launch = clock()
#define TOC() double diff = (clock() - launch) / CLOCKS_PER_SEC
#define TOCOUT(x) cout << ((clock() - launch) / CLOCKS_PER_SEC) << x << endl
*/

#endif /* UTILS_H_ */
