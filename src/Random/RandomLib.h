//	Parameters for the generation of uniform deviates (linear congruential method)		*****
//	And other nonuniformly distributed random numbers									*****
//	***** Described in: [1] D. Knuth, "Seminumerical Algorithms, vol. 2"				*****
//	***** of "The Art of Computer Programming" (Reading Mass.:Addison-Wesley, 1981)		*****
//	Given an initial (seed) integer value, I1, a sequence of integers, I, is generated using the mapping:
//	Inew = (A * Iold + C) mod M
//	The uniform deviate is computed as R = In/M

#ifndef __RANDOM_LIB_
#define	__RANDOM_LIB_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <vector>

#define	A	202		
#define	C	0		
#define	M	1000000	

//#define A	16807.0			
//#define C	0		
//#define M	2147483647.0

#ifndef PI
	#define PI	3.14159265	
#endif

unsigned long GenerateSeed(void);
double Random(void);									// Linear Congruential Method
double* RandomBuffer(double *samples, int number);
std::vector<double> *RandomVector(std::vector<double> *samples, int number);

double RandomAB(double a, double b);
double* RandomBufferAB(double *samples, int number, double a, double b);
std::vector<double> *RandomVectorAB(std::vector<double> *samples, int number, double a, double b);

double RandomTriangAB(double a, double b);
double* RandomTriangBufferAB(double *samples, int number, double a, double b, double c);
std::vector<double> *RandomTriangVectorAB(std::vector<double> *samples, int number, double a, double b, double c);

double RandomTriangABC(double a, double b, double c);
double* RandomTriangBufferABC(double *samples, int number, double min, double mid, double max);
std::vector<double> *RandomTriangVectorABC(std::vector<double> *samples, int number, double min, double mid, double max);

double RandomExp(double lambda);
double* RandomExpBuffer(double *samples, int number, double lambda);
std::vector<double> *RandomExpVector(std::vector<double> *samples, int number, double lambda);

double RandomNormal(bool bound=false);		// Box-Muller transformation
double* RandomNormalBuffer(double *samples, int number, bool bound=false);
std::vector<double> *RandomNormalVector(std::vector<double> *samples, int number, bool bound=false);

double RandomNormal(double mu, double sigma, bool bound=false);
double* RandomNormalBuffer(double *samples, int number, double mu, double sigma, bool bound=false);
std::vector<double> *RandomNormalVector(std::vector<double> *samples, int number, double mu, double sigma, bool bound=false);


// Testing functions:
void Random_rand_Comparison(void);
void TestRandomAB(int number, double min, double max);
void TestRandomTriangAB(int number, double min, double max);
void TestRandomTriangABC(int number, double min, double mid, double max);
void TestRandomNormal(int number);
void TestRandomNormal2(int number, double mu, double sigma);
void TestRandomExp(int number, double lambda);

#endif
