#include "RandomLib.h"

unsigned long seed = 0;	///< Seed for random numbers generation
bool init = false;		///< Indicates whether seed has been generated or not

/********************************************************/
/*	Function GenerateSeed(void)							*/
/*	Generates a seed for random numbers generation		*/
/********************************************************/
unsigned long GenerateSeed(void)
{
	if (!init)	// seed generation
	{
		struct timeb myTime;
		ftime(&myTime);
		seed = (unsigned long)myTime.millitm;
		init = true;
	}
	return seed;
}

/********************************************************/
/*	Function Random(unsigned long *seed)				*/
/*	Generates random numbers with a given seed			*/
/*	from an Uniform Distribution in the range [0,1]		*/
/*	Returns a random number if successful,				*/
/*	-1 if an error occurs								*/
/********************************************************/
double Random(void)
{
	GenerateSeed();
	seed = (unsigned long)(fmod(A * seed + C, M));
	return (double)seed / M;
}

/************************************************************************/
/*	Function RandomBuffer(double *samples, unsigned int number)			*/	
/*	Generates "number" random samples									*/
/*	from an Uniform Distribution in the range [0,1]						*/
/*	Returns a pointer to the samples if successful						*/
/*	NULL if an error occurs												*/
/************************************************************************/
double* RandomBuffer(double *samples, int number)
{
	if (number <= 0) return NULL;
	if (samples == NULL) 	
		samples = (double*)malloc(number * sizeof(double));
	// Seed generation from the current time (milliseconds)
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples[i] = Random();
	}
	return samples;
}

std::vector<double>* RandomVector(std::vector<double> *samples, int number)
{
	if (number<=0) return NULL;
	if (samples == NULL)
		samples = new std::vector<double>(number);
	else
		samples->resize(number);
	// Seed generation from the current time (milliseconds)
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples->at(i) = Random();
	}
	return samples;
}

/********************************************************/
/*	Function RandomAB(unsigned long *seed)				*/
/*	Generates random numbers with a given seed			*/
/*	from an Uniform Distribution in the range [a,b]		*/
/*	Returns a random number if successful,				*/
/*	-1 if an error occurs								*/
/********************************************************/
double RandomAB(double a, double b)
{
	GenerateSeed();
	return (double)(a + (b-a)*Random());
}

/********************************************************************************/
/*	Function RandomBufferAB(double *samples, unsigned int number, int a, int b)	*/	
/*	Generates "number" random samples											*/
/*	from an Uniform Distribution in the range [a,b]								*/
/*	Returns a pointer to the samples if successful								*/
/*	NULL if an error occurs														*/
/********************************************************************************/
double* RandomBufferAB(double *samples, int number, double a, double b)
{
	if (number <= 0) return NULL;
	if (samples == NULL) 	
		samples = (double*)malloc(number * sizeof(double));
	// Seed generation from the current time (milliseconds)
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples[i] = RandomAB(a, b);
	}
	return samples;
}
std::vector<double> *RandomVectorAB(std::vector<double> *samples, int number, double a, double b)
{
	if (number<=0) return NULL;
	if (samples == NULL)
		samples = new std::vector<double>(number);
	else
		samples->resize(number);
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples->at(i) = RandomAB(a, b);
	}
	return samples;
}

/************************************************************************/
/*	Function RandomTriangAB(unsigned long *seed, double a, double b)	*/
/*	Generates random numbers with a given seed							*/
/*	from an TRIANGULAR Distribution in the range [a,b]					*/
/*	Returns a random number if successful,								*/
/*	-1 if an error occurs												*/
/************************************************************************/
double RandomTriangAB(double a, double b)
{
	GenerateSeed();
	return (RandomAB(a/2, b/2) + RandomAB(a/2, b/2));
}

/************************************************************************************/
/*	Function RandomTriangBufferAB(double *samples, int number, double a, double b)	*/
/*	Generates "number" random samples												*/
/*	from an TRIANGULAR Distribution in the range [a,b]								*/
/*	Returns a pointer to the samples if successful									*/
/*	NULL if an error occurs															*/
/************************************************************************************/
double* RandomTriangBufferAB(double *samples, int number, double a, double b)
{
	if (number <= 0) return NULL;
	if (samples == NULL) 	
		samples = (double*)malloc(number * sizeof(double));
	// Seed generation from the current time (milliseconds)
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples[i] = RandomTriangAB(a, b);
	}
	return samples;
}
std::vector<double> *RandomTriangVectorAB(std::vector<double> *samples, int number, double a, double b)
{
	if (number<=0) return NULL;
	if (samples == NULL)
		samples = new std::vector<double>(number);
	else
		samples->resize(number);
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples->at(i) = RandomTriangAB(a, b);
	}
	return samples;
}

/************************************************************************/
/*	Function RandomTriangAB(unsigned long *seed, double a, double b)	*/
/*	Generates random numbers with a given seed							*/
/*	from an TRIANGULAR Distribution in the range [a,b]					*/
/*	Returns a random number if successful,								*/
/*	-1 if an error occurs												*/
/************************************************************************/
double RandomTriangABC(double a, double b, double c)
{
	GenerateSeed();
	double x = 0;
	double p = (b-a)/(c-a);
	double y = Random();
	if (y > p) x = c - (c-b) * sqrt((1-y)/(1-p));
	if (y <= p) x = a + (b-a) * sqrt(y/p);
	return x;
}

/************************************************************************************/
/*	Function RandomTriangBufferAB(double *samples, int number, double a, double b)	*/
/*	Generates "number" random samples												*/
/*	from an TRIANGULAR Distribution in the range [a,b]								*/
/*	Returns a pointer to the samples if successful									*/
/*	NULL if an error occurs															*/
/************************************************************************************/
double* RandomTriangBufferABC(double *samples, int number, double a, double b, double c)
{
	if (number <= 0) return NULL;
	if (samples == NULL) 	
		samples = (double*)malloc(number * sizeof(double));
	// Seed generation from the current time (milliseconds)
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples[i] = RandomTriangABC(a, b, c);
	}
	return samples;
}
std::vector<double> *RandomTriangVectorABC(std::vector<double> *samples, int number, double a, double b, double c)
{
	if (number<=0) return NULL;
	if (samples == NULL)
		samples = new std::vector<double>(number);
	else
		samples->resize(number);
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples->at(i) = RandomTriangABC(a, b, c);
	}
	return samples;
}

/************************************************************************/
/*	Function RandomExp(unsigned long *seed, double lambda)				*/
/*	Generates random numbers with a given seed							*/
/*	from an EXPONENTIAL Distribution, mean value "lambda"				*/
/*	Returns a random number if successful,								*/
/*	-1 if an error occurs												*/
/************************************************************************/
double RandomExp(double lambda)
{
	GenerateSeed();
	return (-lambda * log(1-Random()));
}

/************************************************************************************/
/*	Function RandomExpBuffer(double *samples, int number, double lambda)			*/
/*	Generates "number" random samples												*/
/*	from an EXPONENTIAL Distribution, mean value "lambda"							*/
/*	Returns a pointer to the samples if successful									*/
/*	NULL if an error occurs															*/
/************************************************************************************/
double* RandomExpBuffer(double *samples, int number, double lambda)
{
	if (number <= 0) return NULL;
	if (samples == NULL) 	
		samples = (double*)malloc(number * sizeof(double));
	// Seed generation from the current time (milliseconds)
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples[i] = RandomExp(lambda);
	}
	return samples;
}
std::vector<double> *RandomExpVector(std::vector<double> *samples, int number, double lambda)
{
	if (number<=0) return NULL;
	if (samples == NULL)
		samples = new std::vector<double>(number);
	else
		samples->resize(number);
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples->at(i) = RandomExp(lambda);
	}
	return samples;
}

/****************************************************************************/
/*	Function RandomNormal(unsigned long *seed)								*/
/*	Generates random numbers with a given seed								*/
/*	from an NORMAL Distribution, mean of zero and standard devation of one	*/
/*	Returns a random number if successful,									*/
/*	-1 if an error occurs													*/
/****************************************************************************/
double RandomNormal(bool bound)
{
	GenerateSeed();
	double number = (cos(2 * PI * Random()) * sqrt((-2) * log(Random())));
	if (!bound)
		return number;
	else
	{
		if ((number>=-2) && (number<=2))
			return number;
		else
			return RandomNormal(bound);
	}
	
}

/****************************************************************************/
/*	Function RandomNormalBuffer(double *samples, int number)				*/
/*	Generates "number" random samples										*/
/*	from an NORMAL Distribution, mean of zero and standard devation of one	*/
/*	Returns a pointer to the samples if successful							*/
/*	NULL if an error occurs													*/
/****************************************************************************/
double* RandomNormalBuffer(double *samples, int number, bool bound)
{
	if (number <= 0) return NULL;
	if (samples == NULL) 	
		samples = (double*)malloc(number * sizeof(double));
	// Seed generation from the current time (milliseconds)
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples[i] = RandomNormal(bound);
	}
	return samples;
}
std::vector<double> *RandomNormalVector(std::vector<double> *samples, int number, bool bound)
{
	if (number<=0) return NULL;
	if (samples == NULL)
		samples = new std::vector<double>(number);
	else
		samples->resize(number);
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples->at(i) = RandomNormal(bound);
	}
	return samples;
}

/********************************************************************************/
/*	Function RandomNormal(unsigned long *seed, double mu, double sigma)			*/
/*	Generates random numbers with a given seed									*/
/*	from an NORMAL Distribution, mean of "mu" and standard devation of "sigma	*/
/*	Returns a random number if successful,										*/
/*	-1 if an error occurs														*/
/********************************************************************************/
double RandomNormal(double mu, double sigma, bool bound)
{
	GenerateSeed();
	return (mu + sigma * RandomNormal(bound));
}

/************************************************************************************/
/*	Function RandomNormalBuffer(double *samples, int number, float mu, float sigma)	*/
/*	Generates "number" random samples												*/
/*	from an NORMAL Distribution, mean of "mu" and standard devation of "sigma"		*/
/*	Returns a pointer to the samples if successful									*/
/*	NULL if an error occurs															*/
/************************************************************************************/
double* RandomNormalBuffer(double *samples, int number, double mu, double sigma, bool bound)
{
	if (number <= 0) return NULL;
	if (samples == NULL) 	
		samples = (double*)malloc(number * sizeof(double));
	// Seed generation from the current time (milliseconds)
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples[i] = RandomNormal(mu, sigma, bound);
	}
	return samples;
}
std::vector<double> *RandomNormalVector(std::vector<double> *samples, int number, double mu, double sigma, bool bound)
{
	if (number<=0) return NULL;
	if (samples == NULL)
		samples = new std::vector<double>(number);
	else
		samples->resize(number);
	GenerateSeed();
	for (int i = 0; i < number; i++)
	{
		samples->at(i) = RandomNormal(mu, sigma, bound);
	}
	return samples;
}

/****************************************************************/
/*	Function Random_rand_Comparison(void)						*/	
/*	Compares function Random() and standard C function rand()	*/
/*	Stores the results (generated random numbers) in			*/
/*	c:\unif.txt													*/
/****************************************************************/
void Random_rand_Comparison(void)
{	
	// Seed generation from the current time (milliseconds)
	GenerateSeed();
	srand(seed);
	FILE* archivo = fopen("c:\\unif.txt", "w");

	for (int i = 0; i < 100000; i++)
	{
		fprintf(archivo, "%f\t%f\n", Random(), (float)rand()/(float)RAND_MAX);
	}	
	fclose(archivo);
}

/********************************************************/
/*	Function TestRandomAB(void)							*/
/*	Stores data generated by RandomAB() in the file		*/
/*	c:\testRandomAB.txt									*/	
/*	(so it can be analyzed with Matlab					*/
/********************************************************/
void TestRandomAB(int number, double min, double max)
{	
	double* data = (double*)malloc(number * sizeof(double));
	RandomBufferAB(data, number, min, max);
	FILE* archivo = fopen("c:\\testRandomAB.txt", "w");
	for (int i = 0; i < number; i++)
	{
		fprintf(archivo, "%f\n", data[i]);
	}	
	fclose(archivo);
}

/********************************************************/
/*	Function TestRandomTriangAB(void)					*/
/*	Stores data generated by RandomAB() in the file		*/
/*	c:\testRandomTriangAB.txt							*/	
/*	(so it can be analyzed with Matlab					*/
/********************************************************/
void TestRandomTriangAB(int number, double min, double max)
{	
	double* data = (double*)malloc(number * sizeof(double));
	RandomTriangBufferAB(data, number, min, max);
	FILE* archivo = fopen("c:\\testRandomTriangAB.txt", "w");
	for (int i = 0; i < number; i++)
	{
		fprintf(archivo, "%f\n", data[i]);
	}	
	fclose(archivo);
}

/********************************************************/
/*	Function TestRandomTriangABC(void)					*/
/*	Stores data generated by RandomABC() in the file	*/
/*	c:\testRandomTriangABC.txt							*/	
/*	(so it can be analyzed with Matlab					*/
/********************************************************/
void TestRandomTriangABC(int number, double min, double mid, double max)
{	
	double* data = (double*)malloc(number * sizeof(double));
	RandomTriangBufferABC(data, number, min, mid, max);
	FILE* archivo = fopen("c:\\testRandomTriangAB.txt", "w");
	for (int i = 0; i < number; i++)
	{
		fprintf(archivo, "%f\n", data[i]);
	}	
	fclose(archivo);
}

/********************************************************/
/*	Function TestRandomNormal(void)						*/
/*	Stores data generated by RandomNormal() in the file	*/
/*	c:\testRandomNormal.txt								*/	
/*	(so it can be analyzed with Matlab					*/
/********************************************************/
void TestRandomNormal(int number)
{	
	double* data = (double*)malloc(number * sizeof(double));
	RandomNormalBuffer(data, number);
	FILE* archivo = fopen("c:\\testRandomNormal.txt", "w");
	for (int i = 0; i < number; i++)
	{
		fprintf(archivo, "%f\n", data[i]);
	}	
	fclose(archivo);
}

/********************************************************/
/*	Function TestRandomNormal(void)						*/
/*	Stores data generated by RandomNormal()	specifying	*/
/*	mean value and typical deviation in the file		*/
/*	c:\testRandomNormal2.txt							*/	
/*	(so it can be analyzed with Matlab					*/
/********************************************************/
void TestRandomNormal2(int number, double mu, double sigma)
{	
	double* data = (double*)malloc(number * sizeof(double));
	RandomNormalBuffer(data, number, mu, sigma);
	FILE* archivo = fopen("c:\\testRandomNormal2.txt", "w");
	for (int i = 0; i < number; i++)
	{
		fprintf(archivo, "%f\n", data[i]);
	}	
	fclose(archivo);
}

/********************************************************/
/*	Function TestRandomExp(void)						*/
/*	Stores data generated by RandomExp() in the file	*/
/*	c:\testRandomExp.txt								*/
/*	(so it can be analyzed with Matlab					*/
/********************************************************/
void TestRandomExp(int number, double lambda)
{	
	double* data = (double*)malloc(number * sizeof(double));
	RandomExpBuffer(data, number, lambda);
	FILE* archivo = fopen("c:\\testRandomExp.txt", "w");
	for (int i = 0; i < number; i++)
	{
		fprintf(archivo, "%f\n", data[i]);
	}	
	fclose(archivo);
}
