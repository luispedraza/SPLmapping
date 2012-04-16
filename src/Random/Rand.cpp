// Rand.cpp: implementation of the CRand class.
// Author: Luis Pedraza
//////////////////////////////////////////////////////////////////////

#include "Rand.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <vector>

CRand::CRand()
{
	m_seed = 0;
	GenerateSeed();
}

CRand::~CRand()
{
}

double CRand::Random()
{
	m_seed = (unsigned long)(fmod(A * m_seed + C, M));
	return (double)m_seed / M;
}

double CRand::RandomNormal(double mu, double sigma, bool bound)
{
	return (mu + sigma * RandomNormal(bound));
}

double CRand::RandomNormal(bool bound)
{
	double number = (cos(2 * PI * Random()) * sqrt((-2) * log(Random())));
	if (!bound) return number;
	else
	{
		if ((number>=-2) && (number<=2)) return number;
		else return RandomNormal(bound);
	}
}

void CRand::GenerateSeed()
{
	while (m_seed == 0)
	{
		struct timeb myTime;
		ftime(&myTime);
		m_seed = (unsigned long)myTime.millitm;
	}
}
