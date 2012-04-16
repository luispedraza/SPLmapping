// Rand.h: interface for the CRand class.
// Author: Luis Pedraza
//////////////////////////////////////////////////////////////////////
#if !defined(__RAND_H_)
#define __RAND_H_

#define	A	202		
#define	C	0		
#define	M	1000000	

//#define A	16807.0			
//#define C	0		
//#define M	2147483647.0

#ifndef PI
	#define PI	3.14159265	
#endif

class CRand  
{
public:
	void GenerateSeed(void);
	double RandomNormal(bool bound);
	double RandomNormal(double mu, double sigma, bool bound);
	double Random(void);
	CRand();
	virtual ~CRand();
private:
	unsigned long m_seed;		///< Seed for random numbers generation
};

#endif // !defined(__RAND_H_)
