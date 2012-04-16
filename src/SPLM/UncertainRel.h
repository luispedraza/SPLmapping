// UncertainRel.h: interface for the CUncertainRel class.
// Author: Luis Pedraza
//////////////////////////////////////////////////////////////////////

#if !defined(UNCERTAIN_REL_)
#define UNCERTAIN_REL_

#include "SpatialRel.h"
#include <newmat/newmat.h>


/** Uncertain relation **/
class CUncertainRel : public CSpatialRel  
{
public:
	CUncertainRel operator-(const CUncertainRel &xkj) const;
	CUncertainRel operator-() const;
	CUncertainRel operator+(const CUncertainRel &xjk) const;
	CUncertainRel();
	virtual ~CUncertainRel();
	NEWMAT::SymmetricMatrix Sigma;	///< Covariance matrix of the relationship

};

CUncertainRel Compound(CUncertainRel &xij, CUncertainRel &xjk, NEWMAT::Matrix &J1, NEWMAT::Matrix &J2);

#endif // !defined(UNCERTAIN_REL_)
