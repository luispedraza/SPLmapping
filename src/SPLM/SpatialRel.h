// SpatialRel.h: interface for the CSpatialRel class.
// Author: Luis Pedraza
//////////////////////////////////////////////////////////////////////
#if !defined(SPATIAL_REL_)
#define SPATIAL_REL_

/** Simple spatial relation **/
class CSpatialRel  
{
public:
	CSpatialRel operator-(const CSpatialRel& xkj) const;
	CSpatialRel operator-() const;
	CSpatialRel operator+(const CSpatialRel& xjk) const;
	double ang;
	double y;
	double x;
	CSpatialRel(float xval, float yval, float angval);
	CSpatialRel();
	virtual ~CSpatialRel();
};

#endif // !defined(SPATIAL_REL_)
