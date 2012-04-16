// Point.h: interface for the Point class.
//
//////////////////////////////////////////////////////////////////////

#ifndef POINT_H_
#define POINT_H_

class Point
{
public:
	Point();
	Point(float xval, float yval);
	virtual ~Point();

	float modulo() const;
	Point operator - (const Point &v) const;
	float operator *(Point &v);
	float operator ^ (Point & v);
	Point operator *(float f) const;
	Point operator + (const Point &v) const;
	Point Unitario() const;
	float Distance(const Point& p1,const Point& p2);

	float x;	///< x coordinate
	float y;	///< y cordinate


};

#endif // (POINT_H_)
