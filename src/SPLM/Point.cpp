// Point.cpp: implementation of the Point class.
//
//////////////////////////////////////////////////////////////////////

#include "Point.h"
#include <math.h>

/** Devuelve la distancia en valor absoluto entre dos puntos **/
float Distance(const Point& p1,const Point& p2)
{
	float d_x = p1.x-p2.x;
	float d_y = p1.y-p2.y;
	return (float)sqrt(d_x*d_x + d_y*d_y);
}
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Point::Point()
{
	x = y = 0.0f;
}

Point::~Point()
{

}

Point::Point(float xval, float yval)
{
	x = xval; 
	y = yval;	
}

/** Devuelve el modulo del vector que define el punto **/
float Point::modulo() const
{
	return (float)sqrt(x*x+y*y);
}

/** Resta de los vectores que definen dos puntos **/
Point Point::operator - (const Point &v) const
{
	Point res;
	res.x=x-v.x;
	res.y=y-v.y;
	return res;
}

/** Producto escalar de los vectores que definen dos puntos **/
float Point::operator *(Point &v)	
{
	return (x*v.x + y*v.y);
}

/** Producto vectorial de los vectores que definen dos puntos **/
float Point::operator ^ (Point & v)
{
	return (x*v.y - y*v.x);
}

/** Producto del vector que define un punto por un escalar **/
Point Point::operator *(float f) const
{
	Point res;
	res.x=x*f;
	res.y=y*f;
	return res;
}
Point Point::operator + (const Point &v) const
{
	Point res;
	res.x=x+v.x;
	res.y=y+v.y;
	return res;
}

Point Point::Unitario() const
{
	Point retorno;
	float mod = modulo();
	if(mod > 0.00001)
	{
		retorno.x=x/mod;
		retorno.y=y/mod;
	}
	return retorno;
}

/*float Point::argumento()
{
	return (float)atan2(y,x);
}	

std::ifstream& operator>>(std::ifstream& f,Point& v)
{
	f>>v.x;
	f>>v.y;
	return f;
}

Point Point::Normal()
{
	Point v;
	v.x=y;
	v.y=-x;
	return v.Unitario();
}*/
