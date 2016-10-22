/*
 * Vector3.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: donghokang
 */

#include "Vector3.h"

using namespace vector;

Vector3::Vector3(void):x(0.0),y(0.0),z(0.0)
{
	// constructor with void parameter
	// create Vector Object with x=0, y=0, z=0
}

Vector3::Vector3(double ix, double iy, double iz):x(ix),y(iy),z(iz)
{
	// constructor with three parameter
	// create Vector Object with x=ix, y=iy, z=iz
}

Vector3::Vector3(double in[3]):x(in[0]),y(in[1]),z(in[2])
{
	// constructor with three parameter
	// create Vector Object with x=ix, y=iy, z=iz
}

Vector3::~Vector3() {
}

// functions below
Vector3& Vector3::operator=(const Vector3 iVec)
{
	x = iVec.x;
	y = iVec.y;
	z = iVec.z;
	return *this;
}
void Vector3::display(void) const
{
	// function for logging
	printf("(%f, %f, %f)\n ", x, y, z);
}
Vector3 Vector3::operator+(Vector3 iVec) const
{
	// function for +
	return Vector3(x+iVec.x, y+iVec.y, z+iVec.z);
}
Vector3 Vector3::operator-(Vector3 iVec) const
{
	// function for -
	return Vector3(x-iVec.x, y-iVec.y, z-iVec.z);
}
Vector3 Vector3::operator*(double mult) const
{
	// function for multiple
	return Vector3(x*mult, y*mult, z*mult);
}
Vector3 Vector3::operator/(double divisor) const
{
	// function for division
	return Vector3(x/divisor, y/divisor, z/divisor);
}
double Vector3::Magnitude(void) const
{
	// function for absolute value
	return(sqrt(x*x+y*y+z*z));
}
double Vector3::Dot(Vector3 iVec) const
{
	// function for dot product
	return x*iVec.x + y*iVec.y + z*iVec.z;
}


double& Vector3::operator[](int i) 
{
	switch(i)
{
case 0 : return x;
case 1 : return y;
case 2 : return z;
}
}

