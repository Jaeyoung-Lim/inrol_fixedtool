/*
 * Vector3.h
 *
 *  Created on: Jan 15, 2015
 *      Author: donghokang
 */

#ifndef VECTOR3_H_
#define VECTOR3_H_

#include <iostream>
#include <cmath>		// for mathmatic functions
#include <stdio.h>

namespace vector {

class Vector3 {
public:
	Vector3();
	Vector3(double, double, double);
	Vector3(double in[3]);
	virtual ~Vector3();
public:
	Vector3& operator=(const Vector3);
	Vector3 operator+(Vector3) const;
	Vector3 operator-(Vector3) const;
	Vector3 operator*(double) const;
	Vector3 operator/(double) const;
	double Dot(Vector3) const;
	Vector3 Cross(Vector3) const;
	double Magnitude(void) const;
	void display(void) const;
	double& operator[](int i);
public:
	double x;
	double y;
	double z;
};
}
#endif /* VECTOR3_H_ */
