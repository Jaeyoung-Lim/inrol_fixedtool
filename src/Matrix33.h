/*
 * Matrix33.h
 *
 *  Created on: Jan 15, 2015
 *      Author: donghokang
 */

#ifndef MATRIX33_H_
#define MATRIX33_H_

#include <iostream>
#include <cmath>		// for mathmatic functions
#include <stdio.h>
#include "Vector3.h"

using namespace vector;

namespace matrix {

class Matrix33 {
public:
	Vector3 r1, r2, r3;
public:
	Matrix33();
	Matrix33(double (&)[9]);
	virtual ~Matrix33();
public:
	Matrix33 Trans(void) const;			// trans of matrix
	Vector3 operator*(Vector3) const;	// multiple with vector3
	Matrix33 operator*(double) const;	// multiple with constant
	Matrix33 operator/(double) const;	// divide with constant
	Matrix33 operator+(Matrix33) const;	// sum of two matrix
	Matrix33 operator*(Matrix33) const;	// multiple of two matrix
	double & operator[](int pos);
	void Display(void) const;
};
}

#endif /* MATRIX33_H_ */
