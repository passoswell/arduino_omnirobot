/*
 *  MatrixMath.h Library for Matrix Math
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 *  Modified to work with Arduino 1.0/1.5 by randomvibe & robtillaart
 *  Made into a real library on GitHub by Vasilis Georgitzikis (tzikis)
 *  so that it's easy to use and install (March 2015)
 */


#ifdef __cplusplus
extern "C" {
#endif

#ifndef MATRIXMATH_H
#define MATRIXMATH_H

/*#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
*/
	//void Print(float* A, int m, int n, String label);

extern  float sqrt_approx(float z);
extern  float invSqrt_approx(float x);
extern  void oneNormVector(float* v, int sizev1, float* norm);
extern  void twoNormVectorSquared(float* v, int sizev1, float* norm);
extern  void twoNormVector(float* v, int sizev1, float* norm);
extern  void concatenateVector(float* v1, float*v2, int sizev1, int sizev2, float* vresult);
extern	void copyMatrix(float* A, int n, int m, float* B);
extern	void multiplyMatrix(float* A, float* B, int m, int p, int n, float* C);
extern  void addMatrix(float* A, float* B, int m, int n, float* C);
extern	void subtractMatrix(float* A, float* B, int m, int n, float* C);
extern	void transposeMatrix(float* A, int m, int n, float* C);
extern	void scaleMatrix(float* A, float *B, int m, int n, float k);
extern	int invertMatrix(float* A, int n); //Returns 0 if no success

#endif /* MATRIXMATH_H */



#ifdef __cplusplus
}
#endif
