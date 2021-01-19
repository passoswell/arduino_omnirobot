/*
 *  MatrixMath.cpp Library for Matrix Math
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 */


#include <math.h>
#include <stdlib.h>
#include "myalgebra.h"



/** 
 * Matrix Printing Routine
 * Uses tabs to separate numbers under assumption printed float width won't cause problems
 */
/*void printMatrix(float* A, int m, int n, String label)
1{
	// A = input matrix (m x n)
	int i, j;
	Serial.println();
	Serial.println(label);
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			Serial.print(A[n * i + j]);
			Serial.print("\t");
		}
		Serial.println();
	}
}
*/


/** 
 * Assumes that float is in the IEEE 754 single precision floating point format
 * and that int is 32 bits. 
 */
float sqrt_approx(float z) {
    typedef union{long asint32; float asfloat;}FLOAT_INT32;
    FLOAT_INT32 val;
    val.asfloat = z;
    /**
     * To justify the following code, prove that
     * ((((val_int / 2^m) - b) / 2) + b) * 2^m = ((val_int - 2^m) / 2) + ((b + 1) / 2) * 2^m)
     * where
     * b = exponent bias
     * m = number of mantissa bits
     */

      //val_int -= 1 << 23; /* Subtract 2^m. */
      //val_int >>= 1; /* Divide by 2. */
      //val_int += 1 << 29; /* Add ((b + 1) / 2) * 2^m. */
      
      //or

      //a = -0x4B0D2;
      //val.asint32 = (1L << 29) + (val.asint32 >> 1) - (1L << 22)+a;
      
      //or
      
      val.asint32 = (val.asint32 >> 1) + 532676608L - 0x4B0D2L;
      
    return val.asfloat;
    //return *(float*)&val_int; /* Interpret again as float */
}


float invSqrt_approx(float z) {
    //float xhalf = 0.5f*z;
    typedef union{long asint32; float asfloat;}FLOAT_INT32;
    FLOAT_INT32 val;
    val.asfloat = z;
    val.asint32 = 0x5f375a86 - (val.asint32 >> 1);
    /* The next line can be repeated any number of times to increase accuracy */
    //val.asint32 = val.asint32 * (1.5f - xhalf * u.x * u.x);
    return val.asfloat;
}






void oneNormVector(float* v, int sizev1, float* norm){
  //v = input vector
  //sizev1 = size of v1
  //norm = 1-norm, or sum of absolute values
  int i;
  *norm = 0.0;
  for(i=0;i<sizev1;i++){
    *norm += fabs(*(v+i));
  }
}

void twoNormVectorSquared(float* v, int sizev1, float* norm){
  //v = input vector
  //sizev1 = size of v1
  //norm = 2-norm, or square root of sum of squares
  int i;
  *norm = 0.0;
  for(i=0;i<sizev1;i++){
    *norm += (*(v+i)) * (*(v+i));
  }
}

void twoNormVector(float* v, int sizev1, float* norm){
  //v = input vector
  //sizev1 = size of v1
  //norm = 2-norm, or square root of sum of squares
  twoNormVectorSquared(v, sizev1, norm);
  *norm = sqrt(*norm);
}

//extern void concatenateVector(float* v1, float*v2, int sizev1, int sizev2, float* vresult){
void concatenateVector(float* v1, float*v2, int sizev1, int sizev2, float* vresult){
  //v1 = first input vector
  //v2 = second input vector
  //sizev1 = size of v1
  //sizev2 = size of v2
  //vresult = vector where the concatenated vector is stored.
  copyMatrix(v1,sizev1,1,vresult);
  copyMatrix(v2,sizev2,1,vresult+sizev1);  
}


//extern void copyMatrix(float* A, int n, int m, float* B){
void copyMatrix(float* A, int n, int m, float* B){
	// A = input matrix (m x p)
	// m = number of rows in A
	// n = number of columns in B
	// B = input matrix (p x n)
  int i,j;
  j = m*n;
  for (i = 0; i < j ; i++){
    *B = *A;
    A++;B++;
  }
}

//Matrix Multiplication Routine
// C = A*B
void multiplyMatrix(float* A, float* B, int m, int p, int n, float* C){
	// A = input matrix (m x p)
	// B = input matrix (p x n)
	// m = number of rows in A
	// p = number of columns in A = number of rows in B
	// n = number of columns in B
	// C = output matrix = A*B (m x n)
	int i, j, k;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++){
			*(C + n*i + j) = 0;
			for (k = 0; k < p; k++)
				*(C + n*i + j) = *(C + n*i + j) + *(A + p*i + k) * (*(B + n*k + j));
		}
}


//Matrix Addition Routine
void addMatrix(float* A, float* B, int m, int n, float* C){
  // A = input matrix pointer(m x n)
  // B = input matrix pointer(m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix pointer = A+B (m x n)
  int i,j;
  j = m*n;
  for (i = 0; i < j ; i++){
    *C = *A + *B;
    A++;B++;C++;
  }
}


//Matrix Subtraction Routine
void subtractMatrix(float* A, float* B, int m, int n, float* C){
	// A = input matrix pointer(m x n)
	// B = input matrix pointer(m x n)
	// m = number of rows in A = number of rows in B
	// n = number of columns in A = number of columns in B
	// C = output matrix pointer = A-B (m x n)
  int i,j;
  j = m*n;
  for (i = 0; i < j ; i++){
    *C = *A - *B;
    A++;B++;C++;
  }
}


//Matrix Transpose Routine
void transposeMatrix(float* A, int m, int n, float* C){
	// A = input matrix (m x n)
	// m = number of rows in A
	// n = number of columns in A
	// C = output matrix = the transpose of A (n x m)
	int i, j;
	for (i = 0; i < m; i++){
		for(j = 0; j < n; j++)
			//C[m * j + i] = A[n * i + j];
      *(C+m*j+i) = *(A+n*i+j);
   }
}

void scaleMatrix(float* A, float *B, int m, int n, float k){
	// A = input matrix (m x n)
	// m = number of rows in A
	// n = number of columns in A
	//k = real constant
	int i,j;
  j = m*n;
  for (i = 0; i < j ; i++){
    *B = (*A)*k;
    A++; B++;
  }
}


//Matrix Inversion Routine
// * This function inverts a matrix based on the Gauss Jordan method.
// * Specifically, it uses partial pivoting to improve numeric stability.
// * The algorithm is drawn from those presented in
//	 NUMERICAL RECIPES: The Art of Scientific Computing.
// * The function returns 1 on success, 0 on failure.
// * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED
int invertMatrix(float* A, int n){
  // A = input matrix AND result matrix
  // n = number of rows = number of columns in A (n x n)
  int pivrow = 0;   // keeps track of current pivot row
  int k, i, j;    // k: overall index along diagonal; i: row index; j: col index
  int pivrows[n]; // keeps track of rows swaps to undo at end
  float tmp;    // used for finding max value and making column swaps

  for (k = 0; k < n; k++){
    // find pivot row, the row with biggest entry in current column
    tmp = 0;
    for (i = k; i < n; i++){
      if (fabs(*(A + i*n + k)) >= tmp){ // 'Avoid using other functions inside abs()?'
        tmp = fabs(*(A + i*n + k));
        pivrow = i;
      }
    }

    // check for singular matrix
    if (*(A + pivrow*n + k) == 0.0f){
      //Serial.println("Inversion failed due to singular matrix");
      return 0;
    }

    // Execute pivot (row swap) if needed
    if (pivrow != k){
      // swap row k with pivrow
      for (j = 0; j < n; j++){
        tmp = *(A + k*n + j);
        *(A + k*n + j) = *(A + pivrow*n + j);
        *(A + pivrow*n + j) = tmp;
      }
    }
    pivrows[k] = pivrow;  // record row swap (even if no swap happened)

    tmp = 1.0f / *(A + k*n + k);  // invert pivot element
    *(A + k*n + k) = 1.0f;    // This element of input matrix becomes result matrix

    // Perform row reduction (divide every element by pivot)
    for (j = 0; j < n; j++){
      *(A + k*n + j) = (*(A + k*n + j)) * tmp;
    }

    // Now eliminate all other entries in this column
    for (i = 0; i < n; i++){
      if (i != k){
        tmp = *(A + i*n + k);
        *(A + i*n + k) = 0.0f; // The other place where in matrix becomes result mat
        for (j = 0; j < n; j++){
          *(A + i*n + j) = *(A + i*n + j) - *(A + k*n + j) * tmp;
        }
      }
    }
  }

  // Done, now need to undo pivot row swaps by doing column swaps in reverse order
  for (k = n - 1; k >= 0; k--){
    if (pivrows[k] != k){
      for (i = 0; i < n; i++){
        tmp = *(A + i*n + k);
        *(A + i*n + k) = *(A + i*n + pivrows[k]);
        *(A + i*n + pivrows[k]) = tmp;
      }
    }
  }
  return 1;
}
