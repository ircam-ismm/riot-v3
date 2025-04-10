#include <Arduino.h>

#include "functions.h"


// returns the decimals of a number, ie 12.2134 -> 0.2134
float fract(float x){
  return x - floorf(x);
  }

// clamp(x, a, b) makes sure that x is between a and b.
float clamp(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

float Fmod(float a, float b) {
  return a - floorf(a / b) * b;
}

int32_t clampi32(int32_t x, int32_t a, int32_t b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

int16_t clamptoi16(int32_t x) {
  return clampi32(x, -32768, 32767);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Symmetrize scatter matrix
void symmetrize10x10(double M[10][10]) {
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < i; j++) {
            M[i][j] = M[j][i];
        }
    }
}

void printMatrix3x3(const double A[3][3]) {
  for(int i = 0 ; i < 3 ; i++) {
    Serial.printf("[ ");
    for(int j = 0 ; j < 3 ; j++) {
      Serial.printf("%f ", A[i][j]);
    }
    Serial.printf(" ]\n");
  }
}

void printMatrix3x3(const float A[3][3]) {
  for(int i = 0 ; i < 3 ; i++) {
    Serial.printf("[ ");
    for(int j = 0 ; j < 3 ; j++) {
      Serial.printf("%f ", A[i][j]);
    }
    Serial.printf(" ]\n");
  }
}

void printMatrix10x10(const double A[10][10]) {
  for(int i = 0 ; i < 10 ; i++) {
    Serial.printf("[ ");
    for(int j = 0 ; j < 10 ; j++) {
      Serial.printf("%f ", A[i][j]);
    }
    Serial.printf(" ]\n");
  }
}


double dot(const double *a, const double *b, int len) {
	double sum = 0.0;
	for(int i = 0; i < len; i++) {
		sum += a[i] * b[i];
	}
    return sum;
}

double norm(const double *v, int len) {
    return sqrt(dot(v, v, len));
}

// Normalize the matrix so that diagonal elements are 1
// Normalization occurs "in place" (doesn't keep the source matrix)
// Fails if either diagonal element is 0.0
bool normalize3x3(double A[3][3]) {
	double scale;
	bool success = true;
	double normRes[3][3];
	for(int i = 0; i < 3; i++) {
		scale = A[i][i];
		if(scale != 0.0) {
			scale = 1.0 / scale;
		}
		else {
			scale = 1.0;
			success = false;
		}
		for(int j = 0; j < 3; j++) {
			normRes[i][j] = A[i][j] * scale;
		}
	}
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			A[i][j] = normRes[i][j];
		}
	}
	return success;
}
	   
	   
void normalizeVector(double *v, int len) {
    double norm = 0.0;
    for (int i = 0; i < len; i++)
        norm += v[i] * v[i];
    norm = sqrt(norm);
    if (norm == 0.0)
		norm = 1.0;
	else norm = 1.0 / norm;
    for (int i = 0; i < len; i++)
        v[i] *= norm;
}

double diffNormVector(const double *a, const double *b, int len) {
    double sum = 0.0;
    for (int i = 0; i < len; ++i)
        sum += (a[i] - b[i]) * (a[i] - b[i]);
    return sqrt(sum);
}

void matVecMul10x10(const double A[10][10], const double v[10], double result[10]) {
    for (int i = 0; i < 10; ++i) {
        result[i] = 0.0;
        for (int j = 0; j < 10; ++j)
            result[i] += A[i][j] * v[j];
    }
}

bool powerIteration10x10(const double A[10][10], double eigenvec[10], int numIter, double tol) {
    double b[10] = {0.0};  // Initial guess
	b[0] = 1.0;
    double b_next[10];
	bool success = false;
	
    for (int k = 0; k < numIter; ++k) {
        matVecMul10x10(A, b, b_next);
        normalizeVector(b_next,10);
        
        if (diffNormVector(b, b_next, 10) < tol) {
			if(k < numIter) {	// most likely
				success = true;
			}
            break;
		}

        for (int i = 0; i < 10; ++i)
            b[i] = b_next[i];
    }

    // Copy result
    for (int i = 0; i < 10; ++i)
        eigenvec[i] = b[i];
	return success;
}

// Compute 3x3 inverse (basic method)
bool inverse3x3(const double M[3][3], double inv[3][3]) {
    double det =
        M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
        M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
        M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);


	//Serial.printf("Determinant = %f\n", det);

    if (fabs(det) < 1e-6) return false;

    double invdet = 1.0 / det;

    inv[0][0] =  (M[1][1]*M[2][2] - M[1][2]*M[2][1]) * invdet;
    inv[0][1] = -(M[0][1]*M[2][2] - M[0][2]*M[2][1]) * invdet;
    inv[0][2] =  (M[0][1]*M[1][2] - M[0][2]*M[1][1]) * invdet;
    inv[1][0] = -(M[1][0]*M[2][2] - M[1][2]*M[2][0]) * invdet;
    inv[1][1] =  (M[0][0]*M[2][2] - M[0][2]*M[2][0]) * invdet;
    inv[1][2] = -(M[0][0]*M[1][2] - M[0][2]*M[1][0]) * invdet;
    inv[2][0] =  (M[1][0]*M[2][1] - M[1][1]*M[2][0]) * invdet;
    inv[2][1] = -(M[0][0]*M[2][1] - M[0][1]*M[2][0]) * invdet;
    inv[2][2] =  (M[0][0]*M[1][1] - M[0][1]*M[1][0]) * invdet;

    return true;
}





//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
// (thank you quake)
float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  uint32_t i = *(uint32_t*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y)); // First iteration
  y = y * (1.5f - (halfx * y * y)); // optional second iteration
  return y;
}

// We need to check accuracy of those => make a test routine for it

// Variant with 1/3 of the error of the code above
// https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/
float accurateinvSqrt(float x){
  uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
  float tmp = *(float*)&i;
  return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}