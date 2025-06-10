

//////////////////////////////////////////////////////////////////////////////////////
// Generic tool box for data processing and various math functions

#ifndef _FUNCTIONS_H
#define _FUNCTIONS_H

#include <Arduino.h>


// {0;360°} range to {-180;+180°}
#define FROM_360_DEGREE(theta) (fmodf(theta + 180.0f, 360.0f) - 180.0f)
//#define FROM_360_DEGREE(theta) fmodf(theta + 180.f, 360.f)
// {-180;+180°} range to {0;360°}
#define TO_360_DEGREE(theta) (fmodf(theta + 360.0f, 360.0f))

#define EPSILON 1e-6f


float fract(float x);
float clamp(float x, float a, float b);
float Fmod(float a, float b);
int32_t clampi32(int32_t x, int32_t a, int32_t b);
int16_t clamptoi16(int32_t x);
float fmap(float x, float in_min, float in_max, float out_min, float out_max);
float accurateinvSqrt(float x);
float invSqrt(float x);
void symmetrize10x10(double M[10][10]);
void printMatrix3x3(const double A[3][3]);
void printMatrix3x3(const float A[3][3]);
void printMatrix10x10(const double A[10][10]);
bool inverse3x3(const double M[3][3], double inv[3][3]);
bool normalize3x3(double A[3][3]);
double norm(const double *v, int len);
double dot(const double *a, const double *b, int len);
void matVecMul10x10(const double A[10][10], const double v[10], double result[10]);
void normalizeVector(double *v, int len);
double diffNormVector(const double *a, const double *b, int len);
bool powerIteration10x10(const double A[10][10], double eigenvec[10], int numIter = 100, double tol = 1e-6f);


// Circular buffer of N elements of class T. 
// Filter method adds a new element to the next position in buffer the sums
// all elements and divides by N to get an average <=> moving average filter
// Not optimum : replace with a moving average with moving sum to avoid
// recomputing the whole sum each time
template<class T, int N>
class BoxFilter {
public:
  T filter(const T& v) {
    data[pos] = v;
    pos++;
    if (pos == N) pos = 0;

    T ret = data[0];
    for (int i = 1; i < N; i++) {
      ret += data[i];
    }
    average = ret / N;
    return average;
  }

  T init(const T& v) {
    for (int i = 0; i < N; i++) {
      data[i] = v;
    }
	return(v);
  }

  T data[N];
  T average;
  int pos = 0;
};



/////////////////////////////////////////////////////
// A median filter implementation
template<class T, int N>
class MedianFilter {
public:
  T filter(const T& v) {
    // add V to the circular buffer - TBD
    push(v);
    //printf("Pushed   ");
    //printData();
    bubbleSort();
    //printf("Sorted   ");
    //printSorted();
    return(sorted[midPoint]);
  }

  T init(const T& v) {
    for (int i = 0; i < N; i++) {
      data[i] = v;
    }
	return v;
  }

  void printData(void) {
    for(int i = 0 ; i < N ; i++)
      printf("%d ", data[i]);
    printf("\n");
  }
  
  void printSorted(void) {
    for(int i = 0 ; i < N ; i++)
      printf("%d ", sorted[i]);
    printf("\n");
  }
  
  void swap(int i, int j) { 
    T temp = sorted[i];
    sorted[i] = sorted[j]; 
    sorted[j] = temp;
  } 

  void push(T newVal) {
    for(int i=N-1;i>0;i--) {
      data[i]=data[i-1];
    }
    data[0] = newVal;
  }
  
  void bubbleSort() { // fine as we sort on a short number of items
    int i, j;
    for (i = 0; i < N; i++) { // copy shift buffer
      sorted[i] = data[i];
    }
    for (i = 0; i < N - 1; i++) { // sort
      // Last i elements are already in place
      for (j = 0; j < N - i - 1; j++) {
        if (sorted[j] > sorted[j + 1])
          swap(j, j + 1);
      }
    }
  }
  
  T data[N];
  T sorted[N];
  int midPoint = (N-1)/2;
};



class linearInterpolator {
private:
    float startValue, endValue, interpolatedValue;
    float duration, startTime;
	bool active;

public:
    // Constructor
    linearInterpolator() : startValue(0), endValue(0), duration(1), startTime(0), active(0) {}

    // Set the start and end values
    void begin(float start, float end) {
        startValue = start;
        endValue = end;
    }

    // Set the interpolation duration (in seconds)
    void setDuration(float time) {
        duration = (time > 0) ? time : 1.0f; // Avoid division by zero
    }

    // Start the interpolation (record start time)
    void start(void) {
        startTime = millis();
		active = true;
    }

    // Get interpolated value at given time
    float getValue(void) {
		float t = millis() - startTime;
		if(t > duration) {
			active = false;
			interpolatedValue = endValue;
		}
		else {
			t = t / duration;
			t = clamp(t, 0.0f, 1.0f); // Clamp between 0 and 1
			interpolatedValue = startValue + t * (endValue - startValue);
		}
        return interpolatedValue;
    }
	
	bool isActive() { return active; }
};


#endif
