

//////////////////////////////////////////////////////////////////////////////////////
// Generic tool box for data processing and various math functions

#ifndef _FUNCTIONS_H
#define _FUNCTIONS_H

#include <Arduino.h>


// {0;360°} range to {-180;+180°}
#define FROM_360_DEGREE(theta) fmod(theta + 180.f, 360.f) - 180.f
// {-180;+180°} range to {0;360°}
#define TO_360_DEGREE(theta) fmod(theta + 180.f, 360.f)



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


#endif
