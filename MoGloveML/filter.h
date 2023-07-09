//Filter
#include "quaternion.h"

const float MIN_CUTOFF = 1.0;   // Minimum cutoff frequency
const float BETA = 0.05;        // Cutoff slope
const float MIN_LAMBDA = 0.001; // Minimum filtering amount
const float ALPHA = 0.001;      // Filtering amount when data is static

Quaternion prevRawQuat;    // Previous raw quaternion
Quaternion filteredQuat;   // Filtered quaternion
Quaternion prevFilteredQuat; // Previous filtered quaternion
float prevDeltaTime = 0;         // Previous time difference



Quaternion oneEuroFilter(Quaternion rawQuat, float deltaTime)
{
  // Calculate the frequency of change
  float dX = angDist(rawQuat, prevRawQuat) / deltaTime;
  
  // Calculate the cutoff frequency
  float cutoff = MIN_CUTOFF + BETA * fabs(dX);
  
  // Calculate the filtering amount
  float lambda = ALPHA + (cutoff / MIN_CUTOFF) * (1 - ALPHA);
  
  // Update the filtered quaternion
  filteredQuat = lambda * rawQuat + (1 - lambda) * prevFilteredQuat;
  
  // Update the previous values for the next iteration
  prevRawQuat = rawQuat;
  prevFilteredQuat = filteredQuat;
  prevDeltaTime = deltaTime;
  return filteredQuat;
}
