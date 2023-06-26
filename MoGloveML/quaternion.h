#ifndef QUATERNION_UTILS_H
#define QUATERNION_UTILS_H

#include <Math.h>

struct Quaternion {
  float w, x, y, z;
};

struct EulerAngles {
  float roll;
  float pitch;
  float yaw;
};

float magQuat(const Quaternion& quat) {
  return sqrt(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
}


void normQuat(Quaternion& quat) {
  float mag = magQuat(quat);
  quat.w /= mag;
  quat.x /= mag;
  quat.y /= mag;
  quat.z /= mag;
}

Quaternion scaleQuat(const Quaternion& quat, float scalar) {
  //Quaternion sucks at scaler rotation
  //Slerp => shortest path => not what I want
  //Quaternion=>Euler=>Scale=>Quaternion
  Quaternion result;

  //Convert to Euler
  float roll = atan2(2*(quat.w*quat.x+quat.y*quat.z), 1-2*(quat.x*quat.x+quat.y*quat.y));
  float pitch = asin(2*(quat.w*quat.y-quat.z*quat.x));
  float yaw = atan2(2*(quat.w*quat.z+quat.x*quat.y),1-2*(quat.y*quat.y+quat.z*quat.z));

  //Apply Scale
  roll = roll/scalar;
  pitch = pitch/scalar;
  yaw = yaw/scalar;

  //Convert back to Quaternion
  result.w = cos(roll) * cos(roll) * cos(roll) + sin(roll) * sin(roll) * sin(roll);
  result.x = sin(roll) * cos(roll) * cos(roll) - cos(roll) * sin(roll) * sin(roll);
  result.y = cos(roll) * sin(roll) * cos(roll) + sin(roll) * cos(roll) * sin(roll);
  result.z = cos(roll) * cos(roll) * sin(roll) - sin(roll) * sin(roll) * cos(roll);
  normQuat(result);
  return result;
}


Quaternion add2Quats(const Quaternion& quat1, const Quaternion& quat2){
  Quaternion result;
  result.w = quat1.w + quat2.w;
  result.x = quat1.x + quat2.x;
  result.y = quat1.y + quat2.y;
  result.z = quat1.z + quat2.z;
  normQuat(result);
  return result;
}
#endif
