#ifndef QUATERNION_UTILS_H
#define QUATERNION_UTILS_H

#include <Math.h>

struct Quaternion {
  float w, x, y, z;
};

struct Euler {
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

Euler quat2Euler(const Quaternion& quat){
  Euler euler;
  euler.roll = atan2(2*(quat.w*quat.x+quat.y*quat.z), 1-2*(quat.x*quat.x+quat.y*quat.y));
  euler.pitch = asin(2*(quat.w*quat.y-quat.z*quat.x));
  euler.yaw = atan2(2*(quat.w*quat.z+quat.x*quat.y),1-2*(quat.y*quat.y+quat.z*quat.z));
  return euler;
}

Quaternion euler2Quat(const Euler& euler){
  float cr = cos(euler.roll * 0.5);
  float sr = sin(euler.roll * 0.5);
  float cp = cos(euler.pitch * 0.5);
  float sp = sin(euler.pitch * 0.5);
  float cy = cos(euler.yaw * 0.5);
  float sy = sin(euler.yaw * 0.5);
  Quaternion quat;
  quat.w = cr * cp * cy + sr * sp * sy;
  quat.x = sr * cp * cy - cr * sp * sy;
  quat.y = cr * sp * cy + sr * cp * sy;
  quat.z = cr * cp * sy - sr * sp * cy;
  normQuat(quat);
  return quat;
}

Quaternion scaleQuat(const Quaternion& quat, float scalar) {
  //Quaternion=>Euler=>Scale=>Quaternion
  //Convert to euler
  Euler euler = quat2Euler(quat);
  
  //Apply Scale
  euler.roll = euler.roll/scalar;
  euler.pitch = euler.pitch/scalar;
  euler.yaw = euler.yaw/scalar;

  //Convert back to Quaternion
  Quaternion result = euler2Quat(euler);
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
