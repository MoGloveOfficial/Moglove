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

Quaternion multQuat(Quaternion q1, Quaternion q2) {
  Quaternion result;
  result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
  return result;
}


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

Quaternion conjQuat(Quaternion& quat){
  Quaternion result;
  result.w = quat.w;
  result.x = -quat.x;
  result.y = -quat.y;
  result.z = -quat.z;
}



float AngDist(Quaternion& quat1, Quaternion& quat2){
  //Angular difference of two quats
  Quaternion delta = multQuat(quat1, conjQuat(quat2));
  return 2.0 * acos(delta.w);
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

/*
Quaternion scaleQuat(const Quaternion& quat, float scalar){
  Quaternion result;
  result.w = quat.w;
  result.x = quat.x*scalar;
  result.y = quat.y*scalar;
  result.z = quat.z*scalar;
  normQuat(result);
  return result;
}
*/

Quaternion addQuat(const Quaternion& quat1, const Quaternion& quat2){
  Quaternion result;
  result.w = quat1.w + quat2.w;
  result.x = quat1.x + quat2.x;
  result.y = quat1.y + quat2.y;
  result.z = quat1.z + quat2.z;
  normQuat(result);
  return result;
}
#endif
