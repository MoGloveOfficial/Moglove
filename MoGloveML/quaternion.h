#ifndef QUATERNION_UTILS_H
#define QUATERNION_UTILS_H

struct Quaternion {
  float w, x, y, z;
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
  Quaternion result;
  result.w = quat.w * scalar;
  result.x = quat.x * scalar;
  result.y = quat.y * scalar;
  result.z = quat.z * scalar;
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
