#pragma once
#include "quat.h"
#include "transform.h"
struct DualQuaternion {
  union {
    struct {
      quat real;
      quat dual;
    } parts;
    float v[8];
  };
  inline DualQuaternion() {
    parts.real = quat(0, 0, 0, 1);
    parts.dual = quat(0, 0, 0, 0);
  }
  inline DualQuaternion(const quat &r, const quat &d) {
    parts.real = r;
    parts.dual = d;
  }
};

DualQuaternion operator+(const DualQuaternion &l, const DualQuaternion &r);
DualQuaternion operator*(const DualQuaternion &l, const DualQuaternion &r);
DualQuaternion operator*(const DualQuaternion &dq, float f);
bool operator==(const DualQuaternion &l, const DualQuaternion &r);
bool operator!=(const DualQuaternion &l, const DualQuaternion &r);
float dot(const DualQuaternion &l, const DualQuaternion &r);
DualQuaternion conjugate(const DualQuaternion &dq);
DualQuaternion normalized(const DualQuaternion &dq);
void normalize(DualQuaternion &dq);
DualQuaternion transformToDualQuat(const Transform &t);
Transform dualQuatToTransform(const DualQuaternion &dq);
vec3 transformVector(const DualQuaternion &dq, const vec3 &v);
vec3 transformPoint(const DualQuaternion &dq, const vec3 &v);
