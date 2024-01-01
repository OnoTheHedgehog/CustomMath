#include "dualQuaternion.h"
#include <math.h>
DualQuaternion operator+(const DualQuaternion &l, const DualQuaternion &r) {
  return DualQuaternion(l.parts.real + r.parts.real,
                        l.parts.dual + r.parts.dual);
}

DualQuaternion operator*(const DualQuaternion &dq, float f) {
  return DualQuaternion(dq.parts.real * f, dq.parts.dual * f);
}

DualQuaternion operator*(const DualQuaternion &l, const DualQuaternion &r) {
  DualQuaternion lhs = normalized(l);
  DualQuaternion rhs = normalized(r);
  return DualQuaternion(lhs.parts.real * rhs.parts.real,
                        lhs.parts.real * rhs.parts.dual +
                            lhs.parts.dual * rhs.parts.real);
}
bool operator==(const DualQuaternion &l, const DualQuaternion &r) {
  return l.parts.real == r.parts.real && l.parts.dual == r.parts.dual;
}
bool operator!=(const DualQuaternion &l, const DualQuaternion &r) {
  return l.parts.real != r.parts.real || l.parts.dual != r.parts.dual;
}

float dot(const DualQuaternion &l, const DualQuaternion &r) {
  return dot(l.parts.real, r.parts.real);
}
DualQuaternion conjugate(const DualQuaternion &dq) {
  return DualQuaternion(conjugate(dq.parts.real), conjugate(dq.parts.dual));
}
DualQuaternion normalized(const DualQuaternion &dq) {
  float magSq = dot(dq.parts.real, dq.parts.real);
  if (magSq < 0.000001f) {
    return DualQuaternion();
  }
  float invMag = 1.0f / sqrtf(magSq);
  return DualQuaternion(dq.parts.real * invMag, dq.parts.dual * invMag);
}
void normalize(DualQuaternion &dq) {
  float magSq = dot(dq.parts.real, dq.parts.real);
  if (magSq < 0.000001f) {
    return;
  }
  float invMag = 1.0f / sqrtf(magSq);
  dq.parts.real = dq.parts.real * invMag;
  dq.parts.dual = dq.parts.dual * invMag;
}

DualQuaternion transformToDualQuat(const Transform &t) {
  quat d(t.position.x, t.position.y, t.position.z, 0);
  quat qr = t.rotation;
  quat qd = qr * d * 0.5f;
  return DualQuaternion(qr, qd);
}
Transform dualQuatToTransform(const DualQuaternion &dq) {
  Transform result;
  result.rotation = dq.parts.real;
  quat d = conjugate(dq.parts.real) * (dq.parts.dual * 2.0f);
  result.position = vec3(d.x, d.y, d.z);
  return result;
}

vec3 transformVector(const DualQuaternion &dq, const vec3 &v) {
  return dq.parts.real * v;
}

vec3 transformPoint(const DualQuaternion &dq, const vec3 &v) {
  quat d = conjugate(dq.parts.real) * (dq.parts.dual * 2.0f);
  vec3 t = vec3(d.x, d.y, d.z);
  return dq.parts.real * v + t;
}
