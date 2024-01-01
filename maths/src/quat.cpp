#include "quat.h"
#include <math.h>

#define QUAT_EPSILON 0.0000001f
quat angleAxis(float angle, const vec3 &axis) {
  vec3 norm = normalized(axis);

  float s = sinf(angle * 0.5f);
  return quat(norm.x * s, norm.y * s, norm.z * s, cosf(angle * 0.5f));
}

quat fromTo(const vec3 &from, const vec3 &to) {
  vec3 f = normalized(from);
  vec3 t = normalized(to);
  if (f == t) {
    return quat();
  } else if (f == t * -1.0f) {
    vec3 ortho = vec3(1, 0, 0);
    if (fabsf(f.y) < fabsf(f.x)) {
      ortho = vec3(0, 1, 0);
    }
    if (fabsf(f.z) < fabs(f.y) && fabs(f.z) < fabsf(f.x)) {
      ortho = vec3(0, 0, 1);
    }
    vec3 axis = normalized(cross(f, ortho));
    return quat(axis.x, axis.y, axis.z, 0);
  }
  vec3 half = normalized(f + t);
  vec3 axis = cross(f, half);
  return quat(axis.x, axis.y, axis.z, dot(f, half));
}

vec3 getAxis(const quat &quat) {
  return normalized(vec3(quat.x, quat.y, quat.z));
}

float getAngle(const quat &quat) { return 2.0f * acosf(quat.w); }

quat operator+(const quat &a, const quat &b) {
  return quat(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}
quat operator-(const quat &a, const quat &b) {
  return quat(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

quat operator-(const quat &q) { return quat(-q.x, -q.y, -q.z, -q.w); }
quat operator+(const quat &a, float b) {
  return quat(a.x * b, a.y * b, a.z * b, a.w * b);
}

bool operator==(const quat &left, const quat &right) {
  return (fabsf(left.x - right.x) <= QUAT_EPSILON &&
          fabsf(left.y - right.y) <= QUAT_EPSILON &&
          fabsf(left.z - right.z) <= QUAT_EPSILON &&
          fabsf(left.w - right.w) <= QUAT_EPSILON);
}
bool operator!=(const quat &a, const quat &b) { return !(a == b); }
bool sameOrientation(const quat &left, const quat &right) {
  return (fabsf(left.x - right.x) <= QUAT_EPSILON &&
          fabsf(left.y - right.y) <= QUAT_EPSILON &&
          fabsf(left.z - right.z) <= QUAT_EPSILON &&
          fabsf(left.w - right.w) <= QUAT_EPSILON) ||
         (fabsf(left.x + right.x) <= QUAT_EPSILON &&
          fabsf(left.y + right.y) <= QUAT_EPSILON &&
          fabsf(left.z + right.z) <= QUAT_EPSILON &&
          fabsf(left.w + right.w) <= QUAT_EPSILON);
}

float dot(const quat &a, const quat &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

float lenSq(const quat &l) {
  return l.x * l.x + l.y * l.y + l.z * l.z + l.w * l.w;
}

float len(const quat &l) {
  float lenSqu = lenSq(l);
  if (lenSqu < QUAT_EPSILON) {
    return 0.0f;
  }
  return sqrtf(lenSqu);
}

void normalize(quat &v) {
  if (lenSq(v) < QUAT_EPSILON) {
    return;
  }
  float invLen = 1.0f / sqrtf(lenSq(v));
  v.x *= invLen;
  v.y *= invLen;
  v.z *= invLen;
  v.w *= invLen;
}

quat normalized(const quat &v) {
  if (lenSq(v) < QUAT_EPSILON) {
    return quat();
  }
  quat res(v.x, v.y, v.z, v.w);
  normalize(res);
  return res;
}

quat conjugate(const quat &q) { return quat(-q.x, -q.y, -q.z, q.w); }
quat inverse(const quat &q) {
  float lenSqu = lenSq(q);
  if (lenSqu < QUAT_EPSILON) {
    return quat();
  }
  float recip = 1.0f / lenSqu;
  return quat(-q.x * recip, -q.y * recip, -q.z * recip, q.w * recip);
}

quat operator*(const quat &Q1, const quat &Q2) {
  return quat(                                                //
      Q2.x * Q1.w + Q2.y * Q1.z - Q2.z * Q1.y + Q2.w * Q1.x,  //
      -Q2.x * Q1.z + Q2.y * Q1.w + Q2.z * Q1.x + Q2.w * Q1.y, //
      Q2.x * Q1.y - Q2.y * Q1.x + Q2.z * Q1.w + Q2.w * Q1.z,  //
      -Q2.x * Q1.x - Q2.y * Q1.y - Q2.z * Q1.z + Q2.w * Q1.w  //
  );
  /* quat result;
  result.data.scalar =
      Q2.data.scalar * Q1.data.scalar - dot(Q2.data.vector, Q1.data.vector);
  result.data.vector = (Q1.data.vector * Q2.data.scalar) +
                       (Q2.data.vector * Q1.data.scalar) +
                       cross(Q2.data.vector, Q1.data.vector);
  return result;*/
}

vec3 operator*(const quat &q, const vec3 &v) {
  return q.data.vector * 2.0f * dot(q.data.vector, v) +
         v * (q.data.scalar * q.data.scalar -
              dot(q.data.vector, q.data.vector)) +
         cross(q.data.vector, v) * 2.0f * q.data.scalar;
}

quat operator*(const quat &a, float b) {
  return quat(a.x * b, a.y * b, a.z * b, a.w * b);
}

quat mix(const quat &from, const quat &to, float t) {
  return from * (1.0f - t) + to * t;
}

quat nlerp(const quat &from, const quat &to, float t) {
  return normalized(from + (to - from) * t);
}

quat slerp(const quat &start, const quat &end, float t) {
  if (fabs(dot(start, end)) > 1.0f - QUAT_EPSILON) {
    return nlerp(start, end, t);
  }
  quat delta = inverse(start) * end;
  return normalized((delta ^ t) * start);
}

quat operator^(const quat &q, float f) {
  float angle = 2.0f * acosf(q.data.scalar);
  vec3 axis = normalized(q.data.vector);
  float halfCos = cosf(f * angle * 0.5f);
  float halfSin = sinf(f * angle * 0.5f);
  return quat(axis.x * halfSin, axis.y * halfSin, axis.z * halfSin, halfCos);
}

quat lookRotation(const vec3 &direction, const vec3 &up) {
  // Find orhtonormal basis vectors
  vec3 f = normalized(direction); // Object forward
  vec3 u = normalized(up);        // Desired up
  vec3 r = cross(u, f);           // Object Right

  u = cross(f, r); // Object up

  // From world to object forward
  quat worldToObject = fromTo(vec3(0, 0, 1), f);
  // what direction is the new object up?
  vec3 objectUp = worldToObject * vec3(0, 1, 0);

  // From object to desired up
  quat u2u = fromTo(objectUp, u);
  // Rotate to forward direction first
  // then twist to correct up
  quat result = worldToObject * u2u;
  return normalized(result);
}

mat4 quatToMat4(const quat &q) {
  vec3 r = q * vec3(1, 0, 0);
  vec3 u = q * vec3(0, 1, 0);
  vec3 f = q * vec3(0, 0, 1);
  return mat4(          //
      r.x, r.y, r.z, 0, //
      u.x, u.y, u.z, 0, //
      f.x, f.y, f.z, 0, //
      0, 0, 0, 1);
}

quat mat4ToQuat(const mat4 &m) {
  vec3 up = normalized(vec3(m.vec.up.x, m.vec.up.y, m.vec.up.z));
  vec3 forward =
      normalized(vec3(m.vec.forward.x, m.vec.forward.y, m.vec.forward.z));
  vec3 right = cross(up, forward);
  up = cross(forward, right);
  return lookRotation(forward, up);
}
