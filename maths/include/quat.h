#pragma once
#include "mat4.h"
#include "vec3.h"

#define QUAT_EPSILON 0.000001f
#define QUAT_DEG2RAD 0.0174533f
#define QUAT_RAD2DEG 57.2958f

struct quat {
  union {
    struct {
      float x;
      float y;
      float z;
      float w;
    };
    struct {
      vec3 vector;
      float scalar;
    } data;
    float v[4];
  };
  inline quat() : x(0), y(0), z(0), w(1) {}
  inline quat(float _x, float _y, float _z, float _w)
      : x(_x), y(_y), z(_z), w(_w) {}
};

quat angleAxis(float angle, const vec3 &axis);

quat fromTo(const vec3 &from, const vec3 &to);
vec3 getAxis(const quat &quat);
float getAngle(const quat &quat);
quat operator+(const quat &a, const quat &b);
quat operator*(const quat &a, const quat &b);
vec3 operator*(const quat &q, const vec3 &v);
quat operator*(const quat &q, float b);
quat operator-(const quat &a, const quat &b);
bool operator==(const quat &left, const quat &right);
bool operator!=(const quat &left, const quat &right);
quat operator-(const quat &q);
quat operator*(const quat &a, float b);
quat operator^(const quat &a, float b);
bool sameOrientation(const quat &l, const quat &r);
float dot(const quat &a, const quat &b);
float lenSq(const quat &v);
float len(const quat &v);
void normalize(quat &v);
quat normalized(const quat &v);
quat conjugate(const quat &q);
quat inverse(const quat &q);
quat mix(const quat &from, const quat &to, float f);
quat nlerp(const quat &from, const quat &to, float f);
quat slerp(const quat &from, const quat &to, float f);
quat lookRotation(const vec3 &direction, const vec3 &up);
mat4 quatToMat4(const quat &q);
quat mat4ToQuat(const mat4 &m);
