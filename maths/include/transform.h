#pragma once

#include "quat.h"
#include <ostream>
struct Transform {
  vec3 position;

  quat rotation;

  vec3 scale;

  Transform(const vec3 &p, const quat &r, const vec3 &s)
      : position(p), rotation(r), scale(s) {}
  Transform()
      : position(vec3(0, 0, 0)), rotation(quat()), scale(vec3(1, 1, 1)) {}
};

Transform combine(const Transform &a, const Transform &b);
Transform mix(const Transform &a, const Transform &b, float t);
Transform inverse(const Transform &t);
mat4 transformToMat4(const Transform &t);
Transform toTransform(const mat4 &t);
vec3 transformPoint(const Transform &a, const vec3 &b);
vec3 transformVector(const Transform &a, const vec3 &b);
std::ostream &operator<<(std::ostream &stream, const Transform &m);
