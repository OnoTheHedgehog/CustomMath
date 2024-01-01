#pragma once

template <typename T> struct Tvec4 {
  union {
    struct {
      T x;
      T y;
      T z;
      T w;
    };
    T v[4];
  };
  inline Tvec4() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {}
  inline Tvec4(T _x, T _y, T _z, T _w) : x(_x), y(_y), z(_z), w(_w) {}
  inline Tvec4(T *fv) : x(fv[0]), y(fv[1]), z(fv[2]), w(fv[3]) {}
};

typedef Tvec4<int> ivec4;
typedef Tvec4<float> vec4;
typedef Tvec4<unsigned int> uivec4;
