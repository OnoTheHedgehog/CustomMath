#pragma once

template <typename T> struct Tvec2 {
  union {
    struct {
      T x;
      T y;
    };
    T v[2];
  };
  inline Tvec2() : x(0.0f), y(0.0f) {}
  inline Tvec2(T _x, T _y) : x(_x), y(_y) {}
  inline Tvec2(T *fv) : x(fv[0]), y(fv[1]) {}
};

typedef Tvec2<int> ivec2;
typedef Tvec2<float> vec2;
