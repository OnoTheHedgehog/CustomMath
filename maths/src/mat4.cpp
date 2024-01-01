#include "mat4.h"
#include <iostream>
#include <math.h>

#define M4D(aRow, bCol)                                                        \
  a.v[0 * 4 + aRow] * b.v[bCol * 4 + 0] +                                      \
      a.v[1 * 4 + aRow] * b.v[bCol * 4 + 1] +                                  \
      a.v[2 * 4 + aRow] * b.v[bCol * 4 + 2] +                                  \
      a.v[3 * 4 + aRow] * b.v[bCol * 4 + 3]

#define M4V4D(mRow, x, y, z, w)                                                \
  m.v[0 * 4 + mRow] * x + m.v[1 * 4 + mRow] * y + m.v[2 * 4 + mRow] * z +      \
      m.v[3 * 4 + mRow] * w

bool operator==(const mat4 &a, const mat4 &b) {
  for (int i = 0; i < 16; ++i) {
    if (fabsf(a.v[i] - b.v[i]) > MAT4_EPSILON) {
      return false;
    }
  }

  return true;
}

bool operator!=(const mat4 &a, const mat4 &b) { return !(a == b); }
mat4 operator+(const mat4 &a, const mat4 &b) {

  return mat4(a.xx + b.xx, a.xy + b.xy, a.xz + b.xz, a.xw + b.xw, //
              a.yx + b.yx, a.yy + b.yy, a.yz + b.yz, a.yw + b.yw, //
              a.zx + b.zx, a.zy + b.zy, a.zz + b.zz, a.zw + b.zw, //
              a.tx + b.tx, a.ty + b.ty, a.tz + b.tz, a.tw + b.tw  //
  );
}

mat4 operator*(const mat4 &a, float f) {

  return mat4(a.xx * f, a.xy * f, a.xz * f, a.xw * f, //
              a.yx * f, a.yy * f, a.yz * f, a.yw * f, //
              a.zx * f, a.zy * f, a.zz * f, a.zw * f, //
              a.tx * f, a.ty * f, a.tz * f, a.tw * f  //
  );
}

mat4 operator*(const mat4 &a, const mat4 &b) {
  return mat4(M4D(0, 0), M4D(1, 0), M4D(2, 0), M4D(3, 0), // Column 0
              M4D(0, 1), M4D(1, 1), M4D(2, 1), M4D(3, 1), // Column 1
              M4D(0, 2), M4D(1, 2), M4D(2, 2), M4D(3, 2), // Column 2
              M4D(0, 3), M4D(1, 3), M4D(2, 3), M4D(3, 3)  // Column 3
  );
}

vec4 operator*(const mat4 &m, const vec4 &v) {
  return vec4(M4V4D(0, v.x, v.y, v.z, v.w), //
              M4V4D(1, v.x, v.y, v.z, v.w), //
              M4V4D(2, v.x, v.y, v.z, v.w), //
              M4V4D(3, v.x, v.y, v.z, v.w));
}

vec3 transformVector(const mat4 &m, const vec3 &v) {
  return vec3(M4V4D(0, v.x, v.y, v.z, 0.0f), //
              M4V4D(1, v.x, v.y, v.z, 0.0f), //
              M4V4D(2, v.x, v.y, v.z, 0.0f));
}

vec3 transformPoint(const mat4 &m, const vec3 &v) {
  return vec3(M4V4D(0, v.x, v.y, v.z, 1.0f), //
              M4V4D(1, v.x, v.y, v.z, 1.0f), //
              M4V4D(2, v.x, v.y, v.z, 1.0f));
}

vec3 transformPoint(const mat4 &m, const vec3 &v, float &w) {
  float _w = w;
  w = M4V4D(3, v.x, v.y, v.z, _w);
  return vec3(M4V4D(0, v.x, v.y, v.z, _w), //
              M4V4D(1, v.x, v.y, v.z, _w), //
              M4V4D(2, v.x, v.y, v.z, _w));
}
#define M4SWAP(x, y)                                                           \
  {                                                                            \
    float t = x;                                                               \
    x = y;                                                                     \
    y = t;                                                                     \
  }

#define M4_3X3MINOR(c0, c1, c2, r0, r1, r2)                                    \
  (m.v[c0 * 4 + r0] * (m.v[c1 * 4 + r1] * m.v[c2 * 4 + r2] -                   \
                       m.v[c1 * 4 + r2] * m.v[c2 * 4 + r1]) -                  \
   m.v[c1 * 4 + r0] * (m.v[c0 * 4 + r1] * m.v[c2 * 4 + r2] -                   \
                       m.v[c0 * 4 + r2] * m.v[c2 * 4 + r1]) +                  \
   m.v[c2 * 4 + r0] * (m.v[c0 * 4 + r1] * m.v[c1 * 4 + r2] -                   \
                       m.v[c0 * 4 + r2] * m.v[c1 * 4 + r1]))

void transpose(mat4 &m) {

  M4SWAP(m.xy, m.yx);
  M4SWAP(m.xz, m.zx);
  M4SWAP(m.xw, m.tx);
  M4SWAP(m.zy, m.yz);
  M4SWAP(m.ty, m.yw);
  M4SWAP(m.tz, m.zw);
}

mat4 transposed(const mat4 &m) {
  return mat4(m.xx, m.yx, m.zx, m.tx, //
              m.xy, m.yy, m.zy, m.ty, //
              m.xz, m.yz, m.zz, m.tz, //
              m.xw, m.yw, m.zw, m.tw);
}

float determinant(const mat4 &m) {
  return m.v[0] * M4_3X3MINOR(1, 2, 3, 1, 2, 3) -
         m.v[4] * M4_3X3MINOR(0, 2, 3, 1, 2, 3) +
         m.v[8] * M4_3X3MINOR(0, 1, 3, 1, 2, 3) -
         m.v[12] * M4_3X3MINOR(0, 1, 2, 1, 2, 3);
}

mat4 adjugate(const mat4 &m) {
  // Cofactor(M[i, j]) = Minor(M[i, j]] * pow(-1, i + j)
  mat4 cofactor;

  cofactor.v[0] = M4_3X3MINOR(1, 2, 3, 1, 2, 3);
  cofactor.v[1] = -M4_3X3MINOR(1, 2, 3, 0, 2, 3);
  cofactor.v[2] = M4_3X3MINOR(1, 2, 3, 0, 1, 3);
  cofactor.v[3] = -M4_3X3MINOR(1, 2, 3, 0, 1, 2);

  cofactor.v[4] = -M4_3X3MINOR(0, 2, 3, 1, 2, 3);
  cofactor.v[5] = M4_3X3MINOR(0, 2, 3, 0, 2, 3);
  cofactor.v[6] = -M4_3X3MINOR(0, 2, 3, 0, 1, 3);
  cofactor.v[7] = M4_3X3MINOR(0, 2, 3, 0, 1, 2);

  cofactor.v[8] = M4_3X3MINOR(0, 1, 3, 1, 2, 3);
  cofactor.v[9] = -M4_3X3MINOR(0, 1, 3, 0, 2, 3);
  cofactor.v[10] = M4_3X3MINOR(0, 1, 3, 0, 1, 3);
  cofactor.v[11] = -M4_3X3MINOR(0, 1, 3, 0, 1, 2);

  cofactor.v[12] = -M4_3X3MINOR(0, 1, 2, 1, 2, 3);
  cofactor.v[13] = M4_3X3MINOR(0, 1, 2, 0, 2, 3);
  cofactor.v[14] = -M4_3X3MINOR(0, 1, 2, 0, 1, 3);
  cofactor.v[15] = M4_3X3MINOR(0, 1, 2, 0, 1, 2);

  return transposed(cofactor);
}

mat4 inverse(const mat4 &m) {

  float det = determinant(m);

  if (det == 0.0f) {
    std::cout << "Matrix determinant is 0"
              << "\n";
    return mat4();
  }

  mat4 adj = adjugate(m);
  return adj * (1 / det);
}

void invert(mat4 &m) {
  float det = determinant(m);
  if (det == 0.0f) {
    std::cout << "Matrix determinant is 0"
              << "\n";
    m = mat4();
  }

  m = adjugate(m) * (1 / det);
}

mat4 frustum(float l, float r, float b, float t, float n, float f) {
  if (l == r || t == b || n == f) {
    std::cout << "WARNING: Trying to create invalid frustum\n";
    return mat4(); // Error
  }
  return mat4((2.0f * n) / (r - l), 0, 0, 0, 0, (2.0f * n) / (t - b), 0, 0,
              (r + l) / (r - l), (t + b) / (t - b), (-(f + n)) / (f - n), -1, 0,
              0, (-2 * f * n) / (f - n), 0);
}

mat4 perspective(float fov, float aspect, float znear, float zfar) {
  float ymax = znear * tanf(fov * 3.14159265359f / 360.0f);
  float xmax = ymax * aspect;

  return frustum(-xmax, xmax, -ymax, ymax, znear, zfar);
}

mat4 ortho(float l, float r, float b, float t, float n, float f) {

  if (l == r || t == b || n == f) {
    std::cout << "Invalid frustrum"
              << "\n";
    return mat4();
  }
  return mat4(2.0f / (r - l), 0, 0, 0,  //
              0, 2.0f / (t - b), 0, 0,  //
              0, 0, -2.0f / (f - n), 0, //
              -((r + l) / (r - l)), -((t + b) / (t - b)), -((f + n) / (f - n)),
              1);
}

mat4 lookAt(const vec3 &position, const vec3 &target, const vec3 &up) {
  // Remember, forward is negative z
  vec3 f = normalized(target - position) * -1.0f;
  vec3 r = cross(up, f); // Right handed
  if (r == vec3(0, 0, 0)) {
    return mat4(); // Error
  }
  normalize(r);
  vec3 u = normalized(cross(f, r)); // Right handed

  vec3 t = vec3(0.0f - dot(r, position), 0.0f - dot(u, position),
                0.0f - dot(f, position));

  return mat4(
      // Transpose upper 3x3 matrix to invert it
      r.x, u.x, f.x, 0, r.y, u.y, f.y, 0, r.z, u.z, f.z, 0, t.x, t.y, t.z, 1);
}

std::ostream &operator<<(std::ostream &stream, const mat4 &m) {

  std::cout << m.vec.right.x << " " << m.vec.up.x << " " << m.vec.forward.x
            << " " << m.vec.position.x << "\n";
  std::cout << m.vec.right.y << " " << m.vec.up.y << " " << m.vec.forward.y
            << " " << m.vec.position.y << "\n";
  std::cout << m.vec.right.z << " " << m.vec.up.z << " " << m.vec.forward.z
            << " " << m.vec.position.z << "\n";
  std::cout << m.vec.right.w << " " << m.vec.up.w << " " << m.vec.forward.w
            << " " << m.vec.position.w << "\n";
  return stream;
}
