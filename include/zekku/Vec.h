#pragma once

#ifndef ZEKKU_VEC_H
#define ZEKKU_VEC_H

#include <array>

namespace zekku {
  template<typename T, size_t n>
  class Vec {
  public:
    template<typename... T2>
    Vec(T2... v) : under{v...} {
      static_assert(sizeof(Vec<T, n>) == n * sizeof(T),
        "Basic sanity check; this should pass");
    }
    Vec(const Vec<T, n>& other) : under(other.under) {}
    Vec<T, n>& operator=(const Vec<T, n>& other) {
      under = other.under;
      return *this;
    }
    T& operator[](size_t i) { return under[i]; }
    const T& operator[](size_t i) const { return under[i]; }
#define DEF_OPERATOR(o) \
    Vec<T, n> operator o(const Vec<T, n>& other) const { \
      Vec<T, n> q; \
      for (size_t i = 0; i < n; ++i) \
        q[i] = under[i] o other[i]; \
      return q; \
    } \
    Vec<T, n>& operator o##=(const Vec<T, n>& other) { \
      for (size_t i = 0; i < n; ++i) \
        under[i] o##= other[i]; \
      return *this; \
    } \
    Vec<T, n> operator o(T other) const { \
      Vec<T, n> q; \
      for (size_t i = 0; i < n; ++i) \
        q[i] = under[i] o other; \
      return q; \
    } \
    Vec<T, n>& operator o##=(T other) { \
      for (size_t i = 0; i < n; ++i) \
        under[i] o##= other; \
      return *this; \
    }
    DEF_OPERATOR(+)
    DEF_OPERATOR(-)
    DEF_OPERATOR(*)
    DEF_OPERATOR(/)
#undef DEF_OPERATOR
    T dot(const Vec<T, n>& other) const {
      T r = 0;
      for (size_t i = 0; i < n; ++i)
        r += under[i] * other[i];
      return r;
    }
    T r2() const {
      return dot(*this);
    }
#define DEF_FIELD(x, o, so) \
    T x() const { \
      static_assert(o < n, \
        "This method is available only to vectors with at least " \
        #so " elements"); \
      return under[o]; \
    } \
    T& x() { \
      static_assert(o < n, \
        "This method is available only to vectors with at least " \
        #so " elements"); \
      return under[o]; \
    }
    DEF_FIELD(x, 0, 1)
    DEF_FIELD(y, 1, 2)
    DEF_FIELD(z, 2, 3)
    DEF_FIELD(w, 3, 4)
#undef DEF_FIELD
  private:
    std::array<T, n> under;
  };
  template<typename T>
  using Vec2 = Vec<T, 2>;
}

#endif