#pragma once

#ifndef ZEKKU_TIMATH_H
#define ZEKKU_TIMATH_H

#include <cmath>
#include <type_traits>

namespace zekku {
  template<typename T> struct TIMath;
  template<typename T>
  struct TIMathFloat {
    static constexpr T abs(T x) {
      return std::abs(x);
    }
    static constexpr bool isWithin(T x, T y, T r) {
      return x * x + y * y <= r * r;
    }
    static constexpr T oneHalf = T(0.5);
    typedef long double DoubleType;
    static T hypot(T x, T y) {
      return std::hypot(x, y);
    }
    static T sqrt(DoubleType x) {
      return (T) std::sqrt(x);
    }
    static constexpr DoubleType longMultiply(T x, T y) {
      return x * y;
    }
  };
  template<> struct TIMath<float> : TIMathFloat<float> {};
  template<> struct TIMath<double> : TIMathFloat<double> {};
  template<> struct TIMath<long double> : TIMathFloat<long double> {};
  template<typename T>
  constexpr T abs(T x) {
    return TIMath<T>::abs(x);
  }
  template<typename T>
  constexpr bool isWithin(T x, T y, T r) {
    return TIMath<T>::isWithin(x, y, r);
  }
  template<typename T>
  constexpr T oneHalf = TIMath<T>::oneHalf;
  template<typename T>
  using DoubleType = typename TIMath<T>::DoubleType;
  template<typename T>
  T hypot(T x, T y) {
    return TIMath<T>::hypot(x, y);
  }
  template<typename T>
  T sqrt(DoubleType<T> x) {
    return TIMath<T>::sqrt(x);
  }
  template<typename T>
  constexpr DoubleType<T> longMultiply(T x, T y) {
    return TIMath<T>::longMultiply(x, y);
  }
  template<typename F>
  DoubleType<F> cross2(const glm::tvec2<F>& a, const glm::tvec2<F>& b) {
    return
      ((DoubleType<F>) a.x) * ((DoubleType<F>) b.y) -
      ((DoubleType<F>) a.y) * ((DoubleType<F>) b.x);
  }
  template<typename F>
  DoubleType<F> dotUnfucked(const glm::tvec2<F>& a, const glm::tvec2<F>& b) {
    return
      ((DoubleType<F>) a.x) * ((DoubleType<F>) b.x) +
      ((DoubleType<F>) a.y) * ((DoubleType<F>) b.y);
  }
}

#endif