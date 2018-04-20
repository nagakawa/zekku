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
}

#endif