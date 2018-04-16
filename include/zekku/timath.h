#pragma once

#ifndef ZEKKU_TIMATH_H
#define ZEKKU_TIMATH_H

#include <cmath>
#include <type_traits>

namespace zekku {
  template<typename T>
  constexpr std::enable_if_t<std::is_floating_point<T>::value, T>
  abs(T x) {
    return std::abs(x);
  }
  template<typename T>
  constexpr std::enable_if_t<std::is_floating_point<T>::value, T>
  isWithin(T x, T y, T r) {
    return x * x + y * y <= r * r;
  }
  template<typename T>
  constexpr std::enable_if_t<std::is_floating_point<T>::value, T>
  oneHalf = T(0.5);
}

#endif