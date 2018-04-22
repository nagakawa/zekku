#pragma once

#ifndef ZEKKU_KFP_INTEROP_TIMATH_H
#define ZEKKU_KFP_INTEROP_TIMATH_H

#include <cmath>
#include <type_traits>

#include <kozet_fixed_point/kfp.h>
#include <kozet_fixed_point/kfp_extra.h>

namespace zekku {
  template<typename I, size_t d>
  struct TIMath<kfp::Fixed<I, d>> {
    using T = kfp::Fixed<I, d>;
    static constexpr T abs(T x) {
      return (x.underlying < 0) ? -x : x;
    }
    static constexpr bool isWithin(T x, T y, T r) {
      return kfp::isInterior(x, y, r);
    }
    static constexpr T oneHalf = T::raw(1 << (d - 1));
    typedef kfp::DoubleTypeExact<T> DoubleType;
    static T hypot(T x, T y) {
      return kfp::hypot(x, y);
    }
    static T sqrt(DoubleType x) {
      return kfp::sqrt<typename T::Underlying, T::fractionalBits()>(x);
    }
  };
}

#endif