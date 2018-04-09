#pragma once

#ifndef ZEKKU_BOX_BITWISE_H
#define ZEKKU_BOX_BITWISE_H

#include <stdint.h>

namespace zekku {
  static const int MultiplyDeBruijnBitPosition[32] =
  {
    0, 9, 1, 10, 13, 21, 2, 29, 11, 14, 16, 18, 22, 25, 3, 30,
    8, 12, 20, 28, 15, 17, 24, 7, 19, 27, 23, 6, 26, 5, 4, 31
  };

  // Thanks https://graphics.stanford.edu/~seander/bithacks.html#IntegerLogDeBruijn
  inline int log2(uint32_t v) {
    v |= v >> 1; // first round down to one less than a power of 2
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    return MultiplyDeBruijnBitPosition[(uint32_t)(v * 0x07C4ACDDU) >> 27];
  }
  inline int log2up(uint32_t v) {
    --v;
    v |= v >> 1; // first round down to one less than a power of 2
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    return 1 + MultiplyDeBruijnBitPosition[(uint32_t)(v * 0x07C4ACDDU) >> 27];
  }
}

#endif