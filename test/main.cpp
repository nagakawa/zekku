#include <stdio.h>

#include "zekku/Pool.h"

constexpr size_t hc = 64;

int main() {
  printf("Testing...\n");
  zekku::Pool<size_t> p;
  size_t handles[hc];
  for (size_t i = 0; i < hc; ++i) {
    size_t h = p.allocate();
    p.get(h) = 35 * i;
    handles[i] = h;
  }
  for (size_t i = 0; i < hc; ++i) {
    size_t val = p.get(handles[i]);
    printf("i = %zu: got %zu, expected %zu\n", i, val, 35 * i);
  }
  return 0;
}