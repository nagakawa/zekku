#pragma once

#ifndef ZEKKU_QUADTREE_H
#define ZEKKU_QUADTREE_H
#include <stddef.h>
#include <stdint.h>
#include <limits>
#include <type_traits>
#include "zekku/Pool.h"

namespace zekku {
  template<typename F = float>
  struct AABB {
    static_assert(std::is_floating_point<F>::value,
      "Your F is not a floating-point number, dum dum!");
    F x, y;
    F w, h;
  };
  constexpr size_t QUADTREE_NODE_COUNT = 4;
  template<
    typename T,
    typename I = uint16_t,
    typename F = float,
    size_t nc = QUADTREE_NODE_COUNT
  >
  class QuadTree {
  public:
    static_assert(std::is_integral<I>::value,
      "Your I is not an integer, dum dum!");
    static_assert(std::is_unsigned<I>::value,
      "Don't use a signed int for sizes, dum dum!");
    static_assert(std::is_floating_point<F>::value,
      "Your F is not a floating-point number, dum dum!");
    QuadTree(const AABB<F>& box) : box(box) {
      root = (I) nodes.allocate();
    }
  private:
    static constexpr I NOWHERE = -1U;
    class Node {
    public:
      Node() :
        nw(NOWHERE), sw(NOWHERE), ne(NOWHERE), se(NOWHERE),
        nodeCount(0) {}
      T nodes[nc];
      I nodeCount;
      I nw, sw, ne, se;
    };
    Pool<Node> nodes;
    I root;
    AABB<F> box;
  };
}
#endif