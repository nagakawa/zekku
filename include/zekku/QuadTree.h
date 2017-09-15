#pragma once

#ifndef ZEKKU_QUADTREE_H
#define ZEKKU_QUADTREE_H
#include <stdint.h>
#include <stddef.h>
#include <limits>
#include "zekku/Pool.h"

namespace zekku {
  constexpr size_t QUADTREE_NODE_COUNT = 4;
  template<typename T, typename I = uint16_t, size_t nc = QUADTREE_NODE_COUNT>
  class QuadTree {
  public:
  private:
    static constexpr I NOWHERE = -1;
    class Node {
    public:
      Node() :
        nw(NOWHERE), sw(NOWHERE), ne(NOWHERE), se(nowhere),
        nodeCount(0) {}
      T nodes[nc];
      I nodeCount;
      I nw, sw, ne, se;
    };
    Pool<Node> nodes;
    I root;
  };
}
#endif