#pragma once

#ifndef ZEKKU_QUADTREE_H
#define ZEKKU_QUADTREE_H
#include <stddef.h>
#include <stdint.h>
#include <limits>
#include <type_traits>
#include "zekku/Pool.h"

namespace zekku {
  template<typename T, typename F = float>
  struct DefaultGetXY {
    static_assert(std::is_floating_point<F>::value,
      "Your F is not a floating-point number, dum dum!");
    F getX(T t) { return t.x; }
    F getY(T t) { return t.y; }
  };
  template<typename F = float>
  struct AABB {
    static_assert(std::is_floating_point<F>::value,
      "Your F is not a floating-point number, dum dum!");
    F x, y;
    F w, h;
    AABB<F> nw() { return {x - w / 2, y - h / 2, w / 2, h / 2}; }
    AABB<F> ne() { return {x + w / 2, y - h / 2, w / 2, h / 2}; }
    AABB<F> sw() { return {x - w / 2, y + h / 2, w / 2, h / 2}; }
    AABB<F> se() { return {x + w / 2, y + h / 2, w / 2, h / 2}; }
  };
  constexpr size_t QUADTREE_NODE_COUNT = 4;
  template<
    typename T,
    typename I = uint16_t,
    typename F = float,
    size_t nc = QUADTREE_NODE_COUNT,
    typename GetXY = DefaultGetXY<T, F>
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
    void insert(T&& t) {
      insert(t, GetXY::getX(t), GetXY::getY(t), root, box);
    }
  private:
    static constexpr I NOWHERE = -1U;
    class Node {
    public:
      Node() :
        nw(NOWHERE), sw(NOWHERE), ne(NOWHERE), se(NOWHERE),
        nodeCount(0) {}
      T nodes[nc];
      I nodeCount; // Set to NOWHERE if not a leaf.
      I nw, sw, ne, se;
    };
    Pool<Node> nodes;
    I root;
    AABB<F> box;
    void insertStem(T&& t, F x, F y, Node& n, AABB<F> box) {
      if (x < box.x) { // West
        if (y < box.y) insert(t, x, y, n.nw, box.nw());
        else insert(t, x, y, n.sw, box.sw());
      } else { // East
        if (y < box.y) insert(t, x, y, n.ne, box.ne());
        else insert(t, x, y, n.se, box.se());
      }
    }
    // Insert an element in the qtree
    void insert(T&& t, F x, F y, I root, AABB<F> box) {
      Node& n = nodes.get(root);
      if (n.nodeCount == NOWHERE) {
        insertStem(t, x, y, n, box);
      } else if (n.nodeCount < nc) {
        n.nodes[n.nodeCount] = t;
        ++n.nodeCount;
      } else {
        // Leaf is full!
        // Split into multiple trees.
        n.nw = nodes.allocate();
        n.ne = nodes.allocate();
        n.sw = nodes.allocate();
        n.se = nodes.allocate();
        n.nodeCount = -1;
        for (size_t i = 0; i < nc; ++i)
          insertStem(std::move(nodes[i]), x, y, n, box);
        insertStem(t, x, y, n, box);
      }
    }
  };
}
#endif