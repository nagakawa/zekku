#pragma once

#ifndef ZEKKU_QUADTREE_H
#define ZEKKU_QUADTREE_H
#include <stddef.h>
#include <stdint.h>
#include <iostream>
#include <limits>
#include <type_traits>
#include "zekku/Pool.h"
#include "zekku/Vec.h"

namespace zekku {
  template<typename T, typename F = float>
  struct DefaultGetXY {
    static_assert(std::is_floating_point<F>::value,
      "Your F is not a floating-point number, dum dum!");
    static F getX(T t) { return t.x; }
    static F getY(T t) { return t.y; }
    static Vec2<F> getPos(T t) { return {t.x, t.y}; }
  };
  template<typename F = float>
  struct AABB {
    static_assert(std::is_floating_point<F>::value,
      "Your F is not a floating-point number, dum dum!");
    Vec2<F> c;
    Vec2<F> s; // centre to corner
    AABB<F> nw() { return {c - s / 2, s / 2}; }
    AABB<F> ne() { return {c + s * Vec2<F>{1.0f, -1.0f} / 2, s / 2}; }
    AABB<F> sw() { return {c + s * Vec2<F>{-1.0f, 1.0f} / 2, s / 2}; }
    AABB<F> se() { return {c + s / 2, s / 2}; }
    bool contains(Vec2<F> p) {
      return
        p[0] >= c[0] - s[0] &&
        p[0] <= c[0] + s[0] &&
        p[1] >= c[1] - s[1] &&
        p[1] <= c[1] + s[1];
    }
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
    void insert(const T& t) {
      T t2 = t;
      insert(std::move(t2));
    }
    void insert(T&& t) {
      Vec2<F> p = GetXY::getPos(t);
      if (!box.contains(p)) {
        std::cerr << "(" << p[0] << ", " << p[1] << ") is out of range!\n";
        std::cerr << "Box is centred at (" << box.c[0] << ", " << box.c[1] << ") ";
        std::cerr << "with w = " << box.s[0] << " and h = " << box.s[1] << "\n";
        exit(-1);
      }
      insert(std::move(t), p, root, box);
    }
  private:
    static constexpr I NOWHERE = -1;
    class Node {
    public:
      Node() :
        nw(NOWHERE), sw(NOWHERE), ne(NOWHERE), se(NOWHERE),
        nodeCount(0) {}
      T nodes[nc];
      I nw, sw, ne, se;
      I nodeCount; // Set to NOWHERE if not a leaf.
    };
    Pool<Node> nodes;
    I root;
    AABB<F> box;
    void insertStem(T&& t, Vec2<F> p, Node& n, AABB<F> box) {
      if (p[0] < box.c[0]) { // West
        if (p[1] < box.c[1]) insert(std::move(t), p, n.nw, box.nw());
        else insert(std::move(t), p, n.sw, box.sw());
      } else { // East
        if (p[1] < box.c[1]) insert(std::move(t), p, n.ne, box.ne());
        else insert(std::move(t), p, n.se, box.se());
      }
    }
    // Insert an element in the qtree
    void insert(T&& t, Vec2<F> p, I root, AABB<F> box) {
      Node& n = nodes.get(root);
      if (n.nodeCount == NOWHERE) {
        insertStem(std::move(t), p, n, box);
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
          insertStem(std::move(n.nodes[i]), p, n, box);
        insertStem(std::move(t), p, n, box);
      }
    }
  };
}
#endif