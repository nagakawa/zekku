#pragma once

#ifndef ZEKKU_QUADTREE_H
#define ZEKKU_QUADTREE_H
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <algorithm>
#include <cmath>
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
    AABB<F> nw() const { return {c - s / 2, s / 2}; }
    AABB<F> ne() const { return {c + s * Vec2<F>{1.0f, -1.0f} / 2, s / 2}; }
    AABB<F> sw() const { return {c + s * Vec2<F>{-1.0f, 1.0f} / 2, s / 2}; }
    AABB<F> se() const { return {c + s / 2, s / 2}; }
    Vec2<F> nwp() const { return c - s; }
    Vec2<F> nep() const { return c + s * Vec2<F>{1.0f, -1.0f}; }
    Vec2<F> swp() const { return c + s * Vec2<F>{-1.0f, 1.0f}; }
    Vec2<F> sep() const { return c + s; }
    bool contains(Vec2<F> p) const {
      return
        p[0] >= c[0] - s[0] &&
        p[0] <= c[0] + s[0] &&
        p[1] >= c[1] - s[1] &&
        p[1] <= c[1] + s[1];
    }
    bool intersects(const AABB<F>& p) const {
      return
        (std::abs(c[0] - p.c[0]) <= (s[0] + p.s[0])) &&
        (std::abs(c[1] - p.c[1]) <= (s[1] + p.s[1]));
    }
  };
  template<typename F = float>
  struct CircleQuery {
    CircleQuery(const Vec2<F>& c, F r) : c(c), r(r) {}
    Vec2<F> c;
    F r;
    bool contains(Vec2<F> p) const {
      return (c - p).r2() <= r * r;
    }
    /*
    This method considers a rounded rectangle around the AABB with
    thickness r. This figure can be thought of as the union of
    four circles and two rectangles.
    thanks https://gamedev.stackexchange.com/a/120897
    */
    bool intersects(const AABB<F>& b) const {
      F r2 = r * r;
      return
        (c - b.nwp()).r2() <= r2 ||
        (c - b.swp()).r2() <= r2 ||
        (c - b.nep()).r2() <= r2 ||
        (c - b.sep()).r2() <= r2 ||
        (
          c.x() >= b.c.x() - b.s.x() &&
          c.x() <= b.c.x() + b.s.x() &&
          c.y() >= b.c.y() - b.s.y() - r &&
          c.y() <= b.c.y() + b.s.y() + r
        ) ||
        (
          c.x() >= b.c.x() - b.s.x() - r &&
          c.x() <= b.c.x() + b.s.x() + r &&
          c.y() >= b.c.y() - b.s.y() &&
          c.y() <= b.c.y() + b.s.y()
        );
    }
  };
  template<typename F = float>
  struct QueryAll {
    bool contains(Vec2<F> p) const { return true; }
    bool intersects(const AABB<F>& b) const { return true; }
  };
  template<typename I = uint16_t>
  struct Handle { I nodeid, index; };
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
      assert(nodes.getCapacity() <= 65536);
    }
    const T& deref(const Handle<I>& h) const {
      return nodes.get(h.nodeid).nodes[h.index];
    }
    T& deref(const Handle<I>& h) {
      return nodes.get(h.nodeid).nodes[h.index];
    }
    template<typename Q = AABB<T>>
    void query(const Q& shape, std::vector<Handle<I>>& out) const {
      query(shape, out, root, box);
    }
    void dump() const {
      dump(root, box);
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
    I createNode() {
      size_t i = nodes.allocate();
      nodes.get(i).nodeCount = 0;
      return (I) i;
    }
    void insertStem(T&& t, Vec2<F> p, Node& n, AABB<F> box) {
      if (p[0] < box.c[0]) { // West
        if (p[1] < box.c[1]) insert(std::move(t), p, n.nw, box.nw());
        else insert(std::move(t), p, n.sw, box.sw());
      } else { // East
        if (p[1] < box.c[1]) insert(std::move(t), p, n.ne, box.ne());
        else insert(std::move(t), p, n.se, box.se());
      }
    }
#define n (nodes.get(root))
    // Insert an element in the qtree
    void insert(T&& t, Vec2<F> p, I root, AABB<F> box) {
      if (n.nodeCount == NOWHERE) {
        insertStem(std::move(t), p, n, box);
      } else if (n.nodeCount < nc) {
        n.nodes[n.nodeCount] = t;
        ++n.nodeCount;
      } else {
        // Leaf is full!
        // Split into multiple trees.
        I nw = createNode();
        I ne = createNode();
        I sw = createNode();
        I se = createNode();
        n.nw = nw;
        n.ne = ne;
        n.sw = sw;
        n.se = se;
        n.nodeCount = NOWHERE;
        for (size_t i = 0; i < nc; ++i) {
          T& sub = n.nodes[i];
          Vec2<F> ps = GetXY::getPos(sub);
          insertStem(std::move(sub), ps, n, box);
        }
        insertStem(std::move(t), p, n, box);
      }
    }
#undef n
    template<typename Q = AABB<T>>
    void query(
        const Q& shape, std::vector<Handle<I>>& out,
        I root, AABB<F> box) const {
      // Abort if the query shape doesn't intersect the box
      if (!shape.intersects(box)) return;
      const Node& n = nodes.get(root);
      if (n.nodeCount == NOWHERE) {
        // It is a stem
        query(shape, out, n.nw, box.nw());
        query(shape, out, n.ne, box.ne());
        query(shape, out, n.sw, box.sw());
        query(shape, out, n.se, box.se());
      } else {
        // Leaf
        for (I i = 0; i < n.nodeCount; ++i) {
          if (shape.contains(GetXY::getPos(n.nodes[i])))
            out.push_back({root, i});
        }
      }
    }
    static void indent(size_t n) {
      for (size_t i = 0; i < n; ++i) std::cerr << ' ';
    }
    static void printAABB(const AABB<F>& box) {
      std::cerr << "[" << box.c[0] - box.s[0] << ", " << box.c[1] - box.s[1] <<
        "; " <<  box.c[0] + box.s[0] << ", " << box.c[1] + box.s[1] << "] ";
    }
    void dump(I root, AABB<F> box, size_t s = 0) const {
      const Node& n = nodes.get(root);
      if (n.nodeCount == NOWHERE) {
        std::cerr << "Stem "; printAABB(box); std::cerr << ":\n";
        indent(s); std::cerr << "NW "; dump(n.nw, box.nw(), s + 1);
        indent(s); std::cerr << "SW "; dump(n.sw, box.sw(), s + 1);
        indent(s); std::cerr << "NE "; dump(n.ne, box.ne(), s + 1);
        indent(s); std::cerr << "SE "; dump(n.se, box.se(), s + 1);
      } else {
        std::cerr << "Leaf "; printAABB(box); std::cerr << ":";
        for (size_t i = 0; i < n.nodeCount; ++i) {
          Vec2<F> p = GetXY::getPos(n.nodes[i]);
          std::cerr << " (" << p.x() << ", " << p.y() << ")";
        }
        std::cerr << "\n";
      }
    }
  };
}
#endif