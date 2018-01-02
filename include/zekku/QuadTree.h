#pragma once

#ifndef ZEKKU_QUADTREE_H
#define ZEKKU_QUADTREE_H
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <type_traits>
#include <glm/glm.hpp>
#include "zekku/Pool.h"

namespace zekku {
  template<typename T, typename F = float>
  struct DefaultGetXY {
    static_assert(std::is_floating_point<F>::value,
      "Your F is not a floating-point number, dum dum!");
    glm::tvec2<F> getPos(T t) const { return {t.x, t.y}; }
  };
  template<typename F = float>
  struct AABB {
    static_assert(std::is_floating_point<F>::value,
      "Your F is not a floating-point number, dum dum!");
    glm::tvec2<F> c;
    glm::tvec2<F> s; // centre to corner
    AABB<F> nw() const { return {c - s * F{0.5}, s * F{0.5}}; }
    AABB<F> ne() const { return {c + s * glm::tvec2<F>{0.5f, -0.5f}, s * F{0.5}}; }
    AABB<F> sw() const { return {c + s * glm::tvec2<F>{-0.5f, 0.5f}, s * F{0.5}}; }
    AABB<F> se() const { return {c + s * F{0.5}, s * F{0.5}}; }
    glm::tvec2<F> nwp() const { return c - s; }
    glm::tvec2<F> nep() const { return c + s * glm::tvec2<F>{1.0f, -1.0f}; }
    glm::tvec2<F> swp() const { return c + s * glm::tvec2<F>{-1.0f, 1.0f}; }
    glm::tvec2<F> sep() const { return c + s; }
    bool contains(glm::tvec2<F> p) const {
      return
        p.x >= c.x - s.x &&
        p.x <= c.x + s.x &&
        p.y >= c.y - s.y &&
        p.y <= c.y + s.y;
    }
    bool intersects(const AABB<F>& p) const {
      return
        (std::abs(c.x - p.c.x) <= (s.x + p.s.x)) &&
        (std::abs(c.y - p.c.y) <= (s.y + p.s.y));
    }
    size_t getClass(glm::tvec2<F> p) const {
      bool east = p.x > c.x;
      bool south = p.y > c.y;
      return (south << 1) | east;
    }
    AABB getSubboxByClass(size_t cl) const {
      bool east = (cl & 1);
      bool south = (cl & 2) >> 1;
      glm::tvec2<F> dir{
        (float) ((east << 1) - 1), (float) ((south << 1) - 1)};
      glm::tvec2<F> halfs = s * F{0.5};
      return {
        c + halfs * dir,
        halfs
      };
    }
  };
  template<typename F = float>
  struct CircleQuery {
    CircleQuery(const glm::tvec2<F>& c, F r) : c(c), r(r) {}
    glm::tvec2<F> c;
    F r;
    bool contains(glm::tvec2<F> p) const {
      return glm::dot(c - p, c - p) <= r * r;
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
        glm::dot(c - b.nwp(), c - b.nwp()) <= r2 ||
        glm::dot(c - b.swp(), c - b.swp()) <= r2 ||
        glm::dot(c - b.nep(), c - b.nep()) <= r2 ||
        glm::dot(c - b.sep(), c - b.sep()) <= r2 ||
        (
          c.x >= b.c.x - b.s.x &&
          c.x <= b.c.x + b.s.x &&
          c.y >= b.c.y - b.s.y - r &&
          c.y <= b.c.y + b.s.y + r
        ) ||
        (
          c.x >= b.c.x - b.s.x - r &&
          c.x <= b.c.x + b.s.x + r &&
          c.y >= b.c.y - b.s.y &&
          c.y <= b.c.y + b.s.y
        );
    }
  };
  template<typename F = float>
  struct QueryAll {
    bool contains(glm::tvec2<F> p) const { return true; }
    bool intersects(const AABB<F>& b) const { return true; }
  };
  template<typename I = uint16_t>
  struct Handle {
    I nodeid, index;
    bool operator==(const Handle<I>& other) const {
      return nodeid == other.nodeid && index == other.index;
    }
    bool operator<(const Handle<I>& other) const {
      if (nodeid < other.nodeid) return true;
      if (nodeid > other.nodeid) return false;
      return index < other.index;
    }
  };
  template<typename I = uint16_t>
  struct HandleHasher {
    size_t operator()(const Handle<I>& h) {
      return (std::hash<I>(h.nodeid) << 1) ^ std::hash<I>(h.index);
    }
  };
  constexpr size_t QUADTREE_NODE_COUNT = 32;
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
    template<typename... Args>
    QuadTree(const AABB<F>& box, Args&&... args) :
        root((I) nodes.allocate()), box(box), gxy(args...) {}
    QuadTree(QuadTree<T, I, F, nc, GetXY>&& other) :
        nodes(std::move(other.nodes)), root(other.root),
        box(other.box), gxy(other.gxy) {
      other.root = (I) other.nodes.allocate();
    }
    QuadTree& operator=(QuadTree<T, I, F, nc, GetXY>&& other) {
      nodes = std::move(other.nodes);
      root = other.root;
      other.root = (I) other.nodes.allocate();
      box = other.box;
      gxy = other.gxy;
      return *this;
    }
    Handle<I> insert(const T& t) {
      T t2 = t;
      return insert(std::move(t2));
    }
    Handle<I> insert(T&& t) {
      glm::tvec2<F> p = gxy.getPos(t);
      if (!box.contains(p)) {
        std::cerr << "(" << p[0] << ", " << p[1] << ") is out of range!\n";
        std::cerr << "Box is centred at (" << box.c[0] << ", " << box.c[1] << ") ";
        std::cerr << "with w = " << box.s[0] << " and h = " << box.s[1] << "\n";
        exit(-1);
      }
      Handle<I> h = insert(std::move(t), p, root, box);
      assert(nodes.getCapacity() <= std::numeric_limits<I>::max());
      return h;
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
    template<typename Q = AABB<T>, typename C>
    void query(const Q& shape, C callback) const {
      query(shape, callback, root, box);
    }
    template<typename Q = AABB<T>, typename C>
    void querym(const Q& shape, C callback) {
      querym(shape, callback, root, box);
    }
    template<typename C>
    QuadTree map(const C& f) const {
      QuadTree q(box, gxy);
      query(QueryAll<F>(), [&q, f](const T& t) {
        q.insert(f(t));
      });
      return q;
    }
    template<typename C>
    QuadTree mapm(const C& f) {
      QuadTree q(box, gxy);
      query(QueryAll<F>(), [&q, f](T&& t) {
        q.insert(f(std::move(t)));
      });
      return q;
    }
    template<typename C, typename P>
    QuadTree mapIf(const C& f, const P& b) const {
      QuadTree q(box, gxy);
      query(QueryAll<F>(), [&q, f, b](const T& t) {
        if (b(t)) q.insert(f(t));
      });
      return q;
    }
    template<typename C, typename P>
    QuadTree mapmIf(const C& f, const P& b) {
      QuadTree q(box, gxy);
      query(QueryAll<F>(), [&q, f, b](T&& t) {
        if (b(t)) q.insert(f(std::move(t)));
      });
      return q;
    }
    void dump() const {
      dump(root, box);
    }
  private:
    static constexpr I NOWHERE = -1;
    static constexpr I LINK = -2;
    class Node {
    public:
      Node() :
        children{NOWHERE, NOWHERE, NOWHERE, NOWHERE},
        nodeCount(0), hash(0) {}
      T nodes[nc];
      // The following fields are unspecified if nodeCount < nc.
      // If nodeCount == LINK, then nw contains the node with additional
      // nodes and the rest of the fields are unspecified.
      // If nodeCount == NOWHERE, then the fields point to the four
      // child quadtrants of this node.
      I children[4];
      I nodeCount; // Set to NOWHERE if not a leaf.
      size_t hash;
    };
    Pool<Node> nodes;
    I root;
    AABB<F> box;
    GetXY gxy;
    I createNode() {
      size_t i = nodes.allocate();
      nodes.get(i).nodeCount = 0;
      return (I) i;
    }
    Handle<I> insertStem(T&& t, glm::tvec2<F>& p, Node& n, AABB<F> box) {
      size_t c = box.getClass(p);
      return insert(std::move(t), p, n.children[c], box.getSubboxByClass(c));
    }
#define n (nodes.get(root))
    // Insert an element in the qtree
    Handle<I> insert(T&& t, glm::tvec2<F>& p, I root, AABB<F> box) {
      if (n.nodeCount == NOWHERE) {
        return insertStem(std::move(t), p, n, box);
      } else if (n.nodeCount == LINK) {
        return insert(std::move(t), p, n.children[0], box);
      } else if (n.nodeCount < nc) {
        n.nodes[n.nodeCount] = std::move(t);
        glm::tvec2<F> ps = gxy.getPos(t);
        n.hash ^= (std::hash<F>{}(ps.x) << 1) ^ std::hash<F>{}(ps.y);
        ++n.nodeCount;
        return { root, (I) (n.nodeCount - 1) };
      } else if (n.hash != 0) {
        // Leaf is full!
        // Split into multiple trees.
        I nw = createNode();
        I ne = createNode();
        I sw = createNode();
        I se = createNode();
        n.children[0] = nw;
        n.children[1] = ne;
        n.children[2] = sw;
        n.children[3] = se;
        n.nodeCount = NOWHERE;
        for (size_t i = 0; i < nc; ++i) {
          T& sub = n.nodes[i];
          glm::tvec2<F> ps = gxy.getPos(sub);
          insertStem(std::move(sub), ps, n, box);
        }
        return insertStem(std::move(t), p, n, box);
      } else {
        // Leaf is full, and chances are:
        // Either all n points are the same, or
        // n/2 points are the same, and n/2 others are too
        // (or other possibilities, but we're worried
        // only about the first case since otherwise
        // it'll cause a stack overflow)
        n.nodeCount = LINK;
        I nw = createNode();
        n.children[0] = nw;
        return insert(std::move(t), p, n.children[0], box);
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
        query(shape, out, n.children[0], box.nw());
        query(shape, out, n.children[1], box.ne());
        query(shape, out, n.children[2], box.sw());
        query(shape, out, n.children[3], box.se());
      } else if (n.nodeCount == LINK) {
        for (I i = 0; i < nc; ++i) {
          if (shape.contains(gxy.getPos(n.nodes[i])))
            out.push_back({root, i});
        }
        query(shape, out, n.children[0], box);
      } else {
        // Leaf
        for (I i = 0; i < n.nodeCount; ++i) {
          if (shape.contains(gxy.getPos(n.nodes[i])))
            out.push_back({root, i});
        }
      }
    }
    template<typename Q = AABB<T>, typename C>
    void query(
        const Q& shape, C callback,
        I root, AABB<F> box) const {
      // Abort if the query shape doesn't intersect the box
      if (!shape.intersects(box)) return;
      const Node& n = nodes.get(root);
      if (n.nodeCount == NOWHERE) {
        // It is a stem
        query(shape, callback, n.children[0], box.nw());
        query(shape, callback, n.children[1], box.ne());
        query(shape, callback, n.children[2], box.sw());
        query(shape, callback, n.children[3], box.se());
      } else if (n.nodeCount == LINK) {
        for (I i = 0; i < nc; ++i) {
          if (shape.contains(gxy.getPos(n.nodes[i])))
            callback(n.nodes[i]);
        }
        query(shape, callback, n.children[0], box);
      } else {
        // Leaf
        for (I i = 0; i < n.nodeCount; ++i) {
          if (shape.contains(gxy.getPos(n.nodes[i])))
            callback(n.nodes[i]);
        }
      }
    }
    template<typename Q = AABB<T>, typename C>
    void querym(
        const Q& shape, C callback,
        I root, AABB<F> box) {
      // Abort if the query shape doesn't intersect the box
      if (!shape.intersects(box)) return;
      Node& n = nodes.get(root);
      if (n.nodeCount == NOWHERE) {
        // It is a stem
        querym(shape, callback, n.children[0], box.nw());
        querym(shape, callback, n.children[1], box.ne());
        querym(shape, callback, n.children[2], box.sw());
        querym(shape, callback, n.children[3], box.se());
      } else if (n.nodeCount == LINK) {
        for (I i = 0; i < nc; ++i) {
          if (shape.contains(gxy.getPos(n.nodes[i])))
            callback(n.nodes[i]);
        }
        querym(shape, callback, n.children[0], box);
      } else {
        // Leaf
        for (I i = 0; i < n.nodeCount; ++i) {
          if (shape.contains(gxy.getPos(n.nodes[i])))
            callback(n.nodes[i]);
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
      const Node* n = &nodes.get(root);
      if (n->nodeCount == NOWHERE) {
        std::cerr << "Stem "; printAABB(box); std::cerr << ":\n";
        for (size_t i = 0; i < 4; ++i) {
          indent(s);
          std::cerr << 
            ((i & 2) != 0 ? 'S' : 'N') <<
            ((i & 1) != 0 ? 'E' : 'W') << ' ';
            dump(n->children[i], box.getSubboxByClass(i), s + 1);
        }
      } else {
        const Node* end = n;
        while (end->nodeCount == LINK) end = &nodes.get(end->children[0]);
        if (end->nodeCount == NOWHERE) {
          std::cerr << "Stem (with overflow nodes) "; printAABB(box); std::cerr << ":";
        } else {
          std::cerr << "Leaf "; printAABB(box); std::cerr << ":";
        }
        while (n->nodeCount == LINK) {
          for (size_t i = 0; i < nc; ++i) {
            glm::tvec2<F> p = gxy.getPos(n->nodes[i]);
            std::cerr << " (" << p.x << ", " << p.y << ")";
          }
          n = &nodes.get(n->children[0]);
        }
        if (n->nodeCount == NOWHERE) {
          std::cerr << '\n';
          for (size_t i = 0; i < 4; ++i) {
            indent(s);
            std::cerr << 
              ((i & 2) != 0 ? 'S' : 'N') <<
              ((i & 1) != 0 ? 'E' : 'W') << ' ';
              dump(n->children[i], box.getSubboxByClass(i), s + 1);
          }
        } else {
          for (size_t i = 0; i < n->nodeCount; ++i) {
            glm::tvec2<F> p = gxy.getPos(n->nodes[i]);
            std::cerr << " (" << p.x << ", " << p.y << ")";
          }
        }
        std::cerr << "\n";
      }
    }
  };
}
#endif