#pragma once

#ifndef ZEKKU_BOX_QUADTREE_H
#define ZEKKU_BOX_QUADTREE_H
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <algorithm>
#include <bitset>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <type_traits>
#include <glm/glm.hpp>
#include "zekku/Pool.h"
#include "zekku/QuadTree.h"

namespace zekku {
  template<typename T, typename F = float>
  struct DefaultGetBB {
    static_assert(std::is_floating_point<F>::value,
      "Your F is not a floating-point number, dum dum!");
    AABB<F> getBox(T t) const { return t.box; }
  };
  // Yes, this sounds pretty silly.
  template<typename F = float>
  struct AABBGetBB {
    static_assert(std::is_floating_point<F>::value,
      "Your F is not a floating-point number, dum dum!");
    AABB<F> getBox(AABB<F> t) const { return t; }
  };
  template<typename F>
  struct BBHash {
    size_t operator()(const AABB<F>& box) const {
      return
        (std::hash<F>()(box.c.x) << 3) ^
        (std::hash<F>()(box.c.y) << 2) ^
        (std::hash<F>()(box.s.x) << 1) ^
        (std::hash<F>()(box.s.y));
    }
  };
  struct BBHandle {
    uint32_t index;
    bool operator==(const BBHandle& other) const {
      return index == other.index;
    }
    bool operator<(const BBHandle& other) const {
      return index < other.index;
    }
  };
  struct BBHandleHasher {
    size_t operator()(const BBHandle& h) {
      return std::hash<size_t>()(h.index);
    }
  };
  /*
    Bounding box-based quadtree based on the existing point quadtree
    implementation and a Python implementation of a BB quadtree here:
    https://www.pygame.org/wiki/QuadTree
  */
  template<
    typename T,
    typename I = uint16_t,
    typename F = float,
    size_t nc = QUADTREE_NODE_COUNT,
    typename GetBB = DefaultGetBB<T, F>
  >
  class BoxQuadTree {
  public:
    static_assert(std::is_integral<I>::value,
      "Your I is not an integer, dum dum!");
    static_assert(std::is_unsigned<I>::value,
      "Don't use a signed int for sizes, dum dum!");
    static_assert(std::is_floating_point<F>::value,
      "Your F is not a floating-point number, dum dum!");
    template<typename... Args>
    BoxQuadTree(const AABB<F>& box, Args&&... args) :
        root((I) nodes.allocate()), box(box), gbox(args...) {}
    BoxQuadTree(QuadTree<T, I, F, nc, GetBB>&& other) :
        nodes(std::move(other.nodes)), root(other.root),
        box(other.box), gbox(other.gbox) {
      other.root = (I) other.nodes.allocate();
    }
    BBHandle insert(const T& t) {
      T t2 = t;
      return insert(std::move(t2));
    }
    BBHandle insert(T&& t) {
      AABB<F> p = gbox.getBox(t);
      if (!box.contains(p)) {
        std::cerr << "(" << p.c.x << ", " << p.c.y << ") +/- (";
        std::cerr << p.s.x << ", " << p.s.y;
        std::cerr << ") is out of range!\n";
        std::cerr << "Box is centred at (" << box.c[0] << ", " << box.c[1] << ") ";
        std::cerr << "with w = " << box.s[0] << " and h = " << box.s[1] << "\n";
        exit(-1);
      }
      uint32_t ti = (uint32_t) canonicals.allocate(std::move(t));
      BBHandle h = insert(canonicals.get(ti), ti, p, root, box);
      assert(nodes.getCapacity() <= std::numeric_limits<I>::max());
      return h;
    }
    const T& deref(const BBHandle& h) const {
      return canonicals.get(h.index);
    }
    T& deref(const BBHandle& h) {
      return canonicals.get(h.index);
    }
    template<typename Q = AABB<T>>
    void query(const Q& shape, std::vector<BBHandle>& out) const {
      auto callback = [&out](uint32_t ti, const T& /*t*/) {
        out.push_back({ ti });
      };
      query(shape, callback, root, box);
      sortHandles(out);
      auto it = std::unique(out.begin(), out.end());
      out.erase(it, out.end());
    }
    template<typename Q = AABB<T>, typename C>
    void query(const Q& shape, C callback) const {
      std::vector<BBHandle> out;
      query(shape, out);
      for (BBHandle h : out) {
        callback(canonicals.get(h.index));
      }
    }
    template<typename Q = AABB<T>, typename C>
    void querym(const Q& shape, C callback) {
      std::vector<BBHandle> out;
      query(shape, out);
      for (BBHandle h : out) {
        callback(std::move(canonicals.get(h.index)));
      }
    }
    template<typename C>
    void apply(const C& f) {
      // Apply f to each element and rebuild the tree.
      clearTree();
      for (auto it = canonicals.begin(); it != canonicals.end(); ++it) {
        T& t = *it;
        f(t);
        AABB<F> p = gbox.getBox(t);
        insert(t, it.i, p, root, box);
      }
    }
    void dump() const {
      dump(root, box);
    }
  private:
    static constexpr I MASK = 0x3FFF;
    static constexpr I NOWHERE = 0x4000;
    static constexpr I LINK = 0x8000;
    class Node {
    public:
      Node() :
        children{NOWHERE, NOWHERE, NOWHERE, NOWHERE},
        nodeCount(0), hash(0) {}
      size_t nodes[nc]; // Indices to `canonicals`
      // If there are duplicates of an object in a quadtree, only
      // one node in the tree will have the corresponding `echo[i]`
      // be false. The rest will have the element set to `true` to
      // avoid double-counting in the query methods.
      std::bitset<nc> echo;
      // The following fields are unspecified if nodeCount < nc.
      // If (nodeCount & LINK) != 0, then children[0] contains the node with 
      // additional nodes and the rest of the fields are unspecified.
      // If (nodeCount & NOWHERE) != 0, then the fields point to the four
      // child quadtrants of this node.
      // If (nodeCount & LINK) == 0, then (nodeCount & MASK) stores
      // the number of nodes stored in the `nodes` array.
      // NOWHERE and LINK cannot be set at the same time.
      // (This implies that if you encounter a LINK node during insertion,
      // you have to slog through it [and possibly even more LINKs]
      // before you can see the children of the node, but this
      // simplifies the logic.)
      I children[4];
      I nodeCount; // Set to NOWHERE if not a leaf.
      size_t hash;
    };
    Pool<Node> nodes;
    Pool<T> canonicals;
    I root;
    AABB<F> box;
    GetBB gbox;
    void clearTree() {
      // Clears the tree structure, but not the elements themselves.
      nodes = Pool<Node>();
      root = createNode();
    }
    I createNode() {
      size_t i = nodes.allocate();
      nodes.get(i).nodeCount = 0;
      return (I) i;
    }
#define n (nodes.get(root))
#define isNowhere ((n.nodeCount & NOWHERE) != 0)
#define isLink    ((n.nodeCount & LINK) != 0)
#define numNodes (n.nodeCount & MASK)
    BBHandle insertStem(
        const T& t, uint32_t ti, const AABB<F>& p,
        size_t root,
        AABB<F> box,
        bool echo) {
      // Find out which subboxes this object intersects
      assert(box.intersects(p));
      bool intersect[4];
      for (size_t i = 0; i < 4; ++i) {
        intersect[i] = box.getSubboxByClass(i).intersects(p);
      }
      // Intersects every quadrant?
      if (intersect[0] && intersect[1] && intersect[2] && intersect[3]) {
        return insert(t, ti, p, root, box, echo, true);
      }
      // Otherwise...
      bool intersected = false;
      for (size_t i = 0; i < 4; ++i) {
        if (intersect[i]) {
          insert(
            t, ti, p,
            n.children[i],
            box.getSubboxByClass(i),
            echo || intersected);
          intersected = true;
        }
      }
      // By now, intersected *should* be true, but rounding errors
      // can result in p intersecting with box but not with any of its
      // subboxes.
      // We can just return ti
      // since that's the index into the `canonicals` array
      return { ti };
    }
    // Insert an element in the qtree.
    // If forceHere is true, then the node will be created on this
    // node and nowhere else, possibly creating a link node.
    BBHandle insert(
        const T& t, uint32_t ti, const AABB<F>& p,
        I root,
        AABB<F> box,
        bool echo = false,
        bool forceHere = false) {
      while (isLink) root = n.children[0];
      if (isNowhere && !forceHere) {
        assert(!isLink);
        return insertStem(t, ti, p, root, box, echo);
      }
      if (numNodes < nc) {
        n.nodes[numNodes] = ti;
        AABB<F> bb = gbox.getBox(t);
        n.hash ^= BBHash<F>()(bb);
        n.echo[numNodes] = echo;
        ++n.nodeCount;
        return { ti };
      } else if (n.hash != 0 && !isLink && !forceHere) {
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
          size_t subi = n.nodes[i];
          const T& sub = canonicals.get(subi);
          AABB<F> ps = gbox.getBox(sub);
          insertStem(sub, subi, ps, root, box, echo);
        }
        return insertStem(t, ti, p, root, box, echo);
      } else {
        // Leaf is full, and chances are:
        // Either all n points are the same, or
        // we have a false positive of the above,
        // or forceHere is true
        // (in which case isNowhere might be true as well)
        I nw = createNode(); // Create a node for overflow
        Node& nwNode = nodes.get(nw);
        // Transfer children from n to nw (if any)
        if (isNowhere) {
          nwNode.nodeCount = NOWHERE;
          memcpy(nwNode.children, n.children, 4 * sizeof(I));
        } else {
          nwNode.nodeCount = 0;
        }
        n.children[0] = nw;
        n.nodeCount = LINK;
        return insert(t, ti, p, n.children[0], box, echo);
      }
    }
#undef n
#undef isNowhere
#undef isLink
#undef numNodes
    template<typename Q = AABB<T>, typename C>
    void query(
        const Q& shape, const C& callback,
        I root, AABB<F> box) const {
      // Abort if the query shape doesn't intersect the box
      if (!shape.intersects(box)) return;
      const Node* np = &(nodes.get(root));
      while ((np->nodeCount & LINK) != 0) {
        for (I i = 0; i < nc; ++i) {
          size_t ni = np->nodes[i];
          const T& n = canonicals.get(ni);
          if (shape.intersects(gbox.getBox(n)))
            callback(ni, n);
        }
        np = &(nodes.get(np->children[0]));
      }
      if ((np->nodeCount & NOWHERE) != 0) {
        // Stem (and possibly a leaf)
        query(shape, callback, np->children[0], box.nw());
        query(shape, callback, np->children[1], box.ne());
        query(shape, callback, np->children[2], box.sw());
        query(shape, callback, np->children[3], box.se());
      }
      for (I i = 0; i < (np->nodeCount & MASK); ++i) {
        size_t ni = np->nodes[i];
        const T& n = canonicals.get(ni);
        if (shape.intersects(gbox.getBox(n)))
          callback(ni, n);
      }
    }
    // Stuff for dumping
    static void indent(size_t n) {
      for (size_t i = 0; i < n; ++i) std::cerr << ' ';
    }
    static void printAABB(const AABB<F>& box) {
      std::cerr << "[" << box.c[0] - box.s[0] << ", " << box.c[1] - box.s[1] <<
        "; " <<  box.c[0] + box.s[0] << ", " << box.c[1] + box.s[1] << "] ";
    }
    void dump(I root, AABB<F> box, size_t s = 0) const {
      const Node* n = &nodes.get(root);
      const Node* end = n;
      while ((end->nodeCount & LINK) != 0) end = &nodes.get(end->children[0]);
      if ((end->nodeCount & NOWHERE) != 0) {
        std::cerr << "Stem (with overflow nodes) "; printAABB(box); std::cerr << ": ";
      } else {
        std::cerr << "Leaf "; printAABB(box); std::cerr << ": ";
      }
      while ((n->nodeCount & LINK) != 0) {
        for (size_t i = 0; i < nc; ++i) {
          AABB<F> p = gbox.getBox(canonicals.get(n->nodes[i]));
          printAABB(p);
          if (n->echo[i]) std::cerr << "(echo) ";
        }
        n = &nodes.get(n->children[0]);
      }
      if ((n->nodeCount & NOWHERE) != 0) {
        std::cerr << '\n';
        for (size_t i = 0; i < 4; ++i) {
          indent(s);
          std::cerr << 
            ((i & 2) != 0 ? 'S' : 'N') <<
            ((i & 1) != 0 ? 'E' : 'W') << ' ';
            dump(n->children[i], box.getSubboxByClass(i), s + 1);
        }
      }
      if (n->nodeCount != NOWHERE) {
        for (size_t i = 0; i < (n->nodeCount & MASK); ++i) {
          AABB<F> p = gbox.getBox(canonicals.get(n->nodes[i]));
          printAABB(p);
          if (n->echo[i]) std::cerr << "(echo) ";
        }
      }
      std::cerr << "\n";
      indent(s);
    }
    static void sortHandles(std::vector<BBHandle>& handles) {
      constexpr size_t bitsPerIter = 8;
      constexpr size_t iterations =
        (sizeof(uint32_t) * CHAR_BIT + bitsPerIter - 1) / bitsPerIter;
      constexpr size_t nBuckets = 1 << bitsPerIter;
      size_t nHandles = handles.size();
      if (nHandles == 0) return;
      BBHandle* handlesAlt = new BBHandle[nHandles];
      BBHandle* curr = handles.data();
      BBHandle* next = handlesAlt;
      // Use LSD radix sort, handling 4 bits at a time
      for (size_t i = 0; i < iterations; ++i) {
        size_t counts[1 + nBuckets] = {0};
        for (size_t j = 0; j < nHandles; ++j) {
          size_t digit = (curr[j].index >> (bitsPerIter * i)) & (nBuckets - 1);
          ++counts[digit + 1];
        }
        for (size_t j = 0; j < nBuckets; ++j) {
          counts[j + 1] += counts[j];
        }
        // Now counts[i] = Sigma_(k = 0)^(i - 1) (# entries in bucket k)
        // In other words, this is the index where the handles
        // in bucket i shall start
        for (size_t j = 0; j < nHandles; ++j) {
          size_t digit = (curr[j].index >> (bitsPerIter * i)) & (nBuckets - 1);
          next[counts[digit]] = curr[j];
          ++counts[digit];
        }
        /*for (size_t j = 0; j < nHandles - 1; ++j) {
          size_t digit = (next[j].index >> (bitsPerIter * i)) & (nBuckets - 1);
          std::cerr << next[j].index << "(" << digit << ") ";
        }
        std::cerr << "\n";*/
        std::swap(curr, next);
      }
      if (nHandles % 2 != 0) {
        memcpy(curr, next, sizeof(BBHandle) * nHandles);
      }
      // DEBUG: sanity check
      /*for (size_t j = 0; j < nHandles - 1; ++j) {
        assert(curr[j].index <= curr[j + 1].index);
      }*/
      delete[] next;
    }
  };
}

#endif