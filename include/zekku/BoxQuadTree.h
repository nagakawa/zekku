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
#include "zekku/bitwise.h"
#include "zekku/BloomFilter.h"
#include "zekku/base.h"

namespace zekku {
  template<typename T, typename F = float>
  struct DefaultGetBB {
    static_assert(std::is_floating_point<F>::value,
      "Your F is not a floating-point number, dum dum!");
    const auto& operator()(const T& t) const { return t.box; }
  };
  // Yes, this sounds pretty silly.
  template<typename F = float>
  struct AABBGetBB {
    static_assert(std::is_floating_point<F>::value,
      "Your F is not a floating-point number, dum dum!");
    const AABB<F>& operator()(const AABB<F>& t) const { return t; }
  };
  template<typename F>
  struct BBHash {
    size_t operator()(const AABB<F>& box) const {
      // XXX: this hash function is just good enough for BoxQuadTree.
      // It's not a very good general hashing function.
      return
        std::hash<F>()(box.c.x + box.c.y - box.s.x - box.s.y);
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
    bool operator>(const BBHandle& other) const {
      return other < *this;
    }
  };
  struct BBHandleHasher {
    size_t operator()(const BBHandle& h) {
      return h.index;
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
    typename B = AABB<F>,
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
    // using BF = BloomFilter<BBHandle, BBHandleHasher, 1>;
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
      B p = gbox(t);
      if (!p.isWithin(box)) {
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
      query(shape, out, root, box);
      // No dedupe needed anymore
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
        B p = gbox(t);
        insert(t, it.i, p, root, box);
      }
    }
    void dump() const {
      dump(root, box);
    }
  private:
    class Node {
    public:
      Node() :
        hash(0), nodeCount(0), link(false), stem(false) {}
      // The following fields are unspecified if neither stem nor link is set.
      // If link is set, then children[0] contains the node with 
      // additional nodes and the rest of the fields are unspecified.
      // If stem is set, then the fields point to the four
      // child quadtrants of this node.
      // If link is not set, then nodeCount stores
      // the number of nodes stored in the `nodes` array.
      // stem and link cannot be set at the same time.
      // (This implies that if you encounter a link node during insertion,
      // you have to slog through it [and possibly even more link's]
      // before you can see the children of the node, but this
      // simplifies the logic.)
      size_t hash;
      I children[4];
      I nodeCount;
      bool link, stem;
      uint32_t nodes[nc]; // Indices to `canonicals`
    };
    Pool<Node> nodes;
    Pool<T> canonicals;
    I root;
    AABB<F> box;
    ZK_NOUNIQADDR GetBB gbox;
    void clearTree() {
      // Clears the tree structure, but not the elements themselves.
      size_t oc = nodes.getCapacity();
      nodes = Pool<Node>(oc);
      root = createNode();
    }
    I createNode() {
      size_t i = nodes.allocate();
      return (I) i;
    }
#define isNowhere (np->stem)
#define isLink    (np->link)
#define numNodes  (np->nodeCount)
    BBHandle insertStem(
        const T& t, uint32_t ti, const B& p,
        size_t root,
        const AABB<F>& box) {
      Node* np = &nodes.get(root);
      // Find out which subboxes this object intersects
      unsigned count = 0;
      unsigned index = 0;
      if (p.intersects(box.nw())) {
        ++count;
        index = 0;
      }
      if (p.intersects(box.ne())) {
        ++count;
        index = 1;
      }
      if (p.intersects(box.sw())) {
        ++count;
        index = 2;
      }
      if (p.intersects(box.se())) {
        ++count;
        index = 3;
      }
      // By now, at least one element of intersect *should* be true,
      // but rounding errors can result in p intersecting with box
      // but not with any of its subboxes.
      // Intersects two or more quadrants?
      if (count >= 2) {
        return insert(t, ti, p, root, box, true);
      }
      // Otherwise...
      if (index == 0)
        insert(t, ti, p, np->children[0], box.nw());
      if (index == 1)
        insert(t, ti, p, np->children[1], box.ne());
      if (index == 2)
        insert(t, ti, p, np->children[2], box.sw());
      if (index == 3)
        insert(t, ti, p, np->children[3], box.se());
      // We can just return ti
      // since that's the index into the `canonicals` array
      return { ti };
    }
    // Insert an element in the qtree.
    // If forceHere is true, then the node will be created on this
    // node and nowhere else, possibly creating a link node.
    BBHandle insert(
        const T& t, uint32_t ti, const B& p,
        I root,
        AABB<F> box,
        bool forceHere = false) {
      Node* np = &nodes.get(root);
      while (isLink) {
        root = np->children[0];
        np = &nodes.get(root);
      }
      if (isNowhere && !forceHere) {
        return insertStem(t, ti, p, root, box);
      }
      if (numNodes < nc) {
        np->nodes[numNodes] = ti;
        B bb = gbox(t);
        np->hash ^= BBHash<F>()(bb);
        ++np->nodeCount;
        return { ti };
      } else if (np->hash != 0 && !isLink && !forceHere) {
        // Leaf is full!
        // Split into multiple trees.
        I nw = createNode();
        I ne = createNode();
        I sw = createNode();
        I se = createNode();
        np = &nodes.get(root);
        np->children[0] = nw;
        np->children[1] = ne;
        np->children[2] = sw;
        np->children[3] = se;
        np->stem = true;
        return insertStem(t, ti, p, root, box);
      } else {
        // Leaf is full, and chances are:
        // Either all n points are the same, or
        // we have a false positive of the above,
        // or forceHere is true
        // (in which case isNowhere might be true as well)
        I nw = createNode(); // Create a node for overflow
        np = &nodes.get(root);
        Node& nwNode = nodes.get(nw);
        // Transfer children from *np to nw (if any)
        if (isNowhere) {
          nwNode.stem = true;
          memcpy(nwNode.children, np->children, 4 * sizeof(I));
        }
        np->children[0] = nw;
        np->link = true;
        np->stem = false;
        return insert(t, ti, p, np->children[0], box);
      }
    }
#undef isNowhere
#undef isLink
#undef numNodes
    template<typename Q = AABB<T>>
    void query(
        const Q& shape, std::vector<BBHandle>& out,
        I root, AABB<F> box) const {
      // Abort if the query shape doesn't intersect the box
      if (!shape.intersects(box)) return;
      const Node* np = &(nodes.get(root));
      while (np->link) {
        for (I i = 0; i < nc; ++i) {
          uint32_t ni = np->nodes[i];
          const T& n = canonicals.get(ni);
          if (shape.intersects(gbox(n)))
            out.push_back({ ni });
        }
        np = &(nodes.get(np->children[0]));
      }
      if (np->stem) {
        // Stem (and possibly a leaf)
        glm::tvec2<F> halfs = box.s * 0.5f;
        query(shape, out, np->children[0],
          AABB<F>{box.c - halfs, halfs});
        query(shape, out, np->children[1],
          AABB<F>{{box.c.x + halfs.x, box.c.y - halfs.y}, halfs});
        query(shape, out, np->children[2],
          AABB<F>{{box.c.x - halfs.x, box.c.y + halfs.y}, halfs});
        query(shape, out, np->children[3],
          AABB<F>{box.c + halfs, halfs});
      }
      for (I i = 0; i < np->nodeCount; ++i) {
        uint32_t ni = np->nodes[i];
        const T& n = canonicals.get(ni);
        if (shape.intersects(gbox(n)))
          out.push_back({ ni });
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
      while (end->link) end = &nodes.get(end->children[0]);
      if (end->stem) {
        std::cerr << "Stem (with overflow nodes) "; printAABB(box); std::cerr << ": ";
      } else {
        std::cerr << "Leaf "; printAABB(box); std::cerr << ": ";
      }
      while (n->link) {
        for (size_t i = 0; i < nc; ++i) {
          AABB<F> p = gbox(canonicals.get(n->nodes[i]));
          printAABB(p);
        }
        n = &nodes.get(n->children[0]);
      }
      if (n->stem) {
        std::cerr << '\n';
        for (size_t i = 0; i < 4; ++i) {
          indent(s);
          std::cerr << 
            ((i & 2) != 0 ? 'S' : 'N') <<
            ((i & 1) != 0 ? 'E' : 'W') << ' ';
            dump(n->children[i], box.getSubboxByClass(i), s + 1);
        }
      }
      if (!n->stem || n->nodeCount != 0) {
        for (size_t i = 0; i < n->nodeCount; ++i) {
          AABB<F> p = gbox(canonicals.get(n->nodes[i]));
          printAABB(p);
        }
      }
      std::cerr << "\n";
      indent(s);
    }
    static void sortHandles(std::vector<BBHandle>& handles) {
      constexpr uint32_t bitsPerIter = 8;
      constexpr uint32_t nBuckets = 1 << bitsPerIter;
      size_t nHandles = handles.size();
      if (nHandles == 0) return;
      BBHandle biggestHandle = *std::max_element(handles.begin(), handles.end());
      uint32_t iterations =
        (log2up(biggestHandle.index) + bitsPerIter - 1) / bitsPerIter;
      BBHandle* handlesAlt = new BBHandle[nHandles];
      BBHandle* curr = handles.data();
      BBHandle* next = handlesAlt;
      // Use LSD radix sort, handling `bitsPerIter` bits at a time
      for (uint32_t i = 0; i < iterations; ++i) {
        uint32_t counts[1 + nBuckets] = {0};
        for (size_t j = 0; j < nHandles; ++j) {
          uint32_t digit = (curr[j].index >> (bitsPerIter * i)) & (nBuckets - 1);
          ++counts[digit + 1];
        }
        for (uint32_t j = 0; j < nBuckets; ++j) {
          counts[j + 1] += counts[j];
        }
        // Now counts[i] = Sigma_(k = 0)^(i - 1) (# entries in bucket k)
        // In other words, this is the index where the handles
        // in bucket i shall start
        for (size_t j = 0; j < nHandles; ++j) {
          uint32_t digit = (curr[j].index >> (bitsPerIter * i)) & (nBuckets - 1);
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
      if (handles.data() != curr) {
        memcpy(handles.data(), curr, sizeof(BBHandle) * nHandles);
      }
      // DEBUG: sanity check
      /**/
      delete[] handlesAlt;
    }
  };
}

#endif