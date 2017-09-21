#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <random>
#include <unordered_set>

#include "zekku/Pool.h"
#include "zekku/QuadTree.h"

constexpr size_t hc = 64;

void testPool() {
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
}

struct Pair {
  float x, y;
  bool operator==(const Pair& other) const {
    return x == other.x && y == other.y;
  }
};

struct PairHash {
  size_t operator()(const Pair& p) const {
    return (std::hash<float>{}(p.x) << 1) ^ std::hash<float>{}(p.y);
  }
};

void testQTree() {
  zekku::QuadTree<Pair> tree({{0.0f, 0.0f}, {100.0f, 100.0f}});
  std::mt19937_64 r;
  r.seed(time(nullptr));
  std::uniform_real_distribution<float> rd(-50.0f, 50.0f);
  std::unordered_set<Pair, PairHash> nearPairs;
  Pair q = {rd(r), rd(r)};
  for (size_t i = 0; i < 1000; ++i) {
    Pair p = {rd(r), rd(r)};
    float dx = p.x - q.x;
    float dy = p.y - q.y;
    if (sqrtf(dx * dx + dy * dy) < 20.0f)
      nearPairs.insert(p);
    tree.insert(p);
  }
  zekku::CircleQuery query(zekku::Vec2<float>{q.x, q.y}, 20.0f);
  std::vector<zekku::Handle<uint16_t>> handles;
  tree.query(query, handles);
  std::unordered_set<Pair, PairHash> actualNearPairs;
  for (const auto& h : handles) {
    const Pair& p = tree.deref(h);
    actualNearPairs.insert(p);
  }
  if (nearPairs != actualNearPairs) {
    std::cerr << "Sets differ:\nnearPairs:";
    for (const Pair& p : nearPairs) {
      std::cerr << " (" << p.x << ", " << p.y << ")";
    }
    std::cerr << "\nactualNearPairs:";
    for (const Pair& p : actualNearPairs) {
      std::cerr << " (" << p.x << ", " << p.y << ")";
    }
  } else {
    std::cerr << "Sets are equal :)\n";
  }
}

int main() {
  printf("Testing...\n");
  testPool();
  testQTree();
  return 0;
}