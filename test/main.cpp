#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <set>

#include "zekku/Pool.h"
#include "zekku/QuadTree.h"
#include "zekku/BoxQuadTree.h"

constexpr size_t hc = 65536;

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
    if (val != 35 * i)
      printf("i = %zu: got %zu, expected %zu\n", i, val, 35 * i);
  }
}

struct Pair {
  float x, y;
  bool operator==(const Pair& other) const {
    return x == other.x && y == other.y;
  }
  bool operator<(const Pair& other) const {
    if (x < other.x) return true;
    if (x > other.x) return false;
    return y < other.y;
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
  std::uniform_real_distribution<float> rd(-100.0f, 100.0f);
  std::set<Pair> nearPairs;
  Pair q = {rd(r), rd(r)};
  for (size_t i = 0; i < 10000; ++i) {
    Pair p = {rd(r), rd(r)};
    float dx = p.x - q.x;
    float dy = p.y - q.y;
    if (hypotf(dx, dy) < 20.0f)
      nearPairs.insert(p);
    tree.insert(p);
  }
  zekku::CircleQuery<float> query(glm::tvec2<float>{q.x, q.y}, 20.0f);
  std::vector<zekku::Handle<uint16_t>> handles;
  tree.query(query, handles);
  std::set<Pair> actualNearPairs;
  for (const auto& h : handles) {
    const Pair& p = tree.deref(h);
    actualNearPairs.insert(p);
  }
  if (nearPairs != actualNearPairs) {
    std::set<Pair> nman, anmn;
    std::set_difference(
      nearPairs.begin(), nearPairs.end(),
      actualNearPairs.begin(), actualNearPairs.end(),
      std::inserter(nman, nman.end())
    );
    std::set_difference(
      actualNearPairs.begin(), actualNearPairs.end(),
      nearPairs.begin(), nearPairs.end(),
      std::inserter(anmn, anmn.end())
    );
    std::cerr << "Sets differ:\nnot detected by qtree:\n";
    for (const Pair& p : nman) {
      std::cerr << " (" << p.x << ", " << p.y << ")";
    }
    std::cerr << "\nfalsely detected by qtree:\n";
    for (const Pair& p : anmn) {
      std::cerr << " (" << p.x << ", " << p.y << ")";
    }
    std::cerr << "\nWith the point (" <<
      q.x << ", " << q.y << ")\n";
    std::cerr << "Dumping tree...\n";
    tree.dump();
    handles.clear();
    tree.query(zekku::QueryAll<float>(), handles);
    std::cerr << "Total " << handles.size() << " elements\n";
  } else {
    std::cerr << "Sets are equal :)\n";
  }
  std::cerr << "Testing performance...\n";
  using namespace std::chrono;
  auto ms = duration_cast<milliseconds>(
    system_clock::now().time_since_epoch()
  );
  size_t ints = 0;
  constexpr size_t iters = 100000;
  for (size_t i = 0; i < iters; ++i) {
    float x = rd(r);
    float y = rd(r);
    std::vector<zekku::Handle<uint16_t>> handles;
    zekku::CircleQuery<float> query(glm::tvec2<float>{x, y}, 20.0f);
    tree.query(query, handles);
    ints += handles.size();
  }
  auto ms2 = duration_cast<milliseconds>(
    system_clock::now().time_since_epoch()
  );
  auto elapsed = ms2 - ms;
  fprintf(stderr,
    "Done! %zu intersections over %zu iterations taking %zu ms.\n",
    ints, iters, elapsed.count());
}

constexpr size_t NPOINT_PATHO = 50;
void testQTreePathological() {
  std::cerr << "Testing nasty cases...\n";
  zekku::QuadTree<Pair> tree({{0.0f, 0.0f}, {100.0f, 100.0f}});
  for (size_t i = 0; i < NPOINT_PATHO; ++i) {
    tree.insert({1.0f, 0.5f});
  }
  std::cerr << "No crash!\n";
  std::vector<zekku::Handle<uint16_t>> handles;
  tree.query(zekku::QueryAll<float>(), handles);
  if (handles.size() != NPOINT_PATHO) {
    fprintf(stderr,
      "Querying returned %zu handles (%zu expected).\n",
      handles.size(),
      NPOINT_PATHO
    );
    tree.dump();
  } else {
    std::cerr << "Querying went fine!\n";
  }
}

void testBBQTree() {
  std::cerr << "Testing bounding box quadtree...\n";
  zekku::BoxQuadTree<
    zekku::AABB<float>,
    /* I = */ uint16_t,
    /* F = */ float,
    /* nc = */ zekku::QUADTREE_NODE_COUNT,
    /* GetBB = */ zekku::AABBGetBB<float>
  > tree({{0.0f, 0.0f}, {100.0f, 100.0f}});
  std::mt19937_64 r; // Ugh, C++ random number generation is a PITA.
  r.seed(time(nullptr));
  std::uniform_real_distribution<float> rd(-1.0f, 1.0f);
  std::vector<zekku::AABB<float>> boxes(10000);
  for (auto& box : boxes) {
    box.c = { 50 * rd(r), 50 * rd(r) };
    box.s = { 2.5 + 2.5 * rd(r), 2.5 + 2.5 * rd(r) };
  }
  for (const auto& box : boxes) {
    tree.insert(box);
  }
}

int main() {
  printf("Testing...\n");
  testPool();
  testQTree();
  testQTreePathological();
  testBBQTree(); // Mmm
  return 0;
}