#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <set>

#include "zekku/bitwise.h"
#include "zekku/Pool.h"
#include "zekku/QuadTree.h"
#include "zekku/BoxQuadTree.h"

template<typename F>
std::ostream& operator<<(std::ostream& fh, const zekku::AABB<F>& box) {
  std::cerr << "[" << box.c[0] - box.s[0] << ", " << box.c[1] - box.s[1] <<
    "; " <<  box.c[0] + box.s[0] << ", " << box.c[1] + box.s[1] << "]";
  return fh;
}

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

struct TestEntry {
  zekku::AABB<float> box;
  glm::vec2 velocity;
};

void testBBQTree() {
  std::cerr << "Testing bounding box quadtree...\n";
  zekku::BoxQuadTree<TestEntry, uint32_t>
    tree({{0.0f, 0.0f}, {100.0f, 100.0f}});
  std::mt19937_64 r; // Ugh, C++ random number generation is a PITA.
  r.seed(time(nullptr));
  std::uniform_real_distribution<float> rd(-1.0f, 1.0f);
  std::vector<TestEntry> entries(10000);
  for (auto& entry : entries) {
    entry.box.c = { 50 * rd(r), 50 * rd(r) };
    entry.box.s = { 2.5 + 2.5 * rd(r), 2.5 + 2.5 * rd(r) };
    float s = 0.75f + 0.25f * rd(r);
    float a = M_PI * rd(r);
    entry.velocity = { s * cosf(a), s * sinf(a) };
  }
  Pair q = {50 * rd(r), 50 * rd(r)};
  zekku::CircleQuery<float> query(glm::tvec2<float>{q.x, q.y}, 20.0f);
  std::set<zekku::AABB<float>> nearPairs;
  for (const auto& entry : entries) {
    tree.insert(entry);
    if (query.intersects(entry.box))
      nearPairs.insert(entry.box);
  }
  std::vector<zekku::BBHandle> handles;
  tree.query(query, handles);
  std::set<zekku::AABB<float>> actualNearPairs;
  for (const auto& h : handles) {
    const zekku::AABB<float>& p = tree.deref(h).box;
    actualNearPairs.insert(p);
  }
  if (nearPairs != actualNearPairs) {
    std::set<zekku::AABB<float>> nman, anmn;
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
    std::cerr << "Sets differ:\nnot detected by bqtree:\n";
    for (const zekku::AABB<float>& p : nman) {
      std::cerr << p << " ";
    }
    std::cerr << "\nfalsely detected by bqtree:\n";
    for (const zekku::AABB<float>& p : anmn) {
      std::cerr << p << " ";
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
  std::uniform_real_distribution<float> rd2(-100.0f, 100.0f);
  std::cerr << "Testing performance...\n";
  using namespace std::chrono;
  auto ms = duration_cast<milliseconds>(
    system_clock::now().time_since_epoch()
  );
  size_t ints = 0;
  constexpr size_t iters = 100000;
  for (size_t i = 0; i < iters; ++i) {
    float x = rd2(r);
    float y = rd2(r);
    std::vector<zekku::BBHandle> handles;
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
  ms = duration_cast<milliseconds>(
    system_clock::now().time_since_epoch()
  );
  ints = 0;
  for (size_t i = 0; i < iters; ++i) {
    float x = rd2(r);
    float y = rd2(r);
    zekku::CircleQuery<float> query(glm::tvec2<float>{x, y}, 20.0f);
    for (const auto& e : entries) {
      if (query.intersects(e.box)) ++ints;
    }
  }
  ms2 = duration_cast<milliseconds>(
    system_clock::now().time_since_epoch()
  );
  elapsed = ms2 - ms;
  fprintf(stderr,
    "(by comparison: %zu intersections by brute force\n"
    "  over %zu iterations taking %zu ms)\n",
    ints, iters, elapsed.count());
  // Test performance of apply
  auto callback = [](TestEntry& e) {
    glm::vec2 newPos = e.box.c + e.velocity;
    if (newPos.x > 50) e.velocity.x = -fabs(e.velocity.x);
    if (newPos.x < -50) e.velocity.x = fabs(e.velocity.x);
    if (newPos.y > 50) e.velocity.y = -fabs(e.velocity.y);
    if (newPos.y < -50) e.velocity.y = fabs(e.velocity.y);
    e.box.c = newPos;
  };
  constexpr size_t updateIters = 1000;
  ms = duration_cast<milliseconds>(
    system_clock::now().time_since_epoch()
  );
  for (size_t i = 0; i < updateIters; ++i) {
    tree.apply(callback);
  }
  ms2 = duration_cast<milliseconds>(
    system_clock::now().time_since_epoch()
  );
  elapsed = ms2 - ms;
  fprintf(stderr,
    "Done! %zu apply() calls taking %zu ms.\n",
    updateIters, elapsed.count());
  ms = duration_cast<milliseconds>(
    system_clock::now().time_since_epoch()
  );
  for (size_t i = 0; i < updateIters; ++i) {
    for (auto& e : entries)
      callback(e);
  }
  ms2 = duration_cast<milliseconds>(
    system_clock::now().time_since_epoch()
  );
  elapsed = ms2 - ms;
  fprintf(stderr,
    "(by comparison, %zu updates to each element of a vector take %zu ms)\n",
    updateIters, elapsed.count());
}

int main() {
  printf("Testing...\n");
  testPool();
  testQTree();
  testQTreePathological();
  testBBQTree(); // Mmm
  return 0;
}