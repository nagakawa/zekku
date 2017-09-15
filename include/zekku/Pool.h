#pragma once

#ifndef ZEKKU_POOL_H
#define ZEKKU_POOL_H
#include <stddef.h>
#include <string.h>
#include <time.h>
#include <algorithm>
#include <random>

namespace zekku {
  constexpr size_t START_CAPAT = 64;
  template<typename T>
  class Pool {
  public:
    Pool() : filled(0), capacity(START_CAPAT),
        elems(new T[START_CAPAT]), allocated(new bool[START_CAPAT]) {
      r.seed(time(nullptr));
      memset(allocated, 0, START_CAPAT * sizeof(bool));
    }
    ~Pool() { delete[] elems; delete[] allocated; }
    Pool(const Pool& other) = delete;
    Pool& operator=(const Pool& other) = delete;
    Pool& operator=(Pool&& other) {
      std::swap(filled, other.filled);
      std::swap(capacity, other.capacity);
      std::swap(elems, other.elems);
      std::swap(allocated, other.allocated);
      return *this;
    }
    T& get(size_t handle) { return elems[handle]; }
    size_t allocate() {
      if (shouldExpand()) expand();
      size_t bucket = r() % capacity;
      while (allocated[bucket]) {
        ++bucket;
        if (bucket >= capacity) bucket -= capacity;
      }
      ++filled;
      allocated[bucket] = true;
      return bucket;
    }
    void deallocate(size_t handle) {
      allocated[handle] = false;
      --filled;
    }
    bool isValid(size_t handle) { return allocated[handle]; }
  private:
    bool shouldExpand() {
      return filled * 4 >= capacity * 3;
    }
    void expand() {
      T* newElems = new T[capacity << 1];
      bool* newAllocated = new bool[capacity << 1];
      for (size_t i = 0; i < capacity; ++i) {
        newElems[i] = elems[i];
      }
      memcpy(newAllocated, allocated, capacity * sizeof(bool));
      memset(newAllocated + capacity, 0, capacity * sizeof(bool));
      delete[] elems;
      elems = newElems;
      delete[] allocated;
      allocated = newAllocated;
      capacity <<= 1;
    }
    size_t filled;
    size_t capacity;
    T* elems;
    bool* allocated;
    std::minstd_rand r;
  };
}
#endif