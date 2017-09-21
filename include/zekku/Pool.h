#pragma once

#ifndef ZEKKU_POOL_H
#define ZEKKU_POOL_H
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <algorithm>
#include <random>
#include <type_traits>

namespace zekku {
  template<typename T>
  T* tmalloc(size_t elems) {
    return (T*) ::malloc(elems * sizeof(T));
  }
  template<typename T>
  T* trealloc(T* p, size_t elems) {
    return (T*) ::realloc(p, elems * sizeof(T));
  }
  constexpr size_t START_CAPAT = 64;
  template<typename T>
  class Pool {
  public:
    static_assert(std::is_trivially_copyable<T>::value,
      "Your T is not trivially copyable, dum dum!");
    Pool() : filled(0), capacity(START_CAPAT),
        elems(tmalloc<T>(START_CAPAT)),
        allocated(tmalloc<bool>(START_CAPAT)) {
      r.seed(time(nullptr));
      memset(allocated, 0, START_CAPAT * sizeof(bool));
    }
    ~Pool() { ::free(elems); ::free(allocated); }
    Pool(const Pool& other) = delete;
    Pool& operator=(const Pool& other) = delete;
    Pool& operator=(Pool&& other) {
      std::swap(filled, other.filled);
      std::swap(capacity, other.capacity);
      std::swap(elems, other.elems);
      std::swap(allocated, other.allocated);
      return *this;
    }
    T& get(size_t handle) {
      return elems[handle];
    }
    const T& get(size_t handle) const {
      return elems[handle];
    }
    template<typename... Args>
    size_t allocate(Args&&... args) {
      if (shouldExpand()) expand();
      size_t bucket = r() % capacity;
      while (allocated[bucket]) {
        ++bucket;
        if (bucket >= capacity) bucket -= capacity;
      }
      ++filled;
      allocated[bucket] = true;
      new(elems + bucket) T(std::forward<Args>(args)...);
      return bucket;
    }
    void deallocate(size_t handle) {
      allocated[handle] = false;
      --filled;
    }
    bool isValid(size_t handle) { return allocated[handle]; }
    size_t size() const { return filled; }
    size_t getCapacity() const { return capacity; }
  private:
    bool shouldExpand() {
      return filled * 4 >= capacity * 3;
    }
    void expand() {
      elems = trealloc<T>(elems, capacity << 1);
      allocated = trealloc<bool>(allocated, capacity << 1);
      memset(allocated + capacity, 0, capacity * sizeof(bool));
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