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
  // ------------------
  // Helper methods for destroying array
  // This is a bit confusing. Basically, the first overload is called
  // if T is not trivially destructible, and the second otherwise.
  template<typename T,
    typename std::enable_if<
      !std::is_trivially_destructible<T>::value, int>::type = 0>
  void freeElems(T* elems, bool* allocated, size_t size) {
    for (size_t i = 0; i < size; ++i) {
      if (!allocated[i]) {
        elems[i].~T();
      }
    }
  }
  template<typename T,
    typename std::enable_if<
      std::is_trivially_destructible<T>::value, int>::type = 0>
  void freeElems(T* elems, bool* allocated, size_t size) {}
  // ------------------
  constexpr size_t START_CAPAT = 64;
  template<typename T>
  class Pool {
  public:
    // static_assert(std::is_trivially_copyable<T>::value,
    //   "Your T is not trivially copyable, dum dum!");
    Pool(size_t c = START_CAPAT) : filled(0), capacity(c),
        elems(tmalloc<T>(c)),
        allocated(tmalloc<bool>(c)) {
      r.seed(time(nullptr));
      memset(allocated, 0, c * sizeof(bool));
    }
    ~Pool() {
      freeElems(elems, allocated, capacity);
      ::free(elems);
      ::free(allocated);
    }
    Pool(const Pool& other) = delete;
    Pool& operator=(const Pool& other) = delete;
    Pool(Pool&& other) :
        filled(other.filled), capacity(other.capacity),
        elems(other.elems), allocated(other.allocated) {
      r.seed(time(nullptr));
      other.filled = 0;
      other.capacity = START_CAPAT;
      other.elems = tmalloc<T>(START_CAPAT);
      other.allocated = tmalloc<bool>(START_CAPAT);
      memset(other.allocated, 0, START_CAPAT * sizeof(bool));
    }
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
      size_t bucket = r() & (capacity - 1);
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
    struct iterator {
      Pool* p;
      size_t i;
      bool operator==(const iterator& other) {
        return p == other.p && i == other.i;
      }
      bool operator!=(const iterator& other) {
        return !(*this == other);
      }
      iterator operator++(int) {
        iterator ip = *this;
        ++(*this);
        return ip;
      }
      iterator operator--(int) {
        iterator ip = *this;
        --(*this);
        return ip;
      }
      iterator& operator++() {
        do ++i;
        while (!p->allocated[i] && i < p->capacity);
        return *this;
      }
      iterator& operator--() {
        do --i;
        while (!p->allocated[i] && i > 0);
        return *this;
      }
      T& operator*() { return p->elems[i]; }
      const T& operator*() const { return p->elems[i]; }
    };
    iterator begin() { return { this, 0        }; }
    iterator end()   { return { this, capacity }; }
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