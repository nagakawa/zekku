#pragma once

#ifndef ZEKKU_BOX_GEOMETRY_H
#define ZEKKU_BOX_GEOMETRY_H

#include <glm/glm.hpp>
#include "zekku/base.h"
#include "zekku/timath.h"

namespace zekku {
  
  template<typename F = float>
  struct AABB {
    static_assert(std::numeric_limits<F>::is_specialized,
      "Your F is not a number, dum dum!");
    glm::tvec2<F> c;
    glm::tvec2<F> s; // centre to corner
    AABB<F> nw() const {
      return {c - s * oneHalf<F>, s * oneHalf<F>};
    }
    AABB<F> ne() const {
      return {c + s * glm::tvec2<F>{oneHalf<F>, -oneHalf<F>}, s * oneHalf<F>};
    }
    AABB<F> sw() const {
      return {c + s * glm::tvec2<F>{-oneHalf<F>, oneHalf<F>}, s * oneHalf<F>};
    }
    AABB<F> se() const {
      return {c + s * oneHalf<F>, s * oneHalf<F>};
    }
    glm::tvec2<F> nwp() const { return c - s; }
    glm::tvec2<F> nep() const { return c + s * glm::tvec2<F>{1, -1}; }
    glm::tvec2<F> swp() const { return c + s * glm::tvec2<F>{-1, 1}; }
    glm::tvec2<F> sep() const { return c + s; }
    bool contains(glm::tvec2<F> p) const {
      return
        p.x >= c.x - s.x &&
        p.x <= c.x + s.x &&
        p.y >= c.y - s.y &&
        p.y <= c.y + s.y;
    }
    bool contains(const AABB<F>& p) const {
      return
        p.c.x - p.s.x >= c.x - s.x &&
        p.c.x + p.s.x <= c.x + s.x &&
        p.c.y - p.s.y >= c.y - s.y &&
        p.c.y + p.s.y <= c.y + s.y;
    }
    bool isWithin(const AABB<F>& p) const {
      return p.contains(*this);
    }
    bool intersects(const AABB<F>& p) const {
      return
        (zekku::abs(c.x - p.c.x) <= (s.x + p.s.x)) &&
        (zekku::abs(c.y - p.c.y) <= (s.y + p.s.y));
    }
    size_t getClass(glm::tvec2<F> p) const {
      bool east = p.x > c.x;
      bool south = p.y > c.y;
      return (south << 1) | east;
    }
    AABB getSubboxByClass(uint32_t cl) const {
      int32_t east  = cl & 1; // 1 if east, 0 if west
      int32_t south = cl & 2; // 2 if south, 0 if north
      glm::tvec2<F> dir{
        (F) ((east << 1) - 1), (F) (south - 1)};
      glm::tvec2<F> halfs = s * oneHalf<F>;
      return {
        c + halfs * dir,
        halfs
      };
    }
  };
  template<typename F>
  bool operator<(const glm::tvec2<F>& a, const glm::tvec2<F>& b) {
    if (a.x < b.x) return true;
    if (b.x < a.x) return false;
    return a.y < b.y;
  }
  template<typename F>
  bool operator==(const glm::tvec2<F>& a, const glm::tvec2<F>& b) {
    return a.x == b.x && a.y == b.y;
  }
  template<typename F>
  bool operator<(const AABB<F>& a, const AABB<F>& b) {
    if (a.c < b.c) return true;
    if (b.c < a.c) return false;
    return a.s < b.s;
  }
  template<typename F>
  bool operator==(const AABB<F>& a, const AABB<F>& b) {
    return a.c == b.c && a.s == b.s;
  }
  template<typename F = float>
  struct Line;
  template<typename F = float>
  struct Circle {
    Circle(const glm::tvec2<F>& c, F r) : c(c), r(r) {}
    Circle() : c(0), r(0) {}
    glm::tvec2<F> c;
    F r;
    bool contains(glm::tvec2<F> p) const {
      glm::tvec2<F> d = c - p;
      return zekku::isWithin(d.x, d.y, r);
    }
    /*
    This method considers a rounded rectangle around the AABB with
    thickness r. This figure can be thought of as the union of
    four circles and two rectangles.
    thanks https://gamedev.stackexchange.com/a/120897
    */
    bool intersects(const AABB<F>& b) const {
      F dx = std::max(zekku::abs(c.x - b.c.x) - b.s.x, F{0});
      F dy = std::max(zekku::abs(c.y - b.c.y) - b.s.y, F{0});
      return zekku::isWithin(dx, dy, r);
    }
    bool intersects(const Circle<F>& b) const {
      F dx = c.x - b.c.x;
      F dy = c.y - b.c.y;
      return zekku::isWithin(dx, dy, r + b.r);
    }
    bool intersects(const Line<F>& l) const;
    bool isWithin(const AABB<F>& p) const {
      AABB<F> bounding = { c, { r, r } };
      return bounding.isWithin(p);
    }
  };
  template<typename F>
  struct Line {
    Line(const glm::tvec2<F>& x1, const glm::tvec2<F>& x2)
      : x1(x1), x2(x2) {}
    Line() : x1(0, 0), x2(0, 0) {}
    glm::tvec2<F> x1, x2;
    bool isWithin(const AABB<F>& b) const {
      return
        zekku::abs(x1.x - b.c.x) <= b.s.x &&
        zekku::abs(x1.y - b.c.y) <= b.s.y &&
        zekku::abs(x2.x - b.c.x) <= b.s.x &&
        zekku::abs(x2.y - b.c.y) <= b.s.y;
    }
    bool intersects(const Line<F>& b) const {
      glm::tvec2<F> r = x2 - x1;
      glm::tvec2<F> s = b.x2 - b.x1;
      DoubleType<F> un = cross2(b.x1 - x1, r);
      DoubleType<F> ud = cross2(r, s);
      if (ud == 0) return un == 0;
      return un >= 0 && un <= ud;
    }
    bool intersects(const Circle<F>& sh) const {
      glm::tvec2<F> r = x2 - x1;
      glm::tvec2<F> f = x1 - sh.c;
      using D = DoubleType<F>;
      using DD = DoubleType<D>;
      D a = dotUnfucked(r, r);
      D b = dotUnfucked(r, f);
      D c = dotUnfucked(f, f) - (D) sh.r * sh.r;
      DD d2 = zekku::longMultiply(b, b) - zekku::longMultiply(a, c) * 4;
      if (d2 < 0) return false;
      D d = zekku::sqrt<D>(d2);
      D t1n = b - d;
      if (t1n >= 0 && t1n <= 2 * a) return true;
      D t2n = b + d;
      return t2n >= 0 && t2n <= 2 * a;
    }
  };
  template<typename F>
  bool Circle<F>::intersects(const Line<F>& l) const {
    return l.intersects(*this);
  }
}

#endif