## zekku

Header-only utilities for game programming.

### Dependencies

The library depends on GLM for its vector classes.

### Pool

A memory pool.

* uses a separate buffer to keep track of which handles are allocated
* doubles its size when 75% full
* handles are assigned randomly, with linear probing

Use `allocate` to get a handle, `get` to convert it to a reference to what
you inserted, and `deallocate` to free the space. You can use iterators
as well.

### BoxQuadTree

WIP quadtree.

* customisable with templates
* by default, stores 32 elements per leaf
* inserting and querying (with customisable shapes)
  * supports querying with AABBs and circles out of the box
* supports any shape, not just points

For a shape to be eligible to be in BoxQuadTree, it must have the following
methods:

    bool intersects(const AABB<F>& b) const;
    bool isWithin(const AABB<F>& p) const;

Also, the quadtree doesn't store shapes themselves -- it stores instances
of an arbitrary data type. You can pass in a `GetBB` object with the
following method:

    B operator()(const T& t) const;

which takes in an element of your quadtree and returns a shape that you
want associated with that element. The default `GetBB` object looks for
a field called `box`.

(There is an older class called `QuadTree` that stores only points.)

### Licence

    Copyright 2018 AGC.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
