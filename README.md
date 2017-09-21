## zekku

Header-only utilities for game programming.

### Pool

A memory pool.

* uses a separate buffer to keep track of which handles are allocated
* doubles its size when 75% full
* handles are assigned randomly, with linear probing

### Vec

Custom vector class.

* +-*/ (all elementwise)
* .dot() dot product
* .r2() square of magnitude
* cross product nyi

### QuadTree

WIP quadtree.

* customisable with templates
* by default, stores 4 elements per leaf
* inserting and querying (with customisable shapes)
  * supports querying with AABBs and circles out of the box

### Licence

GNU GPLv3 or later (see `LICENCE`).