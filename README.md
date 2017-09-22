## zekku

Header-only utilities for game programming.

### Dependencies

The library depends on GLM for its vector classes.

### Pool

A memory pool.

* uses a separate buffer to keep track of which handles are allocated
* doubles its size when 75% full
* handles are assigned randomly, with linear probing

### QuadTree

WIP quadtree.

* customisable with templates
* by default, stores 4 elements per leaf
* inserting and querying (with customisable shapes)
  * supports querying with AABBs and circles out of the box

### Licence

GNU GPLv3 or later (see `LICENCE`).