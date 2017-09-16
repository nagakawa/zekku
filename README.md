## zekku

Header-only utilities for game programming.

### Pool

A memory pool.

* uses a separate buffer to keep track of which handles are allocated
* doubles its size when 75% full
* handles are assigned randomly, with linear probing

### QuadTree

WIP quadtree.

* customisable with templates
* by default, stores 4 elements per leaf

### Licence

GNU GPLv3 (see `LICENCE`).