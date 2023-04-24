# Optimization Of Programs

This repository contains the tasks for the lecuture Program Optimization at the University of Applied Sciences Karlsruhe.

## Task 1

The intersection test is to be supplemented by an optimized variant.
The following calculations shall be delayed and saved:

- If the parameter t is not smaller than the previous intersection parameter
minimum_t, the test shall be terminated early.
- The u-v parameters are to be calculated only at the end of the function.
including the necessary intermediate results.
- Instead of calculating the square root three times by calling the length method
method three times, it is to be calculated only two times. For this the
the square of the length with square_of_length() and the calculation of the u-v
calculation of the u-v parameters must be transformed appropriately.

### Compile and Run

```bash
g++ -Wall -pedantic -march=native -mfpmath=sse -mavx -O3 -D OPTIMIZED_INTERSECTS raytracer.cc statistics.cc
./a.out
```

## Task 2

The calculation of the square root is relatively time consuming depending on the accuracy and precision. Therefore, it is worth optimizing it. Minor inaccuracies can be accepted here.

### Compile and Run

```bash
g++ -Wall -pedantic -march=native -mfpmath=sse -mavx2 -O3 sqrt_opt.cc
./a.out
```

### Compile to Assembler with Debug Info

```bash
g++  -S -g
```

Alternatively by source code:

```bash
g++-c -g example.cc
objdump -S example.o > example.s
```

## Task 3

The number of intersection tests is too high due to the brute force approach.
It can be reduced with the help of spatial subdivisions.
A k-d tree is to be constructed as follows:

- The tree is created for all triangles at once, instead of inserting each triangle individually.
individually.
- The axis-parallel bounding box of the scene is computed at the beginning.
- The k-d tree is built recursively. The bounding box along the
longest axis in the middle into a "left" and a "right" bounding-box.
along the longest axis.
- If a triangle is in both boxes, it is added to the end of the list of the current node.
otherwise it is added to a "left" or "right" list at the end.
or "right" list at the end. These two lists are then
used recursively for the construction of the left and right subtree.
is used.
- The recursion stops when the number of triangles to be inserted falls below a given bound.
a predefined bound.
The search for an intersection point is also to be implemented recursively.
be implemented recursively:
- The search is aborted if the ray of vision does not intersect the bounding box of the node.
node is not intersected.
- If a left or right subtree exists, an intersection point is recursively searched in these subtrees.
is recursively searched for an intersection point.
- After the recursion, an intersection point for the list of triangles at the node is searched for the ray of vision.
of triangles at the node.

The optimized variant of the intersection test shall be used.

### Compile and Run

Run without k-d-trees:

```bash
g++ -Wall -pedantic -std=c++11 -march=native -mfpmath=sse -msse -O3 -D OPTIMIZED_INTERSECTS raytracer.cc statistics.cc kdtree.cc
./a.out
```

Run with k-d-trees:

```bash
g++ -Wall -pedantic -std=c++11 -march=native -mfpmath=sse -msse -O3 -D OPTIMIZED_INTERSECTS -D USE_KDTREE raytracer.cc statistics.cc kdtree.cc
./a.out
```
