![Treexy](doc/treexy.png)

Treexy is a library that implements a compact hierarchical data structure that can store and manipulate
volumetric data, discretized on a three-dimensional grid (AKA, a "Voxel Grid").

Treexy data structure is:

- **Sparse**: it uses only a fraction of the memory that a dense 3D voxel grid would use.
- **Unbonded**: you don't need to define the boundary of the 3D space (*).

If you are familiar with [Octomap](https://octomap.github.io/) and Octrees, you know
that those data structures are also sparse and unbounded.

On the other hand, Treexy is **brutally faster** and, in same cases, even more memory efficient
than an Octree.

This work is strongly based on [OpenVDB](https://www.openvdb.org/) and it can be considered
an implementation of the original paper, with a couple of non-trivial changes:

    K. Museth, 
    “VDB: High-Resolution Sparse Volumes with Dynamic Topology”,
    ACM Transactions on Graphics 32(3), 2013. Presented at SIGGRAPH 2013.
    
You can read the previous paper [here](http://www.museth.org/Ken/Publications_files/Museth_TOG13.pdf).

There is also some overlap with this other paper, but their implementations is **much** simpler,
even if conceptually similar:

     Eurico Pedrosa, Artur Pereira, Nuno Lau 
     "A Sparse-Dense Approach for Efficient Grid Mapping"
     2018 IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC)


**Treexy** is currently under development and I am building this mostly for fun and for
educational purposes. Don't expect any API stability for the time being.

If you think that a data structure like this could be useful for your project,
for the time being you should probably considered using OpenVDB itself.

>(*) The dimension of the 3D space is virtually "infinite":
since **32-bits indexes** are used, given a voxel size as small as **1 cm**,
the maximum range of the X, Y and Z coordinates would be about **+/- 20.000 Km** (yes, Kilometers).

# Benchmark (preliminary)

Take these numbers with a grain of salt, since they are preliminary and benchmark is strongly
influenced by the way the data is stored.
Anyway, they gave you a fair idea of what you may expect, in terms of performance.

```
------------------------------------------------------------------
Benchmark                        Time             CPU   Iterations
------------------------------------------------------------------
Treexy_Create              1443778 ns      1442876 ns          484
OpenVDB_Create             1545890 ns      1534823 ns          389
Octomap_Create            26780041 ns     26750902 ns           26

Treexy_Update              1318036 ns      1317068 ns          528
OpenVDB_Update             1228747 ns      1228155 ns          570
Octomap_Update             8162226 ns      8150508 ns           83

Treexy_IterateAllCells       21371 ns        21360 ns        31614
OpenVDB_IterateAllCells      54912 ns        54885 ns        12278
Octomap_IterateAllCells     227799 ns       227383 ns         3088
```

- **Create** refers to creating a new VoxelGrid from scratch
- **Update** means modyfing the value of an already allocated VoxelGrid.
- **IterateAllCells** will get the value and the coordinates of all the existing cells.

# How to use it

The core of **Treexy** is a header-only library that you can simply copy into your project
and include like this:

```c++
#include "treexy/treexy.hpp"
```

To create a VoxelGrid, where each cell contains an integer value and has size 0.05.

```c++
double voxel_resolution = 0.05;
treexy::VoxelGrid<int> grid( voxel_resolution );
```

Nothing prevents you from having more complex cell values, for instance:

```c++
treexy::VoxelGrid<Eigen::Vector4d> vector_grid( voxel_resolution );
```

To insert values into a cell with coordinates x, y and z, use a
`VoxelGrid::Accessor` object.
In the next code sample, we will create a dense cube of cells with value 42:

```c++
// create the accessor ONCE and reuse it as much as possible
auto accessor = grid.createAccessor();

for( double x = 0; x < 1.0; x += voxel_resolution )
{
  for( double y = 0; y < 1.0; y += voxel_resolution )
  {
    for( double z = 0; z < 1.0; z += voxel_resolution )
    {
      treexy::CoordT coord = grid.posToCoord( x, y, z );
      accessor.setValue( coord, 42 );
    }
  }
}
```

Finally, to read the value of a cell:

```c++
// If the value of the cell has never been set, return nullptr
int* value = accessor.value( coord );
```

# Roadmap

- [x] serialization to/from file.
- [ ] full implementation of the Octomap algorithm (ray casting + probability map).
- [ ] integration with ROS.
- [ ] RViz/RViz2 visualization plugins.
- [ ] integration with [FCL](https://github.com/flexible-collision-library/fcl) for collision detection (?)

# Frequently Asked Question

**What is the point of reimplementing OpenVDB?**

- The number one reason is to have fun and to learn something new :)
- I want this library to be small and easy to integrate into larger projects.
  The core data structure is less than 1000 lines of code.
- It is not an "exact" rewrite, I modified few important aspects of the algorithm
    to make it slightly faster, at least for my specific use cases.

**How much memory does it uses, compared with Octomap?**

It is... complicated.

If you need to store very sparse point clouds, you should expect Treexy to use more memory (20-40% more).
If the point cloud is relatively dense, Treexy might use much less memory than Octomap (less than half).


