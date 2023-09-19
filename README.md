![Bonxai](doc/bonxai.png)

Bonxai is a library that implements a compact hierarchical data structure that
can store and manipulate volumetric data, discretized on a three-dimensional
grid (AKA, a "Voxel Grid").

Bonxai data structure is:

- **Sparse**: it uses only a fraction of the memory that a dense 3D voxel grid would use.
- **Unbounded**: you don't need to define the boundary of the 3D space (*).

>(*) The dimension of the 3D space is virtually "infinite":
since **32-bits indexes** are used, given a voxel size of **1 cm**,
the maximum range of the X, Y and Z coordinates would be about **40.000 Km**.
As a reference **the diameter of planet Earth is 12.000 Km**.

If you are familiar with [Octomap](https://octomap.github.io/) and Octrees, you know
that those data structures are also sparse and unbounded.

On the other hand, Bonxai is **much faster** and, in some cases, even more memory efficient
than an Octree.

This work is strongly based on [OpenVDB](https://www.openvdb.org/) and it can be considered
an implementation of the original paper, with a couple of non-trivial changes:

    K. Museth, 
    “VDB: High-Resolution Sparse Volumes with Dynamic Topology”,
    ACM Transactions on Graphics 32(3), 2013. Presented at SIGGRAPH 2013.
    
You can read the previous paper [here](http://www.museth.org/Ken/Publications_files/Museth_TOG13.pdf).

There is also some overlap with this other paper, but their implementation is much** simpler,
even if conceptually similar:

     Eurico Pedrosa, Artur Pereira, Nuno Lau 
     "A Sparse-Dense Approach for Efficient Grid Mapping"
     2018 IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC)


**Bonxai** is currently under development and I am building this mostly for fun and for
educational purposes. Don't expect any API stability for the time being.

# Benchmark (preliminary)

Take these numbers with a grain of salt, since they are preliminary and the benchmark is 
strongly influenced by the way the data is stored.
Anyway, they gave you a fair idea of what you may expect, in terms of performance.

```
-------------------------------------------
Benchmark                     Time      
-------------------------------------------
Bonxai_Create              1165 us  
Octomap_Create            25522 us  

Bonxai_Update               851 us  
Octomap_Update             3824 us  

Bonxai_IterateAllCells      124 us
Octomap_IterateAllCells     698 us
```

- **Create** refers to creating a new VoxelGrid from scratch
- **Update** means modifying the value of an already allocated VoxelGrid.
- **IterateAllCells** will get the value and the coordinates of all the existing cells.

# How to use it

The core of **Bonxai** is a header-only library that you can simply copy into your project
and include like this:

```c++
#include "bonxai/bonxai.hpp"
```

To create a VoxelGrid, where each cell contains an integer value and has size 0.05.

```c++
double voxel_resolution = 0.05;
Bonxai::VoxelGrid<int> grid( voxel_resolution );
```

Nothing prevents you from having more complex cell values, for instance:

```c++
Bonxai::VoxelGrid<Eigen::Vector4d> vector_grid( voxel_resolution );
// or
struct Foo {
 int a;
 double b;
};
Bonxai::VoxelGrid<Foo> foo_grid( voxel_resolution );
```

To insert values into a cell with coordinates x, y and z, use a
`VoxelGrid::Accessor` object.
In the next code sample, we will create a dense cube of cells with value 42:

```c++
// Each cell will contail a `float` and it will have size 0.05
double voxel_resolution = 0.05;
Bonxai::VoxelGrid<float> grid( voxel_resolution );

// Create this accessor once, and reuse it as much as possible.
auto accessor = grid.createAccessor();

// Create cells with value 42.0 in a 1x1x1 cube.
// Given voxel_resolution = 0.05, this will be equivalent
// to 20x20x20 cells in the grid.

for( double x = 0; x < 1.0; x += voxel_resolution ) {
  for( double y = 0; y < 1.0; y += voxel_resolution ) {
    for( double z = 0; z < 1.0; z += voxel_resolution ) {
      // discretize the position {x,y,z}
      Bonxai::CoordT coord = grid.posToCoord(x, y, z);
      accessor.setValue( coord, 42.0 );
    }
  }
}

// You can read (or update) the value of a cell as shown below.
// If the cell doesn't exist, `value_ptr` will be nullptr, 

Bonxai::CoordT coord = grid.posToCoord(x, y, z);
float* value_ptr = accessor.value( coord );
```

## Note about multi-threading

`Bonxai::VoxelGrid` is **not** thread safe, for write operations.

If you want to access the grid in **read-only** mode, you can
use multi-threading, but each thread should have its own 
`accessor`.

# Roadmap

- [x] serialization to/from file.
- [x] full implementation of the Octomap algorithm (ray casting + probability map).
- [ ] integration with ROS.
- [ ] RViz/RViz2 visualization plugins.
- [ ] integration with [FCL](https://github.com/flexible-collision-library/fcl) for collision detection (?)

# Frequently Asked Question

**What is the point of reimplementing OpenVDB?**

- The number one reason is to have fun and to learn something new :)
- I want this library to be small and easy to integrate into larger projects.
  The core data structure is less than 1000 lines of code.
- It is not an "exact" rewrite, I modified a few important aspects of the algorithm
    to make it slightly faster, at least for my specific use cases.

**How much memory does it use, compared with Octomap?**

It is... complicated.

If you need to store very sparse point clouds, you should expect Bonxai to use more memory (20-40% more).
If the point cloud is relatively dense, Bonxai might use much less memory than Octomap (less than half).


