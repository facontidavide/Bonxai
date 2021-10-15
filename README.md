![Treexy](treexy.png)

Treexy is a library that implements a compact hierarchical data structure that can store and manipulate
volumetric data, discretized on a three-dimensional grid (AKA, a "Voxel Grid").

This data structure is:

- **Sparse**: it uses only a fraction of the memory that a dense 3D voxel grid would use.
- **Unbonded**: you don't need to define the boundary of the 3D space.

For practical purposes, the dimension of the 3D space is virtually "infinite":
if a voxel size of **1 cm** is used, the maximum range of your X, Y and Z coordinates
will be **+/- 20.000 Km** (yes, Kilometers).

If you are familiar with [Octomap](https://octomap.github.io/) and Octree, you know
that those data structures are also sparse and unbounded.

On the other hand, Treexy is **brutally faster** and, in same case, even more memory efficient than an Octree.

This work is strongly inspired by [OpenVDB](https://www.openvdb.org/) and it can be considered
an implementation of the original paper, with a couple of changes:

    K. Museth, “VDB: High-Resolution Sparse Volumes with Dynamic Topology”,
    ACM Transactions on Graphics 32(3), 2013. Presented at SIGGRAPH 2013.
    
http://www.museth.org/Ken/Publications_files/Museth_TOG13.pdf    

It is currently under development and I am building this mostly for fun and for
educational purposes.

If you think that a data structure like this could be useful for your project,
for the time being you should probably considered using OpenVDB itself.
