# this message is contains a collection of voxels representing a map 
# of occupied voxels generated using bonxai this contains N voxels and
# the resolution of the map and relies on header to contain the frame_id 
# relative to the map

std_msgs/Header header
float64 voxel_resolution
Voxel[] occupied_voxels
Voxel[] free_voxels
