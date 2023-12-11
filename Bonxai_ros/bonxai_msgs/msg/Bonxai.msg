# timestamp and frame_id of the Bonxai grid
std_msgs/Header header

# serialized bonxai grid, use bonxai_msgs/conversion.hpp to
# read and write bonxai grids, as well as their associated metadata
# (e.g. datatype, resolution, tree-stuff, etc...).
uint8[] raw_data