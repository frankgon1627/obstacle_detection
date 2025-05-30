# Header for time stamp and frame information
std_msgs/Header header

# MetaData for the map (resolution, origin, dimensions)
nav_msgs/MapMetaData info

# The map data: a float array instead of int8
float32[] data
