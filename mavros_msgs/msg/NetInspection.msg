# NetInspection message
# time_usec: timestamp of message creation in micro seconds since ros started
# time_delta_usec: elapsed time since last reading in micro seconds
# mesh_distribution: ratio (0...1) between net meshes on left image side and all the detected net meshes (used for net edge detection)
# absolute_damages: absolute count of found damages
# mesh_count: count of all the detected currently visible net meshes

# message data
uint64 time_usec
uint64 time_delta_usec
float64 mesh_distribution
uint64 absolute_damages
uint64 mesh_count
