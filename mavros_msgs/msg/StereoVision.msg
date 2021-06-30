# STEREO_VISION message
# time_usec:        timestamp of message creation in micro seconds since ros started
# time_delta_usec:  elapsed time since last reading in micro seconds
# distance:         distance between camera and the surface looked at
# delta_pitch:      pitch offset (radians) between the optical axis of the camera and the normal vector of the regarded surface
# delta_yaw:        yaw offset (radians) between the optical axis of the camera and the normal vector of the regarded surface

# message data
std_msgs/Header header
uint64 time_usec
uint64 time_delta_usec
float32 distance
float32 delta_pitch
float32 delta_yaw
