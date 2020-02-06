# NetTrackingMarker message

# message data
# time_usec: timestamp of message creation in micro seconds since ros started
# time_delta_usec: elapsed time since last reading in micro seconds
# marker_visible Whether an aruco marker is currently detected
# terminate Whether a termination marker is currently detected
# horizontal_pos Horizontal position of the upper visible marker normalized to the image width (-1 ... +1)

uint64 time_usec
uint64 time_delta_usec
bool marker_visible
bool terminate
float64 horizontal_pos
