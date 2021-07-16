# VISUAL_ODOMETRY message
# time_sec:        timestamp of message creation in micro seconds since ros started
# time_delta_sec:  elapsed time since last reading in micro seconds
# vel_net:         velocity of net calculated from optical flow and distance to net in meters per second, positive direction counter-clockwise
# dist_net:        accumulated distance traveled calculated from vel_net in meters

# message data
std_msgs/Header header
uint64 time_usec
uint64 time_delta_usec

float32 vel_net
float32 dist_net
