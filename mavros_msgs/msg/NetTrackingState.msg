# NetTrackingState message

# message data
# state Index of the current state of the net tracking state machine (defined in AP_NetTracking/AP_NetTracking.h of ArduPilot)
# loop_progress Progress of a 360 degrees scan in percent

std_msgs/Header header
uint8 state
float64 loop_progress
