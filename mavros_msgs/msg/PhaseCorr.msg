# OpticalFlow2D message
# time_usec: timestamp of message creation in micro seconds since ros started
# time_delta_usec: elapsed time since last reading in micro seconds
# phase_shift_x: translational x-shift between current and last received image in subpixels (double precision)
# phase_shift_y: translational y-shift between current and last received image in subpixels
# phase_shift_sum_x: integrated translational x-shift since first received image
# phase_shift_sum_y: integrated translational y-shift since first received image

# message data
uint64 time_usec
uint64 time_delta_usec
float64 phase_shift_x
float64 phase_shift_y
float64 phase_shift_sum_x
float64 phase_shift_sum_y
