Header header
geometry_msgs/Vector3 quat_error
float64 computation_time_ns
float64 computation_time_us 
bool computation_success
controller_module/DroneStateMsg initial_state
string git_branch
string git_hash

# Arrays of messages
time[] stamps
controller_module/NmpcControlOutput[] control_outputs
controller_module/DroneStateMsg[] predicted_drone_states
controller_module/DroneStateMsg[] reference_trajectory

# TODO : Make a way to receive the set costs

