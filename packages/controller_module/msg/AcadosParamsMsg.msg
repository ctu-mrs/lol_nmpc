Header header

float64 ac_num_iter
float64 ac_horizon
float64[] ac_time_steps

float64 Q_pos_xy
float64 Q_pos_z
float64 Q_rot_xy
float64 Q_rot_z
float64 Q_vel
float64 Q_omega
float64 Q_thrust

float64 Q_int
float64 R_col_thrust
float64 R_ome_in

# constraints
float64 thrust_actuator_max
float64 thrust_actuator_min
float64 thrust_collective_max
float64 thrust_collective_min
float64[] max_rot_speed
float64[] max_input_rot_speed

float64 tau_mot
