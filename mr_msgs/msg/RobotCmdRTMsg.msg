# Robot command realtime message for mr to robot controller

# This data structure is sent at 50 Hz on TCP port 30003

float64 time
float64[] q_target
float64[] qd_target
float64[] qdd_target
float64[] tcp_force
float64[] tcp_speed
float64[] tcp_pose
float64[] q_current
