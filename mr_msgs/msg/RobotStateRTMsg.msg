# Robot state realtime message for mr from robot controller

# This data structure is got at 50 Hz on TCP port 30003

float64 time
float64[] q_target
float64[] qd_target
float64[] qdd_target
float64[] i_target
float64[] m_target
float64[] q_actual
float64[] qd_actual
float64[] i_actual
float64[] tool_acc_values
float64[] tcp_force
float64[] tool_vector
float64[] tcp_speed
float64 digital_input_bits
float64[] motor_temperatures
float64 controller_timer
float64 test_value
float64 robot_mode
float64[] joint_modes
