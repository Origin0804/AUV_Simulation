# Total Force Message (Internal use only)
# Contains forces and torques acting on the AUV

std_msgs/Header header

# Forces in Newtons
float64 force_x
float64 force_y
float64 force_z

# Torques in Newton-meters
float64 torque_rx
float64 torque_ry
float64 torque_rz
