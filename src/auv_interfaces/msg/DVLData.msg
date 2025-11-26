# DVL (Doppler Velocity Log) Data Message
# Provides velocity information at a certain frequency

std_msgs/Header header

# Linear velocity in m/s
float64 velocity_x
float64 velocity_y
float64 velocity_z

# Validity flags
bool valid
