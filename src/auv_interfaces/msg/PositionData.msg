# Position Data Message (Internal use only)
# Provides position, velocity, acceleration, and direction information

std_msgs/Header header

# Position in meters
float64 position_x
float64 position_y
float64 position_z

# Linear velocity in m/s
float64 velocity_x
float64 velocity_y
float64 velocity_z

# Linear acceleration in m/s^2
float64 acceleration_x
float64 acceleration_y
float64 acceleration_z

# Orientation in radians (roll, pitch, yaw)
float64 roll
float64 pitch
float64 yaw

# Angular velocity in rad/s
float64 angular_velocity_x
float64 angular_velocity_y
float64 angular_velocity_z
