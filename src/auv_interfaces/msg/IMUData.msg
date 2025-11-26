# IMU (Inertial Measurement Unit) Data Message
# Provides acceleration and angle information

std_msgs/Header header

# Linear acceleration in m/s^2
float64 acceleration_x
float64 acceleration_y
float64 acceleration_z

# Angular velocity in rad/s
float64 angular_velocity_x
float64 angular_velocity_y
float64 angular_velocity_z

# Orientation in radians (roll, pitch, yaw)
float64 roll
float64 pitch
float64 yaw
