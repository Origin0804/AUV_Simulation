# Objects Data Message (Internal use only)
# Provides object positions for camera simulation

std_msgs/Header header

# Number of objects
int32 num_objects

# Object IDs
int32[] object_ids

# Object types
string[] object_types

# Object positions in world frame (x, y, z for each object)
float64[] positions_x
float64[] positions_y
float64[] positions_z
