# CV (Computer Vision) Model Message
# Provides target detection information based on simulated camera

std_msgs/Header header

# Whether any target exists in the field of view
bool target_detected

# Number of detected objects
int32 num_objects

# List of detected objects
auv_interfaces/DetectedObject[] objects
