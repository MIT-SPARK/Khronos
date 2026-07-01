# Batch of object changes detected by the ActiveWindowChangeDetector in a single frame,
# published by the ActiveWindowChangeDetectorPublisher sink for the base station to consume.

# header.stamp = publish time. header.frame_id = frame the attributes below are reported in.
std_msgs/Header header

# Name of the robot that produced this report.
string robot_name

# Objects present in the prior map but detected as removed this frame.
khronos_msgs/ChangedObjectInfo[] removed_objects

# Objects newly detected and not present in the prior map.
khronos_msgs/ChangedObjectInfo[] added_objects
