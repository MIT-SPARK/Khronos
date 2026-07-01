# Info describing a single changed object (removed or newly added), reported by the
# ActiveWindowChangeDetectorPublisher sink.

# Identifier: prior-DSG NodeId for removed objects, Track id for added objects.
int64 id

# Time the change occurred (added -> Track::first_seen, removed -> detection/publish time).
builtin_interfaces/Time stamp

# --- Attributes below are populated for ADDED objects only; left default/zero for removed. ---

# Object centroid in the report frame (see AwcdChanges.msg header.frame_id).
geometry_msgs/Point centroid

# Oriented bounding box, in the report frame.
geometry_msgs/Point bbox_center
geometry_msgs/Vector3 bbox_dimensions
geometry_msgs/Quaternion bbox_orientation

# Semantics.
int32 semantic_label
string name
float32 confidence
