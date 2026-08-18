# Info describing a single changed object (removed or newly added), reported by the
# ActiveWindowChangeDetectorPublisher sink.

# Identifier: prior-DSG NodeId for removed objects, detector-owned Object id for added objects
# (stable for the object's lifetime, independent of any underlying Track id).
int64 id

# Time the change occurred (added -> AddedObject::first_seen, removed -> detection/publish time).
builtin_interfaces/Time stamp

# Change-detection confidence and its supporting frame count, populated for BOTH kinds:
# added -> free-space containment EMA (AddedObject::change_confidence) and the number of frames
# that fed it; removed -> free-space probability EMA (RemovedObject::change_confidence) and its
# frame count. This is the signal that actually drives the added/removed decision -- see
# `confidence` below for the (added-only) tracking-quality signal.
float32 change_confidence
int32 num_frames_observed

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

# Tracking-quality confidence (how well-observed/mature the contributing tracks are), distinct
# from change_confidence above which drives the added-object decision. Not populated for removed
# objects.
float32 confidence
