# ID of the object being checked.
int32 node_id

# This id is set to the object with which this object is merged with. If the object is not
# merged, this is set to the 0.
int32 merged_id

# Key timestamps of the object where observed. History of an object is:
# first_absent -> first_persistent -> obj.first_seen -> obj.last_seen -> last_persistent -> last_absent.

# Time stamp when there was last evidence of absence before this object was first seen. If the
# object was never observed absent before, this is set to 0.
uint64 first_absent

# Time stamp when there was first evidence of absence after this object was last seen. If the
# object was never observed absent after, this is set to 0.
uint64 last_absent

# Time stamp when there was first evidence of presence before this object was first seen. If the
# object was never observed present before, this is set to 0.
uint64 first_persistent

# Time stamp when there was last evidence of presence after this object was last seen. If the
# object was never observed present after, this is set to 0.
uint64 last_persistent
