string SET_TASK=SET_TASK
string REPLACE_VIA_POINTS=REPLACE_VIA_POINTS
string PAUSE=PAUSE
string RESUME=RESUME
string CANCEL=CANCEL

uint8 protocol_version
# Receiver boot identity from /air_ground/state; not a sender lease.
string server_id
# One logical serialized producer; strictly increasing within receiver boot.
uint64 sequence
string request_id
string task_id
string operation
# Used by SET_TASK only; otherwise leave the default empty message.
GeoPoint final_goal
# Used by SET_TASK and REPLACE_VIA_POINTS. Empty REPLACE clears remaining vias.
GeoPoint[] via_points
