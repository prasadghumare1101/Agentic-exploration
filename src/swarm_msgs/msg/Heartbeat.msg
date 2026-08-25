# Low-bandwidth liveness beacon shared across the swarm by swarm_communication.
# Kept tiny on purpose: this is the only always-on inter-drone traffic.

std_msgs/Header header

string drone_id
uint32 seq                 # monotonically increasing per drone
uint8 status               # coarse health level (see below)

uint8 STATUS_UNKNOWN = 0
uint8 STATUS_OK      = 1
uint8 STATUS_WARN    = 2
uint8 STATUS_ERROR   = 3
