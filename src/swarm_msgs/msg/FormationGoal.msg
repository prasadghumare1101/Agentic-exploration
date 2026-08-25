# A squad-level formation GOAL, broadcast by the coordinator when running in
# agentic mode. Unlike a per-drone DroneTask, this does NOT say which drone goes
# where — it states the shape and where the group should be, and the drone_agents
# decide amongst themselves (distributed auction) who takes which slot. This is the
# decentralised half of the swarm: intent is central, assignment is emergent.

std_msgs/Header header

string mission_id
int32  phase_idx           # phase this goal belongs to (agents tag their bids/arrivals)
string shape               # "wedge" | "line" | "column" | "circle"
float64 spacing            # metres between adjacent slots
float64 heading            # degrees, 0 = North (direction of travel)
geometry_msgs/Point anchor # group centre in the SHARED world frame (x=East, y=North, z=altitude up)
int32  num_slots           # number of slots to fill (= number of participating drones)
