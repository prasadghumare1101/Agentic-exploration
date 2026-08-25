# One drone_agent's bid in the distributed slot auction. Each agent broadcasts its
# travel cost to EVERY slot; every agent collects all bids and runs the SAME
# deterministic assignment, so they converge on one collision-minimising layout
# with no central assigner. Re-broadcast whenever the goal or the alive-set changes.

std_msgs/Header header

string  mission_id
int32   phase_idx          # which FormationGoal this bid answers (stale bids are ignored)
string  drone_id
float64[] costs            # costs[s] = this drone's travel cost to slot s (metres)
