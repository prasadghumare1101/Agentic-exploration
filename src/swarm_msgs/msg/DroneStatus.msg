# Per-drone status summary published by drone_controller (the node that already
# owns the PX4 subscriptions). swarm_communication reads this to color heartbeats;
# other drones can subscribe for basic shared situational awareness.

std_msgs/Header header

string drone_id
uint8 nav_state            # mirrors px4_msgs VehicleStatus.nav_state
uint8 arming_state         # mirrors px4_msgs VehicleStatus.arming_state
bool offboard_active

geometry_msgs/Point position  # local ENU position (meters)
float32 battery_remaining     # 0.0 .. 1.0 (-1.0 if unknown)
