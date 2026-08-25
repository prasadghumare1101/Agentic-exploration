# Compact per-drone assignment produced by formation_coordinator (or safety_manager).
# One drone -> one active task at a time. The drone_controller expands this into
# PX4 offboard setpoints locally. This is NOT a continuous setpoint stream.

std_msgs/Header header

string mission_id
string drone_id            # e.g. "px4_1"
string action              # "takeoff" | "goto" | "hold" | "rtl" | "land"

geometry_msgs/Point target # target waypoint (local ENU); ignored for hold/rtl/land
float64 target_altitude    # altitude in meters (positive up); used by takeoff/goto
float64 cruise_speed       # m/s the setpoint advances toward target; <=0 = node default
float64 yaw                # desired heading (radians, 0=North); NaN = face travel

# Task-ownership ladder (higher wins; equal priority = latest task wins in the
# controller). Exactly one source drives a drone at a time:
#   SAFETY(255) > FOLLOW(25) > NAV(20) > NORMAL(10)
# NORMAL = coordinator/agent mission; NAV = local_planner (autonomous nav goal);
# FOLLOW = target_follow (a spotted target outranks generic nav); SAFETY = safety_manager.
uint8 priority
uint8 PRIORITY_NORMAL = 10
uint8 PRIORITY_NAV    = 20
uint8 PRIORITY_FOLLOW = 25
uint8 PRIORITY_SAFETY = 255
