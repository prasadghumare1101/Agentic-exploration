# A composable mission plan: an ordered list of phases, each assigning primitives
# to drones. This REPLACES the fixed mission-type whitelist — any prompt that can
# be decomposed into primitives is expressible, and safety is enforced at runtime
# rather than by refusing unknown missions up front.
#
# The plan body is carried as JSON because phases/assignments are variable-depth.
# It is structurally validated and BOUNDS-CHECKED (altitude/geofence/spacing)
# before it is ever published, so this is not unchecked free-form data.
#
# plan_json shape:
# {
#   "phases": [
#     {"name": "form up",
#      "primitive": "formation",
#      "params": {"shape":"wedge","spacing":5,"heading":0,"anchor":[0,3,4]},
#      "drones": ["px4_1","px4_2"],          # optional; default = all
#      "assignments": [                        # optional per-drone override
#         {"drone":"px4_1","primitive":"goto","params":{"target":[0,0,4]}}
#      ],
#      "advance_when": "all_arrived",          # or "timeout"
#      "timeout": 30.0}
#   ]
# }

std_msgs/Header header

string mission_id
string plan_json

# Capabilities the planner could NOT express (e.g. "film", "follow target").
# Reported honestly instead of silently substituting a different mission.
string[] unsupported
