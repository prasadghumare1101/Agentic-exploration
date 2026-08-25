"""Mission contract: composable PRIMITIVES + safety BOUNDS.

This deliberately does NOT enumerate allowed missions. A mission is any sequence
of phases built from these primitives, so novel prompts are expressible by
composition rather than being refused for not matching a known mission type.

What IS enumerated is capability (which primitives the vehicles can actually
execute) and safety bounds (checked here and re-enforced at runtime).
"""

# Orthogonal primitives the deterministic layer can execute.
#
# `survey` is DECLARATIVE: the planner states intent (area, spacing) and the
# coordinator computes the lawnmower geometry. Determinism — the same intent
# always yields identical waypoints, which a model hand-computing trigonometry
# cannot guarantee — and it stops the LLM inventing legs that don't cover the
# stated area.
PRIMITIVES = ('takeoff', 'goto', 'navigate', 'hold', 'formation', 'survey', 'orbit',
              'follow',   # pursue a detected object; perception layer performs it
              'spiral', 'land', 'rtl', 'wait')

# Formation shapes (mirrors formation_coordinator.formations).
FORMATION_SHAPES = ('line', 'wedge', 'column', 'circle')

# Exactly which params each primitive accepts. Anything else is REJECTED: an
# unknown key is almost always a misspelling, and a misspelled safety-relevant
# field (e.g. "speed_mps" instead of "speed") would otherwise be ignored and skip
# its bounds check entirely. Equivalent to JSON-Schema additionalProperties:false.
PARAM_KEYS = {
    'takeoff':   {'altitude', 'speed'},
    'goto':      {'target', 'speed'},
    'navigate':  {'target', 'speed'},   # like goto but routed through the A* local_planner
    'hold':      set(),
    'land':      set(),
    'rtl':       set(),
    'wait':      {'duration'},
    'formation': {'shape', 'spacing', 'heading', 'anchor', 'speed'},
    'survey':    {'center', 'width', 'height', 'spacing', 'altitude', 'heading',
                  'shape', 'formation_spacing', 'speed'},
    'orbit':     {'center', 'radius', 'altitude', 'turns', 'points_per_turn',
                  'shape', 'formation_spacing', 'speed'},
    'spiral':    {'center', 'start_radius', 'growth', 'altitude', 'turns',
                  'points_per_turn', 'shape', 'formation_spacing', 'speed'},
    # follow: pursue a detected object. 'target' names the object; the remaining keys
    # describe WHERE to look for it, so the drone can sweep a search area while hunting
    # (same geometry keys as survey, which is how the search legs are generated).
    'follow':    {'target', 'center', 'width', 'height', 'spacing', 'altitude',
                  'heading', 'radius', 'speed', 'standoff'},
}
PHASE_KEYS = {'name', 'primitive', 'params', 'drones', 'assignments',
              'advance_when', 'timeout'}
PLAN_KEYS = {'phases', 'mission_id', 'unsupported', 'notes'}

# How a phase decides it is finished.
ADVANCE_MODES = ('all_arrived', 'timeout')

# --- safety bounds -------------------------------------------------------
# Checked at plan time AND continuously enforced in flight (drone_controller).
MIN_ALT = 0.5
MAX_ALT = 120.0
GEOFENCE_RADIUS = 700.0      # metres from the shared world origin (raised for ~1 km-wide pentagon ops)
MAX_SPACING = 50.0           # formation spacing sanity limit
# Formation spacing must stay ABOVE safety_manager's min_separation (2.0 m), or a
# legal formation would continuously trip the separation guard. Keep this margin.
MIN_SPACING = 2.5
MAX_PHASE_TIMEOUT = 600.0    # seconds
ARRIVAL_TOLERANCE = 1.5      # metres; phase advance threshold

# --- declarative pattern limits (survey / orbit / spiral) ---
MIN_PATTERN_SPACING = 2.0    # metres between lawnmower lanes / spiral growth
MAX_AREA_SIDE = 400.0        # metres; survey width/height cap
MAX_SURVEY_LEGS = 100        # legs from a single survey
MAX_ORBIT_RADIUS = 200.0     # metres
MAX_PATTERN_TURNS = 20       # orbit / spiral revolutions
MAX_POINTS_PER_TURN = 36     # ring resolution
# Cap on waypoints across the WHOLE plan after every pattern is expanded, so one
# prompt can never generate a mission the executor would grind through forever.
MAX_TOTAL_WAYPOINTS = 300

# --- speed ---
DEFAULT_CRUISE_SPEED = 3.0   # m/s
MAX_CRUISE_SPEED = 12.0      # m/s
