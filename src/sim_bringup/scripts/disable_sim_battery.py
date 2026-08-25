#!/usr/bin/env python3
"""Disable PX4 SITL battery drain on all running instances.

PX4 simulates battery discharge (SIM_BAT_DRAIN / SIM_BAT_MIN_PCT). After a sim
has run for a while the battery "depletes", and PX4 then DENIES ARMING
("Resolve system health failures first") or triggers a battery FAILSAFE that
holds/lands the vehicle regardless of offboard commands. That looks exactly like
the swarm "always holding / one drone won't take off" — but it is PX4 overriding
us, not the coordinator.

Run this ONCE after the sim is up (before or just after launching the stack):
    python3 src/sim_bringup/scripts/disable_sim_battery.py

Instance N (sitl_multiple_run) listens on MAVLink UDP 14540+N.
Needs: pip install pymavlink
"""

import sys
from pymavlink import mavutil

INSTANCES = [int(a) for a in sys.argv[1:]] or [1, 2, 3]

for inst in INSTANCES:
    port = 14540 + inst
    try:
        m = mavutil.mavlink_connection(f'udpin:0.0.0.0:{port}')
        m.wait_heartbeat(timeout=10)
        for name, val in (('SIM_BAT_ENABLE', 0.0), ('SIM_BAT_MIN_PCT', 99.0)):
            m.mav.param_set_send(m.target_system, m.target_component,
                                 name.encode(), val,
                                 mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        print(f'  px4_{inst} (udp {port}): battery sim disabled')
        m.close()
    except Exception as e:
        print(f'  px4_{inst} (udp {port}): FAILED - {e}')

print('done. If a drone was already in battery failsafe, restart that sim '
      'instance; new arming will now succeed.')
