"""Deterministic formation geometry — pure math, no ROS, no LLM.

Computes per-drone slot positions for a group anchored at a `center` with a
`heading`. This is the coordination primitive the squad-level intent commands:
the LLM says "wedge, spacing 5, heading north"; this returns the exact slots and
the executor/agents hold them.

Frame: NED horizontal plane, x=North, y=East, heading = yaw clockwise from North
(radians). Altitude is handled by the caller. Slot 0 is the reference slot
(apex/front for wedge and column, centre for line).
"""

import math

LINE = 'line'
WEDGE = 'wedge'      # V formation
COLUMN = 'column'    # single file
CIRCLE = 'circle'
SHAPES = (LINE, WEDGE, COLUMN, CIRCLE)


def _axes(heading):
    """Forward and right unit vectors in NED (north, east)."""
    forward = (math.cos(heading), math.sin(heading))
    right = (-math.sin(heading), math.cos(heading))
    return forward, right


def slot_offsets(shape, n, spacing, heading=0.0, wedge_half_angle_deg=45.0):
    """Return n (dnorth, deast) offsets from the group centre for `shape`.

    spacing: metres between adjacent slots (radius for circle).
    """
    if n <= 0:
        return []
    if spacing <= 0.0:
        raise ValueError('spacing must be > 0')
    (fn, fe), (rn, re) = _axes(heading)

    def place(along, cross):
        """along=forward metres, cross=right metres -> (dnorth, deast)."""
        return (along * fn + cross * rn, along * fe + cross * re)

    if shape == LINE:
        # abreast, centred on the right (cross) axis
        return [place(0.0, (i - (n - 1) / 2.0) * spacing) for i in range(n)]

    if shape == COLUMN:
        # single file along forward axis; slot 0 in front
        return [place(((n - 1) / 2.0 - i) * spacing, 0.0) for i in range(n)]

    if shape == WEDGE:
        theta = math.radians(wedge_half_angle_deg)
        offs = [place(0.0, 0.0)]          # leader at the apex
        rank, side = 1, -1                # fill left, then right, widening
        for _ in range(1, n):
            offs.append(place(-rank * spacing * math.cos(theta),
                              side * rank * spacing * math.sin(theta)))
            if side == 1:
                rank += 1
            side *= -1
        return offs

    if shape == CIRCLE:
        radius = spacing
        return [(radius * math.cos(2 * math.pi * i / n + heading),
                 radius * math.sin(2 * math.pi * i / n + heading))
                for i in range(n)]

    raise ValueError(f'unknown formation {shape!r}; expected one of {SHAPES}')


def slot_positions(shape, n, center, spacing, heading=0.0, wedge_half_angle_deg=45.0):
    """Absolute (north, east) slots given center=(north, east)."""
    cn, ce = center
    return [(cn + dn, ce + de)
            for dn, de in slot_offsets(shape, n, spacing, heading, wedge_half_angle_deg)]
