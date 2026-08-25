"""Plan validation — STRUCTURE + SAFETY BOUNDS only.

Deliberately not an intent whitelist. It does not ask "is this a known mission?";
it asks "is this plan well-formed, and are its numbers inside safe limits?".
Anything expressible from the primitives is allowed through; safety is then
enforced continuously in flight, not by refusing missions at the origin.

Pure functions, no ROS, easy to unit-test.
"""

import json
from pathlib import Path

from mission_interface import schema

# Formal draft-07 contract: STRUCTURE, types and the primitive whitelist. The
# numeric safety limits (geofence, altitude band, expansion caps) stay in Python
# below — they are cross-field or depend on expansion, which JSON Schema cannot
# express — and are re-enforced in flight by drone_controller.
_SCHEMA_PATH = Path(__file__).with_name('mission_plan.schema.json')
try:
    import jsonschema
    _JSON_SCHEMA = json.loads(_SCHEMA_PATH.read_text())
    _VALIDATOR = jsonschema.Draft7Validator(_JSON_SCHEMA)
except Exception:                      # jsonschema absent -> Python checks still apply
    _VALIDATOR = None


_ALL_PARAM_KEYS = set().union(*schema.PARAM_KEYS.values())


def normalize_plan(plan: dict) -> dict:
    """Repair trivial, SAFE structural variance from the planner before validating.

    LLMs place valid fields in slightly-wrong spots (e.g. `speed` as a phase-level
    key instead of inside `params`). Rejecting the whole mission over that is
    brittle and dangerous in the field. So we RELOCATE a known param key that
    appears at phase level into `params` — the value still gets bounds-checked,
    nothing is invented or dropped. Genuine typos (`speed_mps`) are NOT known keys
    and are left to be rejected, so a safety field can never be silently ignored.
    """
    if not isinstance(plan, dict):
        return plan
    for ph in plan.get('phases', []) or []:
        if not isinstance(ph, dict):
            continue
        _lift(ph, schema.PHASE_KEYS)
        for a in ph.get('assignments') or []:
            if isinstance(a, dict):
                _lift(a, {'drone', 'primitive', 'params'})
    return plan


def clamp_plan(plan: dict):
    """Make an in-bounds plan out of an out-of-bounds one, IN PLACE.

    An out-of-range value (altitude 500, speed 50, spacing 0.5, area too big) is
    CLAMPED to its safe limit and the mission flies the safe version — it is not
    thrown away. Returns a list of human-readable adjustments made. This is the
    'degrade, don't reject' rule: no valid intent is ever lost to a number.
    """
    adj = []

    def cl(where, params, key, lo, hi):
        v = params.get(key)
        if isinstance(v, (int, float)):
            c = min(max(v, lo), hi)
            if c != v:
                params[key] = c
                adj.append(f'{where}.{key}: {v} -> {c}')

    def clamp_xyz(where, params, key):
        v = params.get(key)
        if isinstance(v, (list, tuple)) and len(v) == 3 and all(isinstance(x, (int, float)) for x in v):
            x, y, z = v
            r = (x * x + y * y) ** 0.5
            if r > schema.GEOFENCE_RADIUS:
                f = schema.GEOFENCE_RADIUS / r
                x, y = x * f, y * f
                adj.append(f'{where}.{key}: pulled inside geofence')
            z2 = min(max(z, schema.MIN_ALT), schema.MAX_ALT)
            if z2 != z:
                adj.append(f'{where}.{key} altitude: {z} -> {z2}')
            params[key] = [x, y, z2]

    def clamp_node(where, primitive, params):
        if not isinstance(params, dict):
            return
        cl(where, params, 'speed', 0.1, schema.MAX_CRUISE_SPEED)
        if primitive == 'takeoff':
            cl(where, params, 'altitude', schema.MIN_ALT, schema.MAX_ALT)
        elif primitive in ('goto', 'navigate'):
            clamp_xyz(where, params, 'target')
        elif primitive == 'formation':
            cl(where, params, 'spacing', schema.MIN_SPACING, schema.MAX_SPACING)
            clamp_xyz(where, params, 'anchor')
        elif primitive == 'survey':
            cl(where, params, 'altitude', schema.MIN_ALT, schema.MAX_ALT)
            cl(where, params, 'width', 1.0, schema.MAX_AREA_SIDE)
            cl(where, params, 'height', 1.0, schema.MAX_AREA_SIDE)
            cl(where, params, 'formation_spacing', schema.MIN_SPACING, schema.MAX_SPACING)
            # spacing floor, AND raise it if the area would expand past the leg cap
            cl(where, params, 'spacing', schema.MIN_PATTERN_SPACING, schema.MAX_AREA_SIDE)
            h, sp = params.get('height'), params.get('spacing')
            if isinstance(h, (int, float)) and isinstance(sp, (int, float)) and sp > 0:
                max_lanes = max(1, schema.MAX_SURVEY_LEGS // 2 - 1)
                need = h / max_lanes
                if sp < need:
                    params['spacing'] = round(need, 2)
                    adj.append(f'{where}.spacing: raised to {params["spacing"]} to cap legs')
        elif primitive in ('orbit', 'spiral'):
            cl(where, params, 'altitude', schema.MIN_ALT, schema.MAX_ALT)
            cl(where, params, 'turns', 1, schema.MAX_PATTERN_TURNS)
            cl(where, params, 'points_per_turn', 3, schema.MAX_POINTS_PER_TURN)
            cl(where, params, 'radius', 1.0, schema.MAX_ORBIT_RADIUS)
            cl(where, params, 'start_radius', 1.0, schema.MAX_ORBIT_RADIUS)
            cl(where, params, 'growth', schema.MIN_PATTERN_SPACING, schema.MAX_ORBIT_RADIUS)
        elif primitive == 'wait':
            cl(where, params, 'duration', 0.0, schema.MAX_PHASE_TIMEOUT)

    for i, ph in enumerate(plan.get('phases', []) or []):
        if not isinstance(ph, dict):
            continue
        w = f'phase[{i}]'
        if 'timeout' in ph:
            cl(w, ph, 'timeout', 1.0, schema.MAX_PHASE_TIMEOUT)
        clamp_node(w, ph.get('primitive'), ph.get('params') or {})
        for j, a in enumerate(ph.get('assignments') or []):
            if isinstance(a, dict):
                clamp_node(f'{w}.assign[{j}]', a.get('primitive'), a.get('params') or {})
    return adj


def _lift(node: dict, top_keys) -> None:
    stray = [k for k in list(node)
             if k not in top_keys and k in _ALL_PARAM_KEYS]
    if stray:
        params = node.setdefault('params', {})
        if isinstance(params, dict):
            for k in stray:
                params.setdefault(k, node.pop(k))


def validate_structure(plan: dict):
    """Draft-07 structural pass. Returns list of error strings (empty = OK)."""
    if _VALIDATOR is None:
        return []
    out = []
    for e in sorted(_VALIDATOR.iter_errors(plan), key=lambda e: list(e.path)):
        loc = '/'.join(str(p) for p in e.path) or 'plan'
        out.append(f'{loc}: {e.message}')
    return out[:10]


def _check_xyz(where, xyz, errors):
    """Validate an [x, y, z] target: shape, altitude band, geofence."""
    if not (isinstance(xyz, (list, tuple)) and len(xyz) == 3):
        errors.append(f'{where}: target must be [x, y, z]')
        return
    if not all(isinstance(v, (int, float)) for v in xyz):
        errors.append(f'{where}: target values must be numbers')
        return
    x, y, z = xyz
    if not (schema.MIN_ALT <= z <= schema.MAX_ALT):
        errors.append(f'{where}: altitude {z} outside [{schema.MIN_ALT}, {schema.MAX_ALT}]')
    if (x * x + y * y) ** 0.5 > schema.GEOFENCE_RADIUS:
        errors.append(f'{where}: target outside geofence radius {schema.GEOFENCE_RADIUS} m')


def _check_alt(where, alt, errors):
    if alt is None:
        return
    if not isinstance(alt, (int, float)):
        errors.append(f'{where}: altitude must be a number')
    elif not (schema.MIN_ALT <= alt <= schema.MAX_ALT):
        errors.append(f'{where}: altitude {alt} outside [{schema.MIN_ALT}, {schema.MAX_ALT}]')


def _check_primitive(where, primitive, params, errors):
    """Bounds-check one primitive's parameters. Unknown primitive is an error
    here (the planner should have reported it as `unsupported` instead)."""
    if primitive not in schema.PRIMITIVES:
        errors.append(f'{where}: unknown primitive "{primitive}"; '
                      f'expected one of {schema.PRIMITIVES}')
        return
    params = params or {}

    # Reject unknown keys: a misspelled safety field would otherwise be silently
    # ignored and skip its bounds check (e.g. "speed_mps" bypassing the speed cap).
    unknown = set(params) - schema.PARAM_KEYS.get(primitive, set())
    if unknown:
        errors.append(f'{where}: unknown param(s) {sorted(unknown)} for "{primitive}"; '
                      f'allowed: {sorted(schema.PARAM_KEYS.get(primitive, set()))}')

    sp = params.get('speed')
    if sp is not None and (not isinstance(sp, (int, float))
                           or not (0 < sp <= schema.MAX_CRUISE_SPEED)):
        errors.append(f'{where}: speed must be in (0, {schema.MAX_CRUISE_SPEED}] m/s')

    if primitive == 'takeoff':
        _check_alt(where, params.get('altitude'), errors)
    elif primitive in ('goto', 'navigate'):
        if 'target' not in params:
            errors.append(f'{where}: {primitive} requires "target"')
        else:
            _check_xyz(where, params['target'], errors)
    elif primitive == 'formation':
        shape = params.get('shape', 'wedge')
        if shape not in schema.FORMATION_SHAPES:
            errors.append(f'{where}: shape "{shape}" not in {schema.FORMATION_SHAPES}')
        spacing = params.get('spacing', 5.0)
        if not isinstance(spacing, (int, float)) or \
                not (schema.MIN_SPACING <= spacing <= schema.MAX_SPACING):
            errors.append(
                f'{where}: spacing must be in [{schema.MIN_SPACING}, {schema.MAX_SPACING}] '
                f'(below {schema.MIN_SPACING} m would trip the separation guard)')
        if 'anchor' in params:
            _check_xyz(where + '.anchor', params['anchor'], errors)
    elif primitive == 'survey':
        for key in ('center', 'width', 'height', 'spacing', 'altitude'):
            if key not in params:
                errors.append(f'{where}: survey requires "{key}"')
        c = params.get('center')
        if not (isinstance(c, (list, tuple)) and len(c) == 2
                and all(isinstance(v, (int, float)) for v in c)):
            errors.append(f'{where}: survey center must be [x, y]')
        _check_alt(where, params.get('altitude'), errors)
        for key in ('width', 'height'):
            v = params.get(key)
            if not isinstance(v, (int, float)) or not (0 < v <= schema.MAX_AREA_SIDE):
                errors.append(f'{where}: {key} must be in (0, {schema.MAX_AREA_SIDE}]')
        sp = params.get('spacing')
        if not isinstance(sp, (int, float)) or sp < schema.MIN_PATTERN_SPACING:
            errors.append(f'{where}: survey spacing must be >= {schema.MIN_PATTERN_SPACING}')
        # cap the expansion so one prompt cannot generate thousands of waypoints
        h, sp2 = params.get('height'), params.get('spacing')
        if isinstance(h, (int, float)) and isinstance(sp2, (int, float)) and sp2 > 0:
            legs = 2 * (int(h / sp2) + 1)
            if legs > schema.MAX_SURVEY_LEGS:
                errors.append(f'{where}: survey expands to {legs} legs '
                              f'(max {schema.MAX_SURVEY_LEGS}); increase spacing')
        if 'shape' in params and params['shape'] not in schema.FORMATION_SHAPES:
            errors.append(f'{where}: shape "{params["shape"]}" not in {schema.FORMATION_SHAPES}')
    elif primitive in ('orbit', 'spiral'):
        c = params.get('center')
        if not (isinstance(c, (list, tuple)) and len(c) == 2
                and all(isinstance(v, (int, float)) for v in c)):
            errors.append(f'{where}: {primitive} center must be [x, y]')
        _check_alt(where, params.get('altitude'), errors)
        turns = params.get('turns', 1)
        if not isinstance(turns, int) or not (1 <= turns <= schema.MAX_PATTERN_TURNS):
            errors.append(f'{where}: turns must be 1..{schema.MAX_PATTERN_TURNS}')
        ppt = params.get('points_per_turn', 12)
        if not isinstance(ppt, int) or not (3 <= ppt <= schema.MAX_POINTS_PER_TURN):
            errors.append(f'{where}: points_per_turn must be 3..{schema.MAX_POINTS_PER_TURN}')
        if primitive == 'orbit':
            r = params.get('radius')
            if not isinstance(r, (int, float)) or not (0 < r <= schema.MAX_ORBIT_RADIUS):
                errors.append(f'{where}: radius must be in (0, {schema.MAX_ORBIT_RADIUS}]')
        else:
            g = params.get('growth', 5.0)
            if not isinstance(g, (int, float)) or g < schema.MIN_PATTERN_SPACING:
                errors.append(f'{where}: growth must be >= {schema.MIN_PATTERN_SPACING}')
        if 'shape' in params and params['shape'] not in schema.FORMATION_SHAPES:
            errors.append(f'{where}: shape "{params["shape"]}" not in {schema.FORMATION_SHAPES}')
    elif primitive == 'wait':
        d = params.get('duration', 0.0)
        if not isinstance(d, (int, float)) or d < 0 or d > schema.MAX_PHASE_TIMEOUT:
            errors.append(f'{where}: wait duration must be in [0, {schema.MAX_PHASE_TIMEOUT}]')
    # hold / land / rtl take no bounded params


def validate_plan(plan: dict):
    """Validate a mission plan. Returns (ok: bool, errors: list[str])."""
    errors = []

    if not isinstance(plan, dict):
        return False, ['plan must be an object']

    # Pass 1: formal structure/types/whitelist (draft-07). Bail early — the
    # numeric checks below assume well-formed shapes.
    structural = validate_structure(plan)
    if structural:
        return False, structural

    phases = plan.get('phases')
    if not isinstance(phases, list) or not phases:
        return False, ['plan must contain a non-empty "phases" list']

    stray_plan = set(plan) - schema.PLAN_KEYS
    if stray_plan:
        errors.append(f'plan: unknown key(s) {sorted(stray_plan)}')

    for i, ph in enumerate(phases):
        where = f'phase[{i}]'
        if not isinstance(ph, dict):
            errors.append(f'{where}: must be an object')
            continue

        stray = set(ph) - schema.PHASE_KEYS
        if stray:
            errors.append(f'{where}: unknown key(s) {sorted(stray)}; '
                          f'allowed: {sorted(schema.PHASE_KEYS)}')

        assignments = ph.get('assignments')
        has_group = 'primitive' in ph
        if not has_group and not assignments:
            errors.append(f'{where}: needs "primitive" or "assignments"')

        if has_group:
            _check_primitive(where, ph.get('primitive'), ph.get('params'), errors)

        if assignments is not None:
            if not isinstance(assignments, list):
                errors.append(f'{where}: "assignments" must be a list')
            else:
                for j, a in enumerate(assignments):
                    aw = f'{where}.assignments[{j}]'
                    if not isinstance(a, dict) or 'drone' not in a:
                        errors.append(f'{aw}: needs "drone" and "primitive"')
                        continue
                    _check_primitive(aw, a.get('primitive'), a.get('params'), errors)

        mode = ph.get('advance_when', 'all_arrived')
        if mode not in schema.ADVANCE_MODES:
            errors.append(f'{where}: advance_when must be one of {schema.ADVANCE_MODES}')
        t = ph.get('timeout', 30.0)
        if not isinstance(t, (int, float)) or not (0 < t <= schema.MAX_PHASE_TIMEOUT):
            errors.append(f'{where}: timeout must be in (0, {schema.MAX_PHASE_TIMEOUT}]')

    total = _expanded_waypoints(phases)
    if total > schema.MAX_TOTAL_WAYPOINTS:
        errors.append(f'plan expands to {total} waypoints '
                      f'(max {schema.MAX_TOTAL_WAYPOINTS}); widen spacing or reduce turns')

    return (len(errors) == 0), errors


def _expanded_waypoints(phases) -> int:
    """Waypoint count AFTER declarative patterns are expanded (blow-up guard)."""
    total = 0
    for ph in phases:
        if not isinstance(ph, dict):
            continue
        prim, p = ph.get('primitive'), (ph.get('params') or {})
        if prim == 'survey':
            h, sp = p.get('height'), p.get('spacing')
            total += 2 * (int(h / sp) + 1) if _pos(h) and _pos(sp) else 1
        elif prim in ('orbit', 'spiral'):
            total += int(p.get('turns', 1)) * int(p.get('points_per_turn', 12))
        else:
            total += 1
    return total


def _pos(v):
    return isinstance(v, (int, float)) and v > 0
