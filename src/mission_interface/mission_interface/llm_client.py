"""LLM planner — composes a MissionPlan from primitives.

This is the ONLY place the real Hugging Face token is used. It is loaded from
`src/mission_interface/.env` via python-dotenv and read with os.getenv — the
token is never hardcoded here.

The planner DECOMPOSES an arbitrary prompt into a sequence of phases built from
the primitive set, rather than matching it against a fixed list of missions. If
something genuinely cannot be expressed (no camera, no tracked target), it must
say so in `unsupported` — never silently substitute a different mission.

Public contract: `plan(prompt) -> (plan_dict, unsupported_list)`.
"""

import json
import os
from pathlib import Path

from dotenv import load_dotenv
from huggingface_hub import InferenceClient

from mission_interface import schema


def _find_env_file() -> Path:
    here = Path(__file__).resolve()
    candidates = [here.parents[1] / '.env']
    for parent in here.parents:
        candidates.append(parent / 'src' / 'mission_interface' / '.env')
    for c in candidates:
        if c.is_file():
            return c
    raise FileNotFoundError(
        'Missing env file src/mission_interface/.env. Create it from the sample:\n'
        '  cp src/mission_interface/.env.example src/mission_interface/.env\n'
        'then set your real HF_TOKEN inside it.\nSearched:\n  ' +
        '\n  '.join(str(c) for c in candidates))


_MISSION_DOCTRINE = """You are an autonomous AI mission-commander for a small drone swarm \
— a reliable teammate whose job is to understand the operator's objective, choose the best \
strategy, and decompose it into a SAFE, EXECUTABLE flight plan. Prioritise mission success, \
accuracy, safety, efficiency and teamwork.

The swarm executes PHYSICAL missions. Map the operator's mission type to what the drones can \
actually do, and honestly list anything they cannot:
  Expedition / Discovery      -> explore & map an area: survey (+ spiral for outward search), \
goto for legs, rtl to return.
  Survey / Analysis(physical) -> structured area coverage: one survey with the stated area + \
spacing.
  Monitoring                  -> watch / patrol: orbit a point, or a formation patrol + hold.
  Rescue / Investigation      -> respond & search: survey or spiral over the area, goto to a \
point of interest, hold on station.
  Search & Track / Hunt       -> find and pursue an object: a "survey" (or "spiral") over the \
search area PLUS a "follow" with {"target":"<object>"} - two separate entries, never merged \
into one.
  Coordination                -> divide the work across drones with per-drone "assignments".
  Cognitive types (Research, pure data Analysis, strategy Planning, Innovation) have NO flight \
action — plan any achievable flight part and put the cognitive request in "unsupported"; never \
fake it with an unrelated mission.

Always: break a complex objective into ordered phases; pick the smallest primitive set that \
achieves it; keep drones separated and inside the geofence / altitude band; prefer declarative \
survey/orbit/spiral over hand-written legs; scope every phase when specific drones are named; \
adapt to the stated conditions; and report limits honestly in "unsupported".

"""

_SYSTEM_PROMPT = _MISSION_DOCTRINE + """You plan missions for a small drone swarm. Decompose the \
operator's command into a JSON mission plan. Output ONLY JSON, no prose, no fences.

Available drones: %(drones)s

PRIMITIVES (the only actions the drones can perform):
  takeoff   params: {"altitude": m}
  goto      params: {"target": [x, y, z]}        x=East, y=North, z=altitude (m)
  formation params: {"shape": "line|wedge|column|circle", "spacing": m,
                     "heading": deg (0=North), "anchor": [x, y, z]}
  survey    params: {"center": [x, y], "width": m, "height": m, "spacing": m,
                     "altitude": m, "heading": deg, "shape": "wedge",
                     "formation_spacing": m}
            Covers an AREA with a lawnmower pattern, flown in formation.
  orbit     params: {"center": [x, y], "radius": m, "altitude": m, "turns": n,
                     "points_per_turn": n, "shape": "circle", "formation_spacing": m}
            Circles a point. Use for "orbit/circle around X".
  spiral    params: {"center": [x, y], "start_radius": m, "growth": m, "altitude": m,
                     "turns": n, "points_per_turn": n, "shape": "circle"}
            Outward spiral search. Use for "spiral out from X".

Any phase may add "speed": m/s (max %(max_speed).0f) to set cruise speed.
  hold      params: {}
  wait      params: {"duration": s}
  land      params: {}
  rtl       params: {}

PLAN FORMAT:
{"phases": [
   {"name": "...", "primitive": "<one of above>", "params": {...},
    "drones": ["px4_1"],            // optional, default = all drones
    "advance_when": "all_arrived",  // or "timeout"
    "timeout": 30},
   ...
 ],
 "unsupported": ["..."]   // things you could NOT express with the primitives
}

For per-drone differences use "assignments" instead of "primitive":
   {"name":"split","assignments":[
      {"drone":"px4_2","primitive":"goto","params":{"target":[0,20,4]}},
      {"drone":"px4_3","primitive":"goto","params":{"target":[20,0,4]}}],
    "timeout":45}

RULES:
- Break multi-step commands into multiple phases, in order.
- SWEEPING / SEARCHING / COVERING AN AREA: use ONE "survey" phase. Do NOT hand-
  write the individual legs — state the area and spacing and the executor
  computes the lawnmower geometry deterministically and guarantees coverage.
    e.g. "sweep the area in a wedge":
      {"name":"sweep","primitive":"survey",
       "params":{"center":[0,0],"width":40,"height":40,"spacing":10,
                 "altitude":4,"heading":0,"shape":"wedge","formation_spacing":8},
       "timeout":120}
- MOVING IN FORMATION between named points: emit one "formation" phase per
  waypoint, same shape/spacing, moving the "anchor" (set "heading" to the
  direction of travel). Never use a group "goto" for drones that must hold a
  shape — a shared goto sends every drone to the SAME point and collapses it.
- Use a plain "goto" phase only when the drones do NOT need to hold a shape.
- Regroup = a formation phase.
- Formation spacing must be at least %(min_spacing).1f m.
- SPECIFIC DRONES: if the operator names particular drones, you MUST scope every
  affected phase — either "assignments" (different actions per drone) or
  "drones": ["px4_N"] (same action, subset). NEVER emit a bare group phase when
  specific drones were named: a phase without "drones"/"assignments" commands ALL
  drones and would move the wrong ones.
- "stay put" / "hold position" IS supported — it is the "hold" primitive. Never
  report it as unsupported.
- SIMULTANEOUS WORK = ONE PHASE. Phases run STRICTLY IN SEQUENCE, so three separate
  survey phases make drones wait for each other. When drones must act AT THE SAME
  TIME, emit a SINGLE phase whose "assignments" list gives each drone its own
  primitive and params. Never split concurrent work into one phase per drone.
- DURATIONS: any stated time ("hold for 30 seconds", "wait 10s") MUST become a
  "wait" phase with {"duration": seconds}. Do not drop it.

EXAMPLE — specific drones (note every phase is scoped):
{"phases":[
  {"name":"split","assignments":[
     {"drone":"px4_1","primitive":"goto","params":{"target":[0,30,4]}},
     {"drone":"px4_2","primitive":"hold","params":{}},
     {"drone":"px4_3","primitive":"land","params":{}}],
   "timeout":60}],
 "unsupported":[]}

EXAMPLE — three drones hunting DIFFERENT targets AT THE SAME TIME (one phase):
{"phases":[
  {"name":"takeoff","primitive":"takeoff","params":{"altitude":5},"timeout":40},
  {"name":"search_and_track","assignments":[
     {"drone":"px4_1","primitive":"follow",
      "params":{"target":"person","center":[0,80],"width":120,"height":120,"spacing":5}},
     {"drone":"px4_2","primitive":"follow",
      "params":{"target":"car","center":[80,0],"width":120,"height":120,"spacing":5}},
     {"drone":"px4_3","primitive":"follow",
      "params":{"target":"truck","center":[-80,0],"width":120,"height":120,"spacing":5}}],
   "timeout":300}],
 "unsupported":[]}

EXAMPLE — duration honoured:
{"phases":[
  {"name":"circle","primitive":"formation",
   "params":{"shape":"circle","spacing":8,"heading":0,"anchor":[0,0,8]},"timeout":40},
  {"name":"hold_30s","primitive":"wait","params":{"duration":30},"timeout":35}],
 "unsupported":[]}
- Altitude must be between %(min_alt).1f and %(max_alt).1f m; keep positions within \
%(fence).0f m of the origin.
- FOLLOW / TRACK IS SUPPORTED. Use the "follow" primitive with {"target": "<object>"} \
(e.g. person, car, truck) and NO other params. The flight stack HOLDS STATION under \
"follow" until perception acquires the object, so a "follow" carrying an area covers \
nothing and the drone never finds anything. When the command names a search area, give \
that drone a SEPARATE "survey" entry with the area params: it flies the survey and \
switches to pursuit the instant the object is detected. NEVER report following/tracking \
as unsupported.
- If the command needs a capability the system genuinely lacks (filming for \
broadcast, gripping, physically inspecting the inside of an object), still plan the \
achievable flight part AND list only that missing capability in "unsupported".
- Never invent a different mission to replace something you cannot do."""


class LLMClient:
    """Turn any natural-language command into a validated-shape mission plan."""

    def __init__(self, default_altitude=3.0, drones=None):
        self.default_altitude = float(default_altitude)
        self.drones = list(drones) if drones else ['px4_1']

        load_dotenv(dotenv_path=_find_env_file(), override=True)
        self.token = os.getenv('HF_TOKEN')
        self.model_primary = os.getenv('HF_MODEL_ID_PRIMARY')
        self.model_secondary = os.getenv('HF_MODEL_ID_SECONDARY')

        if not self.token or self.token.startswith('hf_xxx'):
            raise RuntimeError(
                'HF_TOKEN is missing or still the placeholder. Set a real token '
                'in src/mission_interface/.env (never commit it).')
        if not self.model_primary:
            raise RuntimeError('HF_MODEL_ID_PRIMARY is not set in src/mission_interface/.env')

        self._models = [m for m in (self.model_primary, self.model_secondary) if m]
        self._client = InferenceClient(token=self.token)
        self._system = _SYSTEM_PROMPT % {
            'drones': ', '.join(self.drones),
            'min_alt': schema.MIN_ALT, 'max_alt': schema.MAX_ALT,
            'fence': schema.GEOFENCE_RADIUS, 'min_spacing': schema.MIN_SPACING,
            'max_speed': schema.MAX_CRUISE_SPEED,
        }

    # -- public contract ------------------------------------------------------
    def plan(self, prompt: str):
        """NL command -> (plan dict, unsupported list). Tries primary then secondary."""
        errors = []
        for model in self._models:
            try:
                obj = self._extract_json(self._chat(model, prompt))
                unsupported = obj.pop('unsupported', []) or []
                if isinstance(unsupported, str):
                    unsupported = [unsupported]
                return obj, list(unsupported)
            except Exception as e:
                errors.append(f'{model}: {e}')
        raise RuntimeError('All Hugging Face models failed to produce a plan:\n  ' +
                           '\n  '.join(errors))

    def repair(self, prompt: str, plan: dict, errors: list):
        """Ask the planner to FIX a plan that failed validation, rather than giving
        up. Returns (plan dict, unsupported list). One shot; caller re-validates."""
        req = (f'Your plan for the command "{prompt}" failed validation:\n'
               + '\n'.join(f'- {e}' for e in errors)
               + '\n\nHere is the plan you produced:\n' + json.dumps(plan)
               + '\n\nReturn a CORRECTED plan as JSON only. Fix exactly these errors '
                 '(usually a field in the wrong place or a value outside its limit). '
                 'Keep the intent and all other phases identical.')
        last = None
        for model in self._models:
            try:
                obj = self._extract_json(self._chat(model, req))
                unsupported = obj.pop('unsupported', []) or []
                if isinstance(unsupported, str):
                    unsupported = [unsupported]
                return obj, list(unsupported)
            except Exception as e:
                last = e
        raise RuntimeError(f'repair failed: {last}')

    # -- internals ------------------------------------------------------------
    def _chat(self, model, prompt):
        resp = self._client.chat_completion(
            model=model,
            messages=[{'role': 'system', 'content': self._system},
                      {'role': 'user', 'content': prompt}],
            max_tokens=900,
            temperature=0.0,
        )
        return resp.choices[0].message.content

    @staticmethod
    def _extract_json(text: str) -> dict:
        start, end = text.find('{'), text.rfind('}')
        if start == -1 or end == -1 or end < start:
            raise ValueError(f'no JSON object in model reply: {text[:120]!r}')
        return json.loads(text[start:end + 1])
