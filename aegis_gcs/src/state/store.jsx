import React, {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useReducer,
  useRef,
} from 'react';
import { api } from '../lib/api.js';
import { connectRos } from '../lib/ros.js';
import { buildReport, classifyReport, isAbortIntent } from '../lib/reports.js';

/**
 * One store for the whole console.
 *
 * Reads  : /gcs/consolidated_telemetry (10 Hz over rosbridge) + /api/stack/status +
 *          /api/sim/config  ->  every panel renders from this single derived state.
 * Writes : the control API (start/stop stack, select world+mode+moving target, run mission).
 *
 * Nothing here is simulated. If a value is unknown it stays null/0 and the panel shows the
 * NO DATA/--- state rather than an invented number.
 */

const MAX_EVENTS = 400;
const MAX_CHAT = 200;
const PERF_POINTS = 60;

const initialState = {
  now: Date.now(),

  /** rosbridge link + control-API reachability */
  link: { status: 'connecting', lastRx: 0 },
  apiUp: false,

  /** live vehicle state, keyed by drone id (px4_1 ...), from telemetry_node */
  drones: {},
  droneOrder: [],
  selected: null,

  /** mission + stack state */
  mission: {
    name: 'NO MISSION',       // replaced by whatever the planning agent named the job
    state: 'NO MISSION',      // NO MISSION | DISPATCHED | EXECUTING | COMPLETE
    flightMode: '--',
    autonomy: 'SUPERVISED',
    startedAt: Date.now(),
  },
  stack: { services: [], running: false, busy: false },
  sim: {
    mode: 'single',
    world: 'empty',
    drones: 1,
    moving: 'none',
    agentic: false,   // decentralised swarm coordination (drone_agent auction)
    worlds: [],
    movings: [],
    models: [],
  },

  /** LLM command panel */
  chat: [
    {
      id: 'c0',
      t: Date.now(),
      role: 'SYSTEM',
      text: 'AEGIS console online. Select mission type and world, power on the stack, then issue commands in natural language.',
    },
  ],
  commandHistory: [],
  lastPlan: null,
  llmBusy: false,

  events: [],
  eventFilter: 'ALL',
  perf: { cpu: [], mem: [], link: [], posErr: [] },
  system: { cpu: null, mem: null },   // host load, measured by telemetry_node (all agents run here)
  video: { zoom: 1, view: 'auto', collapsed: false, recording: false, recStart: Date.now() },
  overlay: null,
  gpsDenied: false,      // GPS-denied operation (SLAM-only navigation + RTAB-Map view)
  targets: [],           // uploaded target images the LLM + agents use
};

function reducer(state, action) {
  switch (action.type) {
    case 'TICK':
      return { ...state, now: action.now };

    case 'LINK':
      return { ...state, link: { ...state.link, ...action.link } };

    case 'TELEMETRY': {
      // telemetry_node publishes { t, mission?, drones: { px4_1: {...}, ... } }
      const msg = action.msg || {};
      const incoming = msg.drones || {};
      const drones = { ...state.drones };
      for (const [id, d] of Object.entries(incoming)) drones[id] = { ...drones[id], ...d };
      const order = Object.keys(drones).sort();
      return {
        ...state,
        drones,
        droneOrder: order,
        selected: state.selected && drones[state.selected] ? state.selected : order[0] || null,
        mission: { ...state.mission, ...(msg.mission || {}) },
        system: { ...state.system, ...(msg.system || {}) },
        link: { ...state.link, lastRx: msg.t ? msg.t * 1000 : Date.now() },
      };
    }

    case 'STACK':
      return { ...state, stack: { ...state.stack, ...action.value }, apiUp: action.apiUp ?? state.apiUp };

    case 'SIM':
      return { ...state, sim: { ...state.sim, ...action.value } };

    case 'MISSION':
      return { ...state, mission: { ...state.mission, ...action.value } };

    case 'GPS_DENIED':
      return { ...state, gpsDenied: action.value };

    case 'TARGETS':
      return { ...state, targets: action.value };

    case 'EVENT':
      return { ...state, events: [action.event, ...state.events].slice(0, MAX_EVENTS) };

    case 'ACK':
      return {
        ...state,
        events: state.events.map((e) =>
          action.id === 'ALL' || e.id === action.id ? { ...e, ack: true } : e
        ),
      };

    case 'CHAT':
      return { ...state, chat: [...state.chat, action.entry].slice(-MAX_CHAT) };

    case 'HISTORY':
      return { ...state, commandHistory: [action.entry, ...state.commandHistory].slice(0, 200) };

    case 'PLAN':
      return { ...state, lastPlan: action.plan };

    case 'LLM_BUSY':
      return { ...state, llmBusy: action.value };

    case 'PERF': {
      const push = (arr, v) => [...arr, v].slice(-PERF_POINTS);
      return {
        ...state,
        perf: {
          cpu: push(state.perf.cpu, action.sample.cpu),
          mem: push(state.perf.mem, action.sample.mem),
          link: push(state.perf.link, action.sample.link),
          posErr: push(state.perf.posErr, action.sample.posErr),
        },
      };
    }

    case 'SELECT':
      return { ...state, selected: action.id };
    case 'FILTER':
      return { ...state, eventFilter: action.value };
    case 'OVERLAY':
      return { ...state, overlay: action.value };
    case 'VIDEO':
      return { ...state, video: { ...state.video, ...action.value } };

    default:
      return state;
  }
}

/** Derived state — computed once per render so no two panels can disagree. */
function derive(state) {
  const list = state.droneOrder.map((id) => ({ id, ...state.drones[id] }));
  const stale = state.link.status !== 'connected' || state.now - state.link.lastRx > 3000;
  const sel = state.selected ? { id: state.selected, ...state.drones[state.selected] } : null;

  const online = list.filter((d) => !stale && d.armed !== undefined);
  let safety = { state: list.length ? 'HEALTHY' : 'NO TELEMETRY', reason: '' };
  if (stale || !list.length) safety = { state: 'NO TELEMETRY', reason: 'telemetry link stale' };
  else {
    const crit = list.find((d) => (d.battery ?? 100) < 15);
    const caution = list.find((d) => (d.battery ?? 100) < 30 || (d.link_quality ?? 100) < 60);
    if (crit) safety = { state: 'CRITICAL', reason: `${crit.id} battery ${Math.round(crit.battery)}%` };
    else if (caution)
      safety = { state: 'CAUTION', reason: `${caution.id} battery/link degraded` };
  }

  const unacked = state.events.filter((e) => !e.ack && e.severity !== 'INFO');
  const isSwarm = state.sim.mode === 'swarm' || list.length > 1;

  return { ...state, list, stale, selectedDrone: sel, safety, unacked, online, isSwarm };
}

const StateCtx = createContext(null);
const ApiCtx = createContext(null);
export const useConsole = () => useContext(StateCtx);
export const useApi = () => useContext(ApiCtx);

export function ConsoleProvider({ children }) {
  const [raw, dispatch] = useReducer(reducer, initialState);
  const state = useMemo(() => derive(raw), [raw]);
  const stateRef = useRef(state);
  stateRef.current = state;

  const pushEvent = useCallback((source, severity, text) => {
    dispatch({
      type: 'EVENT',
      event: {
        id: `e${Date.now()}${Math.random().toString(36).slice(2, 6)}`,
        t: Date.now(),
        source,
        severity,
        text,
        ack: false,
      },
    });
  }, []);

  const chat = useCallback((role, text) => {
    dispatch({
      type: 'CHAT',
      entry: { id: `c${Date.now()}${Math.random().toString(36).slice(2, 6)}`, t: Date.now(), role, text },
    });
  }, []);

  // --- rosbridge telemetry + events ---
  useEffect(() => {
    const conn = connectRos({
      onState: (msg) => dispatch({ type: 'TELEMETRY', msg }),
      onEvent: (ev) =>
        dispatch({
          type: 'EVENT',
          event: { id: `r${Date.now()}${Math.random().toString(36).slice(2, 6)}`, ack: false, ...ev },
        }),
      onStatus: (l) => dispatch({ type: 'LINK', link: l }),
    });
    return () => conn.close();
  }, []);

  // --- control API polling: stack status + sim config ---
  useEffect(() => {
    let alive = true;
    const pollStack = async () => {
      try {
        const d = await api.stackStatus();
        if (!alive) return;
        dispatch({ type: 'STACK', value: { services: d.services || [], running: !!d.running }, apiUp: true });
        // autonomy + selectors are authoritative from the backend (derived from live state)
        if (d.autonomy) dispatch({ type: 'MISSION', value: { autonomy: d.autonomy } });
        dispatch({ type: 'SIM', value: {
          mode: d.mode ?? undefined, world: d.world ?? undefined, drones: d.drones ?? undefined,
          agentic: d.agentic ?? undefined, gps_denied: d.gps_denied ?? undefined } });
        if (Array.isArray(d.targets)) dispatch({ type: 'TARGETS', value: d.targets });

      } catch {
        if (alive) dispatch({ type: 'STACK', value: { running: false }, apiUp: false });
      }
    };
    const pollMission = async () => {
      // Mission name + progress, polled separately from the stack: the executor publishes no
      // status topic, so the backend derives these from its log. Kept out of pollStack's try
      // so a failure here can never be misread as "the stack is down".
      try {
        const m = await api.missionStatus();
        if (!alive) return;
        const was = stateRef.current.mission.state;
        dispatch({ type: 'MISSION', value: {
          name: m.mission_name || 'NO MISSION', state: m.mission_state || 'NO MISSION' } });
        if (m.mission_state === 'COMPLETE' && was !== 'COMPLETE') {
          dispatch({ type: 'EVENT', event: {
            id: `mc${Date.now()}`, t: Date.now(), source: 'COORDINATOR', severity: 'INFO',
            text: `MISSION COMPLETE — ${m.mission_name || 'mission'}`, ack: false } });
        }
      } catch {
        /* backend not reachable — the stack poller already reports that */
      }
    };
    const pollCfg = async () => {
      try {
        const d = await api.simConfig();
        if (!alive) return;
        dispatch({ type: 'SIM', value: d });
      } catch {
        /* API down — selectors keep their current values */
      }
    };
    pollStack();
    pollMission();
    pollCfg();
    const t1 = setInterval(pollStack, 2000);
    const tm = setInterval(pollMission, 2000);
    const t2 = setInterval(pollCfg, 10000);
    return () => {
      alive = false;
      clearInterval(t1);
      clearInterval(tm);
      clearInterval(t2);
    };
  }, []);

  // --- clock + performance sampling from real telemetry ---
  useEffect(() => {
    const clock = setInterval(() => dispatch({ type: 'TICK', now: Date.now() }), 1000);
    const perf = setInterval(() => {
      const s = stateRef.current;
      if (!s.list.length) return;
      const avg = (f) => s.list.reduce((t, d) => t + (f(d) || 0), 0) / s.list.length;
      dispatch({
        type: 'PERF',
        sample: {
          // measured host load; avg(d.cpu) read a field the backend never sent -> flat 0
          cpu: s.system.cpu ?? 0,
          mem: s.system.mem ?? 0,
          link: avg((d) => d.link_quality),
          posErr: Math.sqrt(avg((d) => (d.pos_err || 0) ** 2)),
        },
      });
    }, 1000);
    return () => {
      clearInterval(clock);
      clearInterval(perf);
    };
  }, []);

  const apiActions = useMemo(
    () => ({
      // ---- stack / sim power ----
      powerToggle: async () => {
        const s = stateRef.current;
        dispatch({ type: 'STACK', value: { busy: true } });
        try {
          if (s.stack.running) {
            pushEvent('SIM', 'WARNING', 'Powering off the simulation stack.');
            await api.stopStack();
          } else {
            pushEvent(
              'SIM',
              'INFO',
              `Starting ${s.sim.mode.toUpperCase()} mission — world ${s.sim.world}` +
                (s.sim.moving && s.sim.moving !== 'none' ? `, target ${s.sim.moving}` : '')
            );
            await api.startStack();
          }
        } catch (e) {
          pushEvent('SIM', 'ERROR', `Stack control failed: ${e.message}`);
        } finally {
          dispatch({ type: 'STACK', value: { busy: false } });
        }
      },

      setSim: async (patch) => {
        dispatch({ type: 'SIM', value: patch });
        try {
          const d = await api.setSimConfig(patch);
          if (d && d.ok !== false) dispatch({ type: 'SIM', value: d });
          else if (d && d.detail) pushEvent('SIM', 'WARNING', d.detail);
        } catch (e) {
          pushEvent('SIM', 'WARNING', `Could not apply selection: ${e.message}`);
        }
      },

      cleanup: async () => {
        try {
          await api.cleanup();
          pushEvent('SIM', 'INFO', 'Reaped orphan processes and cleared Gazebo scratch.');
        } catch (e) {
          pushEvent('SIM', 'WARNING', `Cleanup failed: ${e.message}`);
        }
      },

      // ---- reports: questions about state, answered from live telemetry ----
      report: (kind) => {
        const s = stateRef.current;
        chat('YOU', kind.toUpperCase());
        chat('AGENT', buildReport(kind, s));
        dispatch({
          type: 'HISTORY',
          entry: { id: `h${Date.now()}`, t: Date.now(), text: kind, status: 'ok', summary: `${kind} report` },
        });
      },

      // ---- LLM mission ----
      submitCommand: async (text) => {
        const trimmed = (text || '').trim();
        if (!trimmed) return;

        // "disengage / abort / stop following" is an immediate safety action, not a mission.
        // Sending it to the planner produced a flight plan that the live FOLLOW override
        // (priority 25) then ignored — so route it straight to the abort path.
        if (isAbortIntent(trimmed)) {
          chat('YOU', trimmed);
          try {
            const d = await api.abortMission();
            chat('AGENT', d.reply || 'Disengaged and returning to home.');
            pushEvent('SAFETY', 'WARNING', d.reply || 'Operator disengage — RTL issued.');
          } catch (e) {
            chat('AGENT', `Abort failed: ${e.message}`);
            pushEvent('SAFETY', 'ERROR', `Abort failed: ${e.message}`);
          }
          return;
        }

        // A question about current state is NOT a mission. Answer it here instead of asking
        // the planner to turn "report vehicle status" into a flight plan.
        const kind = classifyReport(trimmed);
        if (kind) {
          const s = stateRef.current;
          chat('YOU', trimmed);
          chat('AGENT', buildReport(kind, s));
          dispatch({
            type: 'HISTORY',
            entry: { id: `h${Date.now()}`, t: Date.now(), text: trimmed, status: 'ok', summary: `${kind} report` },
          });
          return;
        }

        chat('YOU', trimmed);
        dispatch({ type: 'LLM_BUSY', value: true });
        dispatch({
          type: 'HISTORY',
          entry: { id: `h${Date.now()}`, t: Date.now(), text: trimmed, status: 'pending', summary: 'planning…' },
        });
        try {
          const d = await api.runMission(trimmed);
          if (d.ok) {
            const summary = d.summary || 'plan accepted';
            chat('AGENT', d.reply || `Plan accepted: ${summary}`);
            if (d.plan) dispatch({ type: 'PLAN', plan: { plan: d.plan, unsupported: d.unsupported || [], t: Date.now() } });
            dispatch({
              type: 'HISTORY',
              entry: { id: `h${Date.now()}b`, t: Date.now(), text: trimmed, status: 'ok', summary },
            });
            pushEvent('LLM AGENT', 'INFO', `Mission planned: ${summary}`);
          } else {
            const why = d.detail || 'rejected by validation';
            chat('AGENT', `Command rejected: ${why}`);
            dispatch({
              type: 'HISTORY',
              entry: { id: `h${Date.now()}b`, t: Date.now(), text: trimmed, status: 'rejected', summary: why },
            });
            pushEvent('LLM AGENT', 'WARNING', `Command rejected: ${why}`);
          }
        } catch (e) {
          chat('AGENT', `Planner unreachable: ${e.message}`);
          pushEvent('LLM AGENT', 'ERROR', `Planner unreachable: ${e.message}`);
        } finally {
          dispatch({ type: 'LLM_BUSY', value: false });
        }
      },

      // Operator recall. Same disengage-then-RTL path as abort, but it is a normal command,
      // not an emergency, so it is reported as INFO rather than an alert.
      returnHome: async () => {
        try {
          const d = await api.returnHome();
          pushEvent('MISSION', 'INFO', d.reply || 'Return to home issued by operator.');
          chat('AGENT', d.reply || 'Returning to home.');
        } catch (e) {
          pushEvent('MISSION', 'ERROR', `Return home failed: ${e.message}`);
        }
      },

      abortMission: async () => {
        try {
          const d = await api.abortMission();
          pushEvent('SAFETY', 'ERROR', d.reply || 'Mission ABORT issued by operator.');
          chat('AGENT', d.reply || 'Disengaged and returning to home.');
        } catch (e) {
          pushEvent('SAFETY', 'ERROR', `Abort failed: ${e.message}`);
        }
      },

      // ---- GPS-denied operation ----
      setGpsDenied: async (on) => {
        dispatch({ type: 'GPS_DENIED', value: on });
        pushEvent('SLAM', 'INFO', on
          ? 'GPS-denied operation ENGAGED — navigating on SLAM.'
          : 'GPS-denied operation disengaged — GPS-aided navigation.');
        try { await api.setGpsDenied(on); } catch { /* view still switches locally */ }
      },

      // ---- target images: uploaded BEFORE a mission, used by the LLM + agents ----
      uploadTarget: async (file) => {
        if (!file) return;
        const dataURL = await new Promise((res, rej) => {
          const r = new FileReader();
          r.onload = () => res(r.result);
          r.onerror = rej;
          r.readAsDataURL(file);
        });
        chat('YOU', `Uploaded target image: ${file.name}`);
        try {
          const d = await api.uploadTarget(file.name, dataURL);
          if (d.ok) {
            dispatch({ type: 'TARGETS', value: d.targets || [] });
            chat('AGENT', d.reply || `Target registered: ${d.label}.`);
            pushEvent('LLM AGENT', 'INFO', `Target image registered as "${d.label}" (${Math.round((d.conf || 0) * 100)}%). Issue a mission to task the agents.`);
          } else {
            chat('AGENT', `Could not use that image: ${d.detail || 'unrecognised'}`);
          }
        } catch (e) {
          chat('AGENT', `Target upload failed: ${e.message}`);
        }
      },

      clearTargets: async () => {
        dispatch({ type: 'TARGETS', value: [] });
        try { await api.clearTargets(); } catch { /* ignore */ }
      },

      // ---- UI-local ----
      select: (id) => dispatch({ type: 'SELECT', id }),
      setFilter: (value) => dispatch({ type: 'FILTER', value }),
      setOverlay: (value) => dispatch({ type: 'OVERLAY', value }),
      setVideo: (value) => dispatch({ type: 'VIDEO', value }),
      ack: (id) => dispatch({ type: 'ACK', id }),
      chat,
      pushEvent,
    }),
    [chat, pushEvent]
  );

  return (
    <StateCtx.Provider value={state}>
      <ApiCtx.Provider value={apiActions}>{children}</ApiCtx.Provider>
    </StateCtx.Provider>
  );
}
