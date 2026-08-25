/**
 * Client for the AEGIS control API (aero_gcs `control_api.py`, FastAPI on :8000).
 *
 * Everything that CHANGES the system goes through here: starting/stopping the sim stack,
 * selecting world / mission mode / moving target, and running an LLM mission. Read-only live
 * state arrives separately over rosbridge (see lib/ros.js), because that is a 10 Hz stream and
 * does not belong on HTTP.
 *
 * Host is derived from the page so the console works served from the backend or a dev server,
 * on localhost or over the LAN.
 */

const HOST =
  typeof window !== 'undefined' && window.location && window.location.hostname
    ? window.location.hostname
    : 'localhost';

export const API_BASE = `http://${HOST}:8000`;
export const VIDEO_BASE = `http://${HOST}:8080`;
export const ROSBRIDGE_URL = `ws://${HOST}:9090`;

async function req(path, options) {
  const r = await fetch(`${API_BASE}${path}`, options);
  if (!r.ok) throw new Error(`${path} -> HTTP ${r.status}`);
  const text = await r.text();
  return text ? JSON.parse(text) : {};
}

const post = (path) => req(path, { method: 'POST' });
const get = (path) => req(path);

export const api = {
  /** Per-service state of the stack (agent, sitl, rosbridge, telemetry, video). */
  stackStatus: () => get('/api/stack/status'),
  /** Bring the whole simulation stack up / down. This is what the power button calls. */
  startStack: () => post('/api/stack/start'),
  stopStack: () => post('/api/stack/stop'),
  /** Reap orphan gzserver/px4 and clear Gazebo scratch (never touches protected paths). */
  cleanup: () => post('/api/stack/cleanup'),

  /** Available worlds / airframes / moving targets + the current selection. */
  simConfig: () => get('/api/sim/config'),
  /** Select mission mode (single|swarm), world, and moving target. Applies on next start. */
  setSimConfig: ({ mode, world, drones, moving }) => {
    const q = new URLSearchParams();
    if (mode !== undefined) q.set('mode', mode);
    if (world !== undefined) q.set('world', world);
    if (drones !== undefined) q.set('drones', String(drones));
    if (moving !== undefined) q.set('moving', moving);
    return post(`/api/sim/config?${q.toString()}`);
  },

  /** Upload a target image. Classified now, used by the LLM + agents at mission time. */
  uploadTarget: (name, dataURL) =>
    req('/api/mission/target', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, dataURL }),
    }),
  clearTargets: () => post('/api/mission/target/clear'),

  /** GPS-denied operation: navigate on SLAM only. */
  setGpsDenied: (on) => post(`/api/sim/gps_denied?enabled=${on ? 'true' : 'false'}`),

  /** Natural-language mission -> LLM planner -> validator -> execution. */
  runMission: (prompt) => post(`/api/mission/run?prompt=${encodeURIComponent(prompt)}`),
  missionStatus: () => get('/api/mission/status'),
  abortMission: () => post('/api/mission/abort'),
  returnHome: () => post('/api/mission/rtl'),

  /** MJPEG camera feed. view: 'auto' | 'raw' | 'annotated'. */
  videoUrl: (drone = 'px4_1', view = 'auto') =>
    `${VIDEO_BASE}/video_feed?drone=${encodeURIComponent(drone)}&view=${encodeURIComponent(view)}`,
};
