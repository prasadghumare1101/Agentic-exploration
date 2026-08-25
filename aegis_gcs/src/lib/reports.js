/**
 * Operator reports computed from LIVE state.
 *
 * These are QUESTIONS, not missions. Sending "report vehicle status" to the mission planner
 * produced a nonsense flight plan ("report_status", marked unsupported), so the four report
 * actions are answered here from the telemetry the console already holds — instantly, with
 * no round trip, and no chance of commanding the vehicle by accident.
 *
 * Every number below comes from the consolidated telemetry stream. Nothing is invented: if a
 * value is missing the report says so.
 */
import { num, fmtMinSec, compass } from './format.js';

/** Text the operator typed that is a question about state, not an order to fly. */
const REPORT_PATTERNS = [
  [/\b(status|sitrep|state of|how are|health)\b/, 'status'],
  [/\b(summar|recap|overview of the mission)\b/, 'summary'],
  [/\b(explain|why|what is it doing|what are you doing)\b/, 'explain'],
  [/\b(suggest|recommend|advise|what should)\b/, 'suggest'],
  [/^(help|\?|commands)$/, 'help'],
];

/** Explicit disengage/abort phrasing. These must never reach the planner: they are an
 *  immediate safety action, not a mission to be flown. */
const ABORT_RE =
  /\b(disengage|abort|stop (following|tracking|the mission)|break lock|release (the )?(target|lock)|emergency)\b/;

export const isAbortIntent = (text) => ABORT_RE.test((text || '').trim().toLowerCase());

/** Returns 'status' | 'summary' | 'explain' | 'suggest' | 'help' | null. */
export function classifyReport(text) {
  const t = (text || '').trim().toLowerCase();
  if (!t) return null;
  for (const [re, kind] of REPORT_PATTERNS) if (re.test(t)) return kind;
  return null;
}

const line = (k, v) => `${k.padEnd(18)} ${v}`;

function droneLines(d, stale) {
  return [
    line('  state', stale ? 'NO TELEMETRY' : d.armed ? 'ARMED' : 'ONLINE'),
    line('  flight mode', d.flight_mode || '--'),
    line('  battery', `${num(d.battery, 0)}%  (${num(d.battery_voltage, 1)} V, ${fmtMinSec(d.battery_minutes)} left)`),
    line('  altitude', `${num(d.altitude, 1)} m`),
    line('  speed', `${num(d.speed, 1)} km/h   vs ${num(d.climb_rate, 1)} m/s`),
    line('  heading', `${num(d.heading, 0)}° ${compass(d.heading)}`),
    line('  position', `${num(d.lat, 6)}, ${num(d.lon, 6)}`),
    line('  gps', `${d.gps_fix || '--'}  ${d.satellites ?? 0} sats  eph ${num(d.eph, 2)} m`),
    line('  link', `${num(d.link_quality, 0)}%`),
    line('  follow', d.follow_state || 'IDLE'),
    line('  slam', `${d.slam?.source || '--'}  loc ${num(d.slam?.loc_quality, 0)}%  loops ${d.slam?.loops ?? 0}`),
  ].join('\n');
}

export function buildReport(kind, s) {
  const list = s.list || [];
  const stale = s.stale;
  const mode = s.sim?.mode === 'swarm' ? 'SWARM' : 'SINGLE';

  if (kind === 'help') {
    return [
      'You can speak plainly — anything that describes a flight is planned by the LLM,',
      'validated against safety limits, and dispatched to the drones. For example:',
      '',
      '  "take off to 20 m and survey a 100 m circle, then return home"',
      '  "px4_1 follows the person, px4_2 holds overwatch at 30 m"',
      '',
      'These four are questions about the current state, answered instantly:',
      '  status    — full vehicle readout',
      '  summarize — mission summary',
      '  explain   — what the system is doing and why',
      '  suggest   — recommended next action',
      '',
      'Upload a target image to have the agents acquire and follow that object.',
    ].join('\n');
  }

  if (!list.length) {
    return 'No vehicle telemetry. Power on the stack (top-right) and the drones will report in.';
  }

  if (kind === 'status') {
    const head = `${mode} mission · ${list.length} vehicle(s) · sim ${s.stack?.running ? 'UP' : 'OFF'}`;
    return [head, ...list.map((d) => `\n${d.id.toUpperCase()}\n${droneLines(d, stale)}`)].join('\n');
  }

  if (kind === 'summary') {
    const armed = list.filter((d) => d.armed).length;
    const tracking = list.filter((d) => ['DETECT', 'LOCK', 'LOCKED', 'FOLLOW'].includes(d.follow_state));
    const lowest = list.reduce((a, b) => ((a.battery ?? 100) < (b.battery ?? 100) ? a : b));
    const loops = list.reduce((t, d) => t + (d.slam?.loops || 0), 0);
    return [
      `Mission: ${s.mission?.name || 'AEGIS'} (${mode}), world ${s.sim?.world || '--'}.`,
      `Vehicles: ${list.length} reporting, ${armed} armed.`,
      `Tracking: ${tracking.length ? tracking.map((d) => `${d.id} ${d.follow_state}`).join(', ') : 'no target locked.'}`,
      `Battery: lowest is ${lowest.id} at ${num(lowest.battery, 0)}%.`,
      `SLAM: ${loops} loop closure(s) accepted across the fleet.`,
      `Safety: ${s.safety?.state}${s.safety?.reason ? ` — ${s.safety.reason}` : ''}.`,
      `Events logged: ${s.events?.length || 0}.`,
    ].join('\n');
  }

  if (kind === 'explain') {
    const d = s.selectedDrone || list[0];
    const follow = d.follow_state || 'IDLE';
    const what =
      follow === 'FOLLOW'
        ? 'pursuing a locked target using image-space visual servoing (yaw from the bounding-box offset, forward from its apparent size).'
        : follow === 'SEARCH'
        ? 'scanning for the commanded target class; nothing is locked yet.'
        : follow === 'DETECT' || follow === 'LOCK'
        ? 'has a candidate in frame and is building enough consecutive hits to commit to a lock.'
        : 'holding station. No target class is armed, so the detector is deliberately publishing nothing.';
    return [
      `${d.id.toUpperCase()} is ${what}`,
      '',
      `Flight mode is ${d.flight_mode || '--'}; autonomy is ${s.mission?.autonomy || 'SUPERVISED'}, so plans are`,
      'validated against altitude/geofence limits before any of them reach a vehicle.',
      `Localisation is ${d.slam?.source || 'unavailable'} at ${num(d.slam?.loc_quality, 0)}% quality`,
      `(${d.slam?.loops ?? 0} loop closures), which is what the SLAM panel is drawing.`,
    ].join('\n');
  }

  if (kind === 'suggest') {
    const out = [];
    if (!s.stack?.running) out.push('• Power on the stack — no simulation is running.');
    const low = list.filter((d) => (d.battery ?? 100) < 30);
    if (low.length) out.push(`• Bring ${low.map((d) => d.id).join(', ')} home: battery under 30%.`);
    const weak = list.filter((d) => (d.link_quality ?? 100) < 60);
    if (weak.length) out.push(`• Close the range on ${weak.map((d) => d.id).join(', ')}: link is degraded.`);
    const badLoc = list.filter((d) => (d.slam?.loc_quality ?? 100) < 60);
    if (badLoc.length) out.push(`• ${badLoc.map((d) => d.id).join(', ')} has weak localisation — reduce speed or altitude to regain features.`);
    if (!list.some((d) => ['DETECT', 'LOCK', 'LOCKED', 'FOLLOW'].includes(d.follow_state)))
      out.push('• No target is locked. Upload a target image, then issue the follow mission.');
    const idle = list.filter((d) => !d.armed);
    if (idle.length && s.stack?.running) out.push(`• ${idle.map((d) => d.id).join(', ')} are disarmed — issue a takeoff to bring them up.`);
    return out.length ? out.join('\n') : 'All vehicles nominal. No action required.';
  }

  return 'Unknown report.';
}
