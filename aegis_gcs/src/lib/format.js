/** Presentation helpers. Values are formatted here so panels stay declarative. */

export const num = (v, d = 0) =>
  v === null || v === undefined || Number.isNaN(v) ? '--' : Number(v).toFixed(d);

export const pad2 = (n) => String(Math.floor(n)).padStart(2, '0');

/** 12:45:30Z — UTC clock used in the header. */
export const fmtClockZ = (t) => {
  const d = new Date(t);
  return `${pad2(d.getUTCHours())}:${pad2(d.getUTCMinutes())}:${pad2(d.getUTCSeconds())}Z`;
};

const MONTHS = ['JAN', 'FEB', 'MAR', 'APR', 'MAY', 'JUN', 'JUL', 'AUG', 'SEP', 'OCT', 'NOV', 'DEC'];
export const fmtDateUTC = (t) => {
  const d = new Date(t);
  return `${pad2(d.getUTCDate())} ${MONTHS[d.getUTCMonth()]} ${d.getUTCFullYear()}`;
};

/** 12:45:19 — local time-of-day for log/chat rows. */
export const fmtHMS = (t) => {
  const d = new Date(t);
  return `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`;
};

/** 18:42 — minutes:seconds of endurance remaining. */
export const fmtMinSec = (minutes) => {
  if (!minutes || minutes < 0 || !Number.isFinite(minutes)) return '--:--';
  return `${pad2(minutes)}:${pad2((minutes % 1) * 60)}`;
};

/** 01:24:35 — elapsed recording timer. */
export const fmtElapsed = (sec) => {
  if (!Number.isFinite(sec) || sec < 0) sec = 0;
  return `${pad2(sec / 3600)}:${pad2((sec / 60) % 60)}:${pad2(sec % 60)}`;
};

/** Compass point for a heading, e.g. 127 -> SE. */
export const compass = (deg) => {
  const pts = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];
  return pts[Math.round((((deg % 360) + 360) % 360) / 45) % 8];
};

export const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
