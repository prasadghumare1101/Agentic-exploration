import React from 'react';
import { compass } from '../lib/format.js';

/**
 * Real flight instrumentation, drawn from live attitude — not decoration.
 *
 *   X axis  COMPASS TAPE  : a sliding heading tape. Ticks every 5°, labels every 10°,
 *                           cardinals as letters, centred under a fixed index pointer.
 *                           The tape moves; the pointer does not — like a real HSI.
 *   Y axis  PITCH LADDER  : rungs every 10° of pitch, positioned by the same
 *                           degrees-per-pixel scale and rotated by bank angle, so the
 *                           horizon line sits where the real horizon is.
 *
 * Geometry is in a 0..100 viewBox so it scales with the panel at any size.
 */

const VIEW = 100;                 // viewBox units (square, stretched by preserveAspectRatio)
const DEG_PER_VIEW_X = 90;        // compass tape shows +/-45 deg
const PX_PER_DEG_X = VIEW / DEG_PER_VIEW_X;
const DEG_PER_VIEW_Y = 60;        // pitch ladder shows +/-30 deg
const PX_PER_DEG_Y = VIEW / DEG_PER_VIEW_Y;

/** shortest signed angular difference a-b, in (-180, 180] */
const wrap180 = (a, b) => {
  let d = ((a - b + 540) % 360) - 180;
  return d;
};

export function CompassTape({ heading = 0 }) {
  const ticks = [];
  const base = Math.round(heading / 5) * 5;
  for (let t = base - 50; t <= base + 50; t += 5) {
    const hdg = ((t % 360) + 360) % 360;
    const dx = wrap180(hdg, heading);
    if (Math.abs(dx) > 48) continue;
    const x = 50 + dx * PX_PER_DEG_X;
    const cardinal = hdg % 90 === 0;
    const major = hdg % 10 === 0;
    const label = cardinal
      ? ['N', 'E', 'S', 'W'][hdg / 90]
      : major
      ? String(hdg).padStart(3, '0')
      : null;
    ticks.push({ x, cardinal, major, label, key: t });
  }

  return (
    <svg className="absolute top-0 left-0 w-full h-9 pointer-events-none" viewBox={`0 0 ${VIEW} 20`} preserveAspectRatio="none">
      <line x1="0" y1="16" x2={VIEW} y2="16" stroke="rgba(226,232,240,0.35)" strokeWidth="0.3" vectorEffect="non-scaling-stroke" />
      {ticks.map((t) => (
        <g key={t.key}>
          <line
            x1={t.x} y1={t.cardinal ? 9 : t.major ? 11 : 13} x2={t.x} y2="16"
            stroke={t.cardinal ? '#F1F5F9' : 'rgba(226,232,240,0.7)'}
            strokeWidth={t.cardinal ? 0.5 : 0.3}
            vectorEffect="non-scaling-stroke"
          />
          {t.label && (
            <text
              x={t.x} y="7" textAnchor="middle"
              fontSize={t.cardinal ? 6 : 4.4}
              fill={t.cardinal ? '#F1F5F9' : 'rgba(226,232,240,0.8)'}
              fontFamily="ui-monospace, monospace"
              fontWeight={t.cardinal ? 700 : 400}
            >
              {t.label}
            </text>
          )}
        </g>
      ))}
      {/* fixed index pointer — the aircraft's own heading */}
      <polygon points="50,17.5 48.4,20 51.6,20" fill="#22D3EE" />
      <line x1="50" y1="9" x2="50" y2="16" stroke="#22D3EE" strokeWidth="0.6" vectorEffect="non-scaling-stroke" />
    </svg>
  );
}

export function PitchLadder({ pitch = 0, roll = 0 }) {
  const rungs = [];
  const base = Math.round(pitch / 10) * 10;
  for (let p = base - 40; p <= base + 40; p += 10) {
    if (p < -90 || p > 90) continue;
    const dy = (pitch - p) * PX_PER_DEG_Y;      // pitch up -> horizon moves down
    const y = 50 + dy;
    if (y < -10 || y > VIEW + 10) continue;
    rungs.push({ p, y });
  }

  return (
    <svg className="absolute inset-0 w-full h-full pointer-events-none" viewBox={`0 0 ${VIEW} ${VIEW}`} preserveAspectRatio="none">
      {/* the whole ladder banks with the airframe */}
      <g transform={`rotate(${-roll} 50 50)`}>
        {rungs.map(({ p, y }) => {
          const zero = p === 0;
          const half = zero ? 34 : 16;          // horizon line is the widest rung
          return (
            <g key={p}>
              <line
                x1={50 - half} y1={y} x2={50 - (zero ? 8 : 5)} y2={y}
                stroke={zero ? '#22D3EE' : 'rgba(226,232,240,0.75)'}
                strokeWidth={zero ? 0.7 : 0.4}
                strokeDasharray={p < 0 && !zero ? '2 1.5' : undefined}
                vectorEffect="non-scaling-stroke"
              />
              <line
                x1={50 + (zero ? 8 : 5)} y1={y} x2={50 + half} y2={y}
                stroke={zero ? '#22D3EE' : 'rgba(226,232,240,0.75)'}
                strokeWidth={zero ? 0.7 : 0.4}
                strokeDasharray={p < 0 && !zero ? '2 1.5' : undefined}
                vectorEffect="non-scaling-stroke"
              />
              {!zero && (
                <>
                  <text x={50 - half - 2} y={y + 1.4} textAnchor="end" fontSize="3.6"
                        fill="rgba(226,232,240,0.85)" fontFamily="ui-monospace, monospace">
                    {p > 0 ? `+${p}` : p}
                  </text>
                  <text x={50 + half + 2} y={y + 1.4} textAnchor="start" fontSize="3.6"
                        fill="rgba(226,232,240,0.85)" fontFamily="ui-monospace, monospace">
                    {p > 0 ? `+${p}` : p}
                  </text>
                </>
              )}
            </g>
          );
        })}
      </g>

      {/* fixed boresight: what the airframe is actually pointing at */}
      <g stroke="#22D3EE" strokeWidth="0.7" vectorEffect="non-scaling-stroke" fill="none">
        <line x1="42" y1="50" x2="47" y2="50" />
        <line x1="53" y1="50" x2="58" y2="50" />
        <circle cx="50" cy="50" r="1.1" />
      </g>
    </svg>
  );
}

/** Bank indicator: fixed scale at the top, pointer rolls with the airframe. */
export function BankIndicator({ roll = 0 }) {
  const marks = [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60];
  return (
    <svg className="absolute inset-0 w-full h-full pointer-events-none" viewBox="0 0 100 100" preserveAspectRatio="none">
      <g transform="translate(50 50)">
        {marks.map((m) => {
          const a = (m - 90) * (Math.PI / 180);
          const r1 = 30;
          const r2 = m % 30 === 0 ? 26 : 28;
          return (
            <line
              key={m}
              x1={Math.cos(a) * r1} y1={Math.sin(a) * r1}
              x2={Math.cos(a) * r2} y2={Math.sin(a) * r2}
              stroke="rgba(226,232,240,0.6)" strokeWidth="0.4" vectorEffect="non-scaling-stroke"
            />
          );
        })}
        {/* rolling pointer */}
        <g transform={`rotate(${roll})`}>
          <polygon points="0,-24 -1.6,-21 1.6,-21" fill="#22D3EE" />
        </g>
      </g>
    </svg>
  );
}

/** Numeric heading readout under the tape, e.g. 127° SE */
export function HeadingReadout({ heading = 0 }) {
  return (
    <div className="absolute top-9 left-1/2 -translate-x-1/2 px-1.5 py-0.5 bg-black/60 border border-[#22D3EE]/60 rounded text-[10px] font-mono text-[#F1F5F9] tnum pointer-events-none">
      {String(Math.round(((heading % 360) + 360) % 360)).padStart(3, '0')}° {compass(heading)}
    </div>
  );
}
