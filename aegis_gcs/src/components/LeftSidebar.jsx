import React, { useMemo } from 'react';
import { useApi, useConsole } from '../state/store.jsx';
import { Panel, PanelHeader, Dot } from './common.jsx';
import { num, fmtMinSec, compass } from '../lib/format.js';

/** ---------------- DRONE STATUS (live PX4 telemetry) ---------------- */
export function DroneStatus() {
  const s = useConsole();
  const d = s.selectedDrone;
  const has = !!d && !s.stale;

  const battPct = has ? d.battery ?? 0 : 0;
  const battTone = battPct < 15 ? 'bg-red-500' : battPct < 30 ? 'bg-[#F97316]' : 'bg-[#4ADE80]';

  const rows = [
    { k: 'GPS', ok: has && d.gps_ok, val: has ? `${d.gps_fix || (d.gps_ok ? 'FIX' : 'NO FIX')}` : 'NO DATA',
      extra: has && d.eph !== undefined ? `(${num(d.eph, 2)} m)` : '' },
    { k: 'IMU', ok: has, val: has ? 'HEALTHY' : 'NO DATA' },
    { k: 'COMPASS', ok: has && d.heading_ok !== false, val: has ? (d.heading_ok === false ? 'ALIGNING' : 'HEALTHY') : 'NO DATA' },
    { k: 'RC LINK', ok: has && (d.link_quality ?? 0) > 60, val: has ? (d.link_quality > 70 ? 'STRONG' : d.link_quality > 45 ? 'FAIR' : 'WEAK') : 'NO LINK',
      extra: has ? `(${num(d.link_quality, 0)}%)` : '' },
  ];

  return (
    <Panel>
      <PanelHeader
        title={`DRONE STATUS${d ? ` · ${d.id.toUpperCase()}` : ''}`}
        right={
          <span className={`text-[10px] font-bold tracking-widest ${has ? 'text-[#22D3EE]' : 'text-red-400'}`}>
            {has ? (d.armed ? 'ARMED' : 'ONLINE') : 'NO DATA'}
          </span>
        }
      />
      <div className="p-4 flex flex-col gap-5">
        {/* Battery */}
        <div className="flex flex-col gap-1.5">
          <span className="text-[10px] text-[#94A3B8]">BATTERY</span>
          <div className="flex items-center gap-3">
            <div className="w-16 h-6 border border-slate-500 rounded-sm p-[1.5px] relative">
              <div className={`${battTone} h-full transition-all`} style={{ width: `${Math.max(0, Math.min(100, battPct))}%` }}></div>
              <div className="absolute top-1/2 -right-[3px] -translate-y-1/2 w-[2px] h-2.5 bg-slate-500"></div>
            </div>
            <span className="text-xl font-normal text-[#F1F5F9] tracking-normal tnum">{num(battPct, 0)}%</span>
            <div className="ml-auto flex flex-col items-end text-[10px]">
              <div className="flex justify-between w-20"><span className="text-[#94A3B8]">Voltage</span><span className="text-slate-200 tnum">{num(has ? d.battery_voltage : null, 1)} V</span></div>
              <div className="flex justify-between w-20"><span className="text-[#94A3B8]">Est. Time</span><span className="text-slate-200 tracking-normal tnum">{has ? fmtMinSec(d.battery_minutes) : '--:--'}</span></div>
            </div>
          </div>
        </div>

        <div className="h-px bg-slate-800"></div>

        {/* Flight telemetry */}
        <div className="grid grid-cols-2 gap-y-4">
          <div>
            <div className="text-[10px] text-[#94A3B8] mb-0.5">ALTITUDE (AMSL)</div>
            <div className="text-lg text-[#F1F5F9] font-normal tracking-normal lowercase"><span className="text-xl tnum">{num(has ? d.altitude : null, 0)}</span> m</div>
          </div>
          <div>
            <div className="text-[10px] text-[#94A3B8] mb-0.5">SPEED (GND)</div>
            <div className="text-lg text-[#F1F5F9] font-normal tracking-normal lowercase"><span className="text-xl tnum">{num(has ? d.speed : null, 0)}</span> km/h</div>
          </div>
          <div>
            <div className="text-[10px] text-[#94A3B8] mb-0.5">VERTICAL SPEED</div>
            <div className="text-lg text-[#F1F5F9] font-normal tracking-normal lowercase"><span className="text-xl tnum">{num(has ? d.climb_rate : null, 1)}</span> m/s</div>
          </div>
          <div>
            <div className="text-[10px] text-[#94A3B8] mb-0.5">HEADING</div>
            <div className="text-lg text-[#F1F5F9] font-normal tracking-normal uppercase"><span className="text-xl tnum">{num(has ? d.heading : null, 0)}°</span> {has ? compass(d.heading) : ''}</div>
          </div>
        </div>

        <div className="h-px bg-slate-800"></div>

        {/* System status */}
        <div className="flex flex-col gap-2">
          <div className="text-[10px] text-[#94A3B8] mb-1">SYSTEM STATUS</div>
          {rows.map((r) => (
            <div key={r.k} className="flex justify-between items-center text-[11px]">
              <div className="flex items-center gap-2">
                <Dot tone={r.ok ? 'ok' : has ? 'warn' : 'idle'} />
                <span className="text-slate-300">{r.k}</span>
              </div>
              <span className={r.ok ? 'text-[#4ADE80]' : 'text-slate-300'}>
                {r.val} {r.extra && <span className="text-[#94A3B8] lowercase">{r.extra}</span>}
              </span>
            </div>
          ))}
        </div>
      </div>
    </Panel>
  );
}

/** ---------------- SLAM / LOCALIZATION (real RTAB-Map) ---------------- */

/** One RTAB-Map view: real pose graph, keyframes and accepted loop closures. */
function RtabMap({ d, compact }) {
  const slam = d?.slam;
  const geom = useMemo(() => {
    const traj = slam?.traj || [];
    const kf = slam?.keyframes || [];
    const loops = slam?.loops_pts || [];
    if (traj.length < 2) return null;
    const xs = traj.map((p) => p[0]);
    const ys = traj.map((p) => p[1]);
    const minX = Math.min(...xs), maxX = Math.max(...xs);
    const minY = Math.min(...ys), maxY = Math.max(...ys);
    const span = Math.max(maxX - minX, maxY - minY, 1);
    const pad = 16;
    const sc = (200 - pad * 2) / span;
    // ENU -> screen: x east to the right, y north UP (invert screen y)
    const px = (p) => [pad + (p[0] - minX) * sc, 200 - pad - (p[1] - minY) * sc];
    return {
      path: traj.map(px).map(([x, y], i) => `${i ? 'L' : 'M'} ${x.toFixed(1)} ${y.toFixed(1)}`).join(' '),
      kf: kf.map(px),
      loops: loops.map(px),
      cur: px(traj[traj.length - 1]),
    };
  }, [slam]);

  return (
    <div className={`relative bg-[#090C12] rounded border border-slate-700/60 overflow-hidden ${compact ? 'h-[92px]' : 'flex-1 min-h-0'}`}>
      <div
        className="absolute inset-0 opacity-20"
        style={{ backgroundSize: '4px 4px', backgroundImage: 'radial-gradient(circle, #475569 1px, transparent 1px)' }}
      />
      {geom ? (
        <svg className="absolute inset-0 w-full h-full" viewBox="0 0 200 200" preserveAspectRatio="xMidYMid meet">
          <path d={geom.path} fill="none" stroke="#4ADE80" strokeWidth="2" vectorEffect="non-scaling-stroke" />
          {geom.kf.map(([x, y], i) => <circle key={`k${i}`} cx={x} cy={y} r="1.5" fill="#22D3EE" />)}
          {geom.loops.map(([x, y], i) => (
            <g key={`l${i}`}>
              <circle cx={x} cy={y} r="4" fill="none" stroke="#F97316" strokeWidth="1.5" vectorEffect="non-scaling-stroke" />
              <circle cx={x} cy={y} r="1.5" fill="#F97316" />
            </g>
          ))}
          <circle cx={geom.cur[0]} cy={geom.cur[1]} r="3" fill="#F1F5F9" />
        </svg>
      ) : (
        <div className="absolute inset-0 flex items-center justify-center text-[9px] text-[#94A3B8] tracking-wider">
          {slam ? 'BUILDING MAP…' : 'NO SLAM DATA'}
        </div>
      )}

      {/* per-map header: which drone, and whether this is real RTAB-Map or odometry */}
      <div className="absolute top-1 left-1.5 right-1.5 flex justify-between items-center text-[8px] tracking-widest pointer-events-none">
        <span className="text-[#F1F5F9] font-semibold">{d.id.toUpperCase()}</span>
        <span className={slam?.source === 'RTABMAP' ? 'text-[#22D3EE]' : 'text-[#94A3B8]'}>{slam?.source || '--'}</span>
      </div>
      <div className="absolute bottom-1 left-1.5 right-1.5 flex justify-between text-[8px] font-mono tnum text-[#94A3B8] pointer-events-none">
        <span>LOOPS {slam?.loops ?? 0}</span>
        <span>LOC {num(slam?.loc_quality, 0)}%</span>
      </div>
    </div>
  );
}

export function SlamHealth() {
  const s = useConsole();
  const api = useApi();
  // backend flag is authoritative once the stack is up (it decides if rtabmap launched)
  const gpsDenied = s.sim?.gps_denied ?? s.gpsDenied;
  const list = s.list;
  const sel = s.selectedDrone;
  const slam = sel?.slam;
  const live = !!slam && !s.stale;
  const locQ = live ? slam.loc_quality ?? 0 : 0;
  const mapQ = live ? slam.map_quality ?? 0 : 0;
  const healthy = live && locQ > 60;

  return (
    <Panel className="flex-1 overflow-hidden">
      <PanelHeader
        title="SLAM / LOCALIZATION HEALTH"
        right={
          <div className="flex items-center gap-2">
            {/* GPS-DENIED OPERATION: when ON the vehicle is navigating on SLAM alone, and
                the RTAB-Map view is shown here for every active drone. */}
            <button
              onClick={() => api.setGpsDenied(!gpsDenied)}
              title={gpsDenied ? 'GPS-denied ON — navigating on SLAM; RTAB-Map shown' : 'GPS-denied OFF — GPS-aided navigation'}
              className={`flex items-center gap-1 px-1.5 py-0.5 rounded border text-[8px] font-semibold tracking-widest transition-colors ${
                gpsDenied
                  ? 'border-[#22D3EE] text-[#22D3EE] bg-cyan-950/30'
                  : 'border-slate-700 text-[#94A3B8] hover:text-[#F1F5F9]'
              }`}
            >
              <span className={`w-1.5 h-1.5 rounded-full ${gpsDenied ? 'bg-[#22D3EE]' : 'bg-slate-600'}`} />
              GPS-DENIED {gpsDenied ? 'ON' : 'OFF'}
            </button>
            <span
              className={`px-1.5 py-0.5 rounded text-[9px] font-bold tracking-widest border ${
                healthy
                  ? 'text-[#22D3EE] border-cyan-800 bg-cyan-950/30'
                  : live
                  ? 'text-[#F97316] border-orange-800 bg-orange-950/30'
                  : 'text-[#94A3B8] border-slate-700 bg-slate-900/30'
              }`}
            >
              {live ? (healthy ? 'HEALTHY' : 'DEGRADED') : 'NO MAP'}
            </span>
          </div>
        }
      />

      <div className="p-4 flex flex-col gap-3 flex-1 min-h-0">
        <div className="flex flex-col gap-2">
          <div>
            <div className="flex justify-between text-[10px] mb-1">
              <span className="text-[#94A3B8]">LOCALIZATION QUALITY</span>
              <span className="text-[#F1F5F9] font-mono tnum">{num(locQ, 0)}%</span>
            </div>
            <div className="h-1 bg-slate-800 rounded-full overflow-hidden">
              <div className="h-full bg-[#22D3EE] transition-all" style={{ width: `${locQ}%` }} />
            </div>
          </div>
          <div>
            <div className="flex justify-between text-[10px] mb-1">
              <span className="text-[#94A3B8]">MAP QUALITY</span>
              <span className="text-[#F1F5F9] font-mono tnum">{num(mapQ, 0)}%</span>
            </div>
            <div className="h-1 bg-slate-800 rounded-full overflow-hidden">
              <div className="h-full bg-[#22D3EE] opacity-70 transition-all" style={{ width: `${mapQ}%` }} />
            </div>
          </div>
        </div>

        <div className="grid grid-cols-2 text-[11px]">
          <div className="flex flex-col">
            <span className="text-[#94A3B8]">LOOP CLOSURES</span>
            <span className="text-[#F1F5F9] font-mono tnum text-sm mt-0.5">{live ? slam.loops ?? 0 : '--'}</span>
          </div>
          <div className="flex flex-col">
            <span className="text-[#94A3B8]">TRACKED FEATURES</span>
            <span className="text-[#F1F5F9] font-mono tnum text-sm mt-0.5">
              {live ? (slam.features ?? 0).toLocaleString() : '--'}
            </span>
          </div>
        </div>

        {/* RTAB-Map only while GPS-denied operation is engaged.
            Single drone -> one full-height map. Swarm -> a scrolling column, one compact map
            per drone, so several maps never overlap or fight for space. */}
        {gpsDenied ? (
          list.length === 0 ? (
            <div className="flex-1 flex items-center justify-center text-[10px] text-[#94A3B8] tracking-wider border border-slate-700/60 rounded">
              NO VEHICLES REPORTING
            </div>
          ) : list.length === 1 ? (
            <RtabMap d={list[0]} />
          ) : (
            <div className="flex-1 min-h-0 overflow-y-auto flex flex-col gap-2 pr-0.5">
              {list.map((d) => <RtabMap key={d.id} d={d} compact />)}
            </div>
          )
        ) : (
          <div className="flex-1 min-h-0 flex flex-col items-center justify-center gap-1 border border-dashed border-slate-700/60 rounded text-center px-3">
            <span className="text-[10px] text-[#94A3B8] tracking-wider">GPS-AIDED NAVIGATION</span>
            <span className="text-[9px] text-[#94A3B8] normal-case tracking-normal">
              Turn on GPS-DENIED to navigate on SLAM and show the RTAB-Map.
            </span>
          </div>
        )}

        <div className="flex-none flex flex-wrap gap-x-3 gap-y-1 text-[9px] text-[#94A3B8] normal-case tracking-normal">
          <span className="flex items-center gap-1.5"><i className="w-3 h-0.5 bg-[#4ADE80] inline-block" /> Trajectory</span>
          <span className="flex items-center gap-1.5"><i className="w-1.5 h-1.5 rounded-full bg-[#22D3EE] inline-block" /> Keyframes</span>
          <span className="flex items-center gap-1.5"><i className="w-1.5 h-1.5 rounded-full bg-[#F97316] inline-block" /> Loop closure</span>
        </div>
      </div>
    </Panel>
  );
}
