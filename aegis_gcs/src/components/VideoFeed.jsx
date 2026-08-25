import React, { useMemo } from 'react';
import { useApi, useConsole } from '../state/store.jsx';
import { api as backend } from '../lib/api.js';
import { CompassTape, PitchLadder, BankIndicator, HeadingReadout } from './Hud.jsx';
import { num, fmtElapsed, clamp } from '../lib/format.js';

export default function VideoFeed() {
  const s = useConsole();
  const api = useApi();
  const d = s.selectedDrone;
  const live = !!d && !s.stale;
  const det = d?.detection;
  const follow = d?.follow_state || 'IDLE';
  const dist = d?.target_distance;   // metres to the tracked object; null when unlocked

  // Only rebuild the stream URL when the drone or view changes — otherwise the <img>
  // would be torn down on every telemetry tick, restarting the MJPEG connection.
  const src = useMemo(() => (d ? backend.videoUrl(d.id, s.video.view) : null), [d?.id, s.video.view]);

  const zoom = s.video.zoom;
  const box = det
    ? {
        left: `${clamp(det.cx - det.w / 2, 0, 100)}%`,
        top: `${clamp(det.cy - det.h / 2, 0, 100)}%`,
        width: `${clamp(det.w, 1, 100)}%`,
        height: `${clamp(det.h, 1, 100)}%`,
      }
    : null;

  if (s.video.collapsed) {
    return (
      <div className="flex-[1.6] bg-[#111722] border border-slate-700/60 rounded flex flex-col overflow-hidden">
        <div className="flex justify-between items-center px-4 py-2 border-b border-slate-800">
          <h2 className="text-[11px] text-[#F1F5F9] font-semibold tracking-wider">
            LIVE VIDEO FEED{d ? ` · ${d.id.toUpperCase()}` : ''}
          </h2>
          <button onClick={() => api.setVideo({ collapsed: false })} title="Expand feed" className="text-[#94A3B8] hover:text-[#F1F5F9]">
            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><polyline points="18 15 12 9 6 15"></polyline></svg>
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className="flex-[1.6] bg-[#111722] border border-slate-700/60 rounded flex flex-col overflow-hidden relative min-w-0">
      <div className="flex justify-between items-center gap-2 px-4 py-2 border-b border-slate-800 z-10 bg-[#111722]">
        <h2 className="text-[11px] text-[#F1F5F9] font-semibold tracking-wider flex items-center gap-2 shrink-0">
          LIVE VIDEO FEED
          <span className="text-[#94A3B8] font-normal tracking-normal text-[10px] normal-case">
            {live ? `${s.video.view} · MJPEG` : 'no stream'}
          </span>
        </h2>

        <div className="flex items-center gap-2 overflow-x-auto">
          {/* Per-drone feed switching. Every vehicle is selectable here, and picking one in
              the overview panel switches this feed too — both use the same selection. */}
          {s.list.length > 1 &&
            s.list.map((a) => (
              <button
                key={a.id}
                onClick={() => api.select(a.id)}
                title={`Show ${a.id} camera`}
                className={`text-[9px] px-1.5 py-0.5 rounded border shrink-0 ${
                  a.id === s.selected
                    ? 'border-[#22D3EE] text-[#22D3EE] bg-cyan-950/30'
                    : 'border-slate-700 text-[#94A3B8] hover:text-[#F1F5F9]'
                }`}
              >
                {a.id.toUpperCase()}
              </button>
            ))}

          {['auto', 'raw', 'annotated'].map((v) => (
            <button
              key={v}
              onClick={() => api.setVideo({ view: v })}
              className={`text-[9px] px-1.5 py-0.5 rounded border shrink-0 ${
                s.video.view === v
                  ? 'border-[#22D3EE] text-[#22D3EE] bg-cyan-950/30'
                  : 'border-slate-700 text-[#94A3B8] hover:text-[#F1F5F9]'
              }`}
              title={v === 'auto' ? 'Annotated while tracking, raw otherwise' : `Force ${v} feed`}
            >
              {v.toUpperCase()}
            </button>
          ))}
          <button onClick={() => api.setVideo({ collapsed: true })} title="Collapse feed" className="text-[#94A3B8] hover:text-[#F1F5F9] shrink-0">
            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><polyline points="6 9 12 15 18 9"></polyline></svg>
          </button>
        </div>
      </div>

      <div className="flex-1 relative bg-[#0b0d10] overflow-hidden">
        {src ? (
          <img
            src={src}
            alt="camera"
            className="absolute inset-0 w-full h-full object-cover origin-center transition-transform"
            style={{ transform: `scale(${zoom})` }}
          />
        ) : (
          <div className="absolute inset-0 flex items-center justify-center text-[#94A3B8] text-[11px] tracking-wider">
            NO CAMERA — POWER ON THE SIMULATION
          </div>
        )}

        {/* ---- real flight instrumentation, driven by live attitude ---- */}
        <PitchLadder pitch={live ? d.pitch || 0 : 0} roll={live ? d.roll || 0 : 0} />
        <BankIndicator roll={live ? d.roll || 0 : 0} />
        <CompassTape heading={live ? d.heading || 0 : 0} />
        <HeadingReadout heading={live ? d.heading || 0 : 0} />

        {/* speed / altitude tapes flanking the horizon, as on a real HUD */}
        <div className="absolute left-3 top-1/2 -translate-y-1/2 bg-black/50 border border-white/10 rounded px-1.5 py-1 pointer-events-none">
          <div className="text-[7px] text-[#94A3B8] tracking-widest">SPD km/h</div>
          <div className="text-sm font-mono text-[#F1F5F9] tnum">{live ? num(d.speed, 0) : '--'}</div>
        </div>
        <div className="absolute right-3 top-1/2 -translate-y-1/2 bg-black/50 border border-white/10 rounded px-1.5 py-1 text-right pointer-events-none">
          <div className="text-[7px] text-[#94A3B8] tracking-widest">ALT m</div>
          <div className="text-sm font-mono text-[#F1F5F9] tnum">{live ? num(d.altitude, 0) : '--'}</div>
          <div className="text-[8px] font-mono text-[#94A3B8] tnum">{live ? `${num(d.climb_rate, 1)} m/s` : ''}</div>
        </div>

        {/* zoom */}
        <div className="absolute bottom-16 right-3 flex flex-col bg-black/40 border border-white/10 rounded overflow-hidden">
          <button title="Zoom in" onClick={() => api.setVideo({ zoom: clamp(zoom + 0.5, 1, 6) })} className="p-1.5 hover:bg-white/10 text-white/80">
            <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><line x1="12" y1="5" x2="12" y2="19"></line><line x1="5" y1="12" x2="19" y2="12"></line></svg>
          </button>
          <div className="px-1.5 py-0.5 text-[9px] font-mono text-center border-y border-white/10 text-white tnum">{zoom.toFixed(1)}x</div>
          <button title="Zoom out" onClick={() => api.setVideo({ zoom: clamp(zoom - 0.5, 1, 6) })} className="p-1.5 hover:bg-white/10 text-white/80">
            <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><line x1="5" y1="12" x2="19" y2="12"></line></svg>
          </button>
        </div>

        {/* target box — real detector bbox */}
        {box && (
          <div className="absolute border-[1.5px] border-[#22D3EE] bg-cyan-400/5 pointer-events-none" style={box}>
            <div className="absolute -top-1 -left-1 w-2 h-2 border-t-[1.5px] border-l-[1.5px] border-[#22D3EE]"></div>
            <div className="absolute -top-1 -right-1 w-2 h-2 border-t-[1.5px] border-r-[1.5px] border-[#22D3EE]"></div>
            <div className="absolute -bottom-1 -left-1 w-2 h-2 border-b-[1.5px] border-l-[1.5px] border-[#22D3EE]"></div>
            <div className="absolute -bottom-1 -right-1 w-2 h-2 border-b-[1.5px] border-r-[1.5px] border-[#22D3EE]"></div>
            <div className="absolute -top-7 right-0 bg-cyan-900/80 border border-[#22D3EE] text-cyan-50 px-1.5 py-0.5 text-[9px] font-mono whitespace-nowrap backdrop-blur-sm">
              <div>TGT: {det.label || 'TARGET'}#{det.id ?? '--'}</div>
              <div>CONF: {num((det.conf || 0) * 100, 0)}%</div>
              {/* Range to this object, from target_follow's pinhole estimate. Only present
                  while the follower holds a lock, so '--' means no range, not zero range. */}
              <div>DIST: {dist == null ? '--' : `${num(dist, 1)} m`}</div>
            </div>
            <div className="absolute -bottom-5 right-0 bg-[#22D3EE] text-black px-1.5 py-0.5 text-[9px] font-bold tracking-wider whitespace-nowrap">
              {follow}
            </div>
          </div>
        )}

        {/* bottom telemetry bar */}
        <div className="absolute bottom-0 w-full bg-black/60 backdrop-blur-md border-t border-white/10 px-4 py-2 flex justify-between items-center text-[10px]">
          <div className="flex gap-5">
            {[
              ['LAT', live ? num(d.lat, 6) : '--'],
              ['LON', live ? num(d.lon, 6) : '--'],
              ['ALT', live ? `${num(d.altitude, 1)} m` : '--'],
              ['HDG', live ? `${num(d.heading, 0)}°` : '--'],
              ['PITCH', live ? `${num(d.pitch, 1)}°` : '--'],
              ['ROLL', live ? `${num(d.roll, 1)}°` : '--'],
              ['DIST', dist == null ? '--' : `${num(dist, 1)} m`],
            ].map(([k, v]) => (
              <div key={k} className="flex flex-col">
                <span className="text-[#94A3B8] text-[8px] tracking-widest">{k}</span>
                <span className="text-[#F1F5F9] font-mono tnum">{v}</span>
              </div>
            ))}
            <div className="flex flex-col">
              <span className="text-[#94A3B8] text-[8px] tracking-widest">STATE</span>
              <span className={`font-bold ${follow === 'FOLLOW' ? 'text-[#22D3EE]' : 'text-[#F1F5F9]'}`}>{follow}</span>
            </div>
          </div>
          <div className="flex items-center gap-1.5">
            <div className={`w-2 h-2 rounded-full ${s.stack.running ? 'bg-red-500 animate-pulse' : 'bg-slate-600'}`}></div>
            <span className="text-[#94A3B8] text-[8px] tracking-widest">REC</span>
            <span className="text-[#F1F5F9] ml-1 font-mono tnum">
              {s.stack.running ? fmtElapsed((s.now - s.video.recStart) / 1000) : '--:--:--'}
            </span>
          </div>
        </div>
      </div>
    </div>
  );
}
