import React from 'react';
import { useApi, useConsole } from '../state/store.jsx';
import { Dot } from './common.jsx';
import { fmtElapsed } from '../lib/format.js';

/**
 * Bottom status bar — real connection health. Provenance is never asserted: a component only
 * claims to be up when the backend actually reports it.
 */
export default function Footer() {
  const s = useConsole();
  const api = useApi();
  const rosUp = s.link.status === 'connected';
  const svc = (name) => s.stack.services?.find((x) => x.name === name);
  const sitl = svc('sitl');
  const age = rosUp ? (s.now - s.link.lastRx) / 1000 : null;

  const items = [
    { label: `ROS 2: ${rosUp ? 'HUMBLE' : 'NOT CONNECTED'}`, tone: rosUp ? 'ok' : 'idle' },
    { label: `PX4: ${sitl?.running ? 'SITL RUNNING' : 'STOPPED'}`, tone: sitl?.running ? 'ok' : 'idle' },
    { label: `DDS: ${rosUp ? 'FAST DDS' : 'NOT CONNECTED'}`, tone: rosUp ? 'ok' : 'idle' },
    { label: `LLM: ${s.apiUp ? (s.llmBusy ? 'PLANNING' : 'READY') : 'OFFLINE'}`, tone: s.apiUp ? 'ok' : 'err' },
  ];

  return (
    <footer className="flex-none h-7 bg-[#0B101A] border-t border-slate-800/80 flex justify-between items-center px-4 text-[9px] text-[#94A3B8]">
      <div className="flex items-center gap-6">
        {items.map((i) => (
          <div key={i.label} className="flex items-center gap-1.5">
            <Dot tone={i.tone} />
            {i.label}
          </div>
        ))}
      </div>
      <div className="flex items-center gap-6">
        <button onClick={() => api.setOverlay('stack')} className="flex items-center gap-1.5 hover:text-slate-300" title="Stack services">
          <Dot tone={s.stack.running ? (s.stale ? 'warn' : 'ok') : 'idle'} />
          SIM {s.stack.busy ? 'WORKING…' : s.stack.running ? `UP (${s.sim.mode.toUpperCase()} · ${s.sim.world})` : 'OFF'}
          {age !== null && <span className="text-[#94A3B8] ml-1 tnum">{age.toFixed(1)}s</span>}
        </button>
        <div className="flex items-center gap-1.5">
          <Dot tone={s.stack.running ? 'err' : 'idle'} />
          DATA RECORDING {s.stack.running ? fmtElapsed((s.now - s.video.recStart) / 1000) : 'OFF'}
        </div>
        <div className="text-[#94A3B8] tnum">EVENTS: {s.events.length} / 400</div>
      </div>
    </footer>
  );
}
