import React, { useState } from 'react';
import { useApi, useConsole } from '../state/store.jsx';
import { Chip, Overlay, Dot } from './common.jsx';
import { fmtHMS, num } from '../lib/format.js';

const SEV_TEXT = { INFO: 'text-slate-300', WARNING: 'text-[#F97316]', ERROR: 'text-red-400' };
const INPUT =
  'bg-[#182030] border border-slate-700 rounded px-2 py-1.5 text-[10px] text-slate-200 placeholder-slate-500 focus:outline-none focus:border-[#22D3EE]';

export default function Overlays() {
  const s = useConsole();
  const api = useApi();
  const close = () => api.setOverlay(null);

  if (!s.overlay) return null;

  if (s.overlay === 'alerts') {
    return (
      <Overlay title="UNACKNOWLEDGED ALERTS" subtitle={`${s.unacked.length} active`} onClose={close}>
        <div className="p-3 flex flex-col gap-1.5 text-[10px]">
          {s.unacked.length === 0 && <div className="text-[#94A3B8]">NO UNACKNOWLEDGED WARNINGS OR ERRORS</div>}
          {s.unacked.map((e) => (
            <div key={e.id} className="flex gap-3 items-center border border-slate-800 rounded px-2 py-1.5">
              <span className="text-[#94A3B8] font-mono tnum">{fmtHMS(e.t)}</span>
              <span className="w-20 shrink-0 text-[#94A3B8] truncate">{e.source}</span>
              <span className={`flex-1 normal-case tracking-normal ${SEV_TEXT[e.severity]}`}>{e.text}</span>
              <Chip onClick={() => api.ack(e.id)} className="px-2 py-0.5 text-[9px] tracking-wider">ACK</Chip>
            </div>
          ))}
        </div>
        {s.unacked.length > 0 && (
          <div className="p-3 pt-0">
            <Chip onClick={() => api.ack('ALL')} className="w-full py-1.5 text-[9px] font-semibold tracking-wider">ACKNOWLEDGE ALL</Chip>
          </div>
        )}
      </Overlay>
    );
  }

  if (s.overlay === 'log') return <LogOverlay onClose={close} />;
  if (s.overlay === 'plan') return <PlanOverlay onClose={close} />;
  if (s.overlay === 'stack') return <StackOverlay onClose={close} />;

  if (s.overlay === 'swarm') {
    return (
      <Overlay title="SWARM — ALL AGENTS" subtitle="click a row to make it the active feed" onClose={close} wide>
        <table className="w-full text-[10px]">
          <thead className="text-[#94A3B8] border-b border-slate-800">
            <tr>
              {['AGENT', 'STATE', 'ARMED', 'BATT', 'ALT', 'SPD', 'HDG', 'LINK', 'LOC Q', 'LOOPS', 'POS ERR', 'LAT', 'LON'].map((h) => (
                <th key={h} className="text-left font-semibold tracking-wider px-2 py-2">{h}</th>
              ))}
            </tr>
          </thead>
          <tbody className="tnum">
            {s.list.map((d) => (
              <tr
                key={d.id}
                onClick={() => { api.select(d.id); close(); }}
                className={`cursor-pointer border-b border-slate-800/50 hover:bg-slate-800/50 ${d.id === s.selected ? 'bg-slate-800/40' : ''}`}
              >
                <td className="px-2 py-1.5 text-[#F1F5F9]">{d.id.toUpperCase()}</td>
                <td className="px-2 py-1.5 text-[#22D3EE]">{d.follow_state || '--'}</td>
                <td className={`px-2 py-1.5 ${d.armed ? 'text-[#4ADE80]' : 'text-[#94A3B8]'}`}>{d.armed ? 'YES' : 'NO'}</td>
                <td className={`px-2 py-1.5 ${(d.battery ?? 0) < 30 ? 'text-[#F97316]' : 'text-slate-200'}`}>{num(d.battery, 0)}%</td>
                <td className="px-2 py-1.5 text-slate-300">{num(d.altitude, 0)} m</td>
                <td className="px-2 py-1.5 text-slate-300">{num(d.speed, 0)}</td>
                <td className="px-2 py-1.5 text-slate-300">{num(d.heading, 0)}°</td>
                <td className={`px-2 py-1.5 ${(d.link_quality ?? 0) < 60 ? 'text-[#F97316]' : 'text-slate-300'}`}>{num(d.link_quality, 0)}%</td>
                <td className="px-2 py-1.5 text-slate-300">{num(d.slam?.loc_quality, 0)}%</td>
                <td className="px-2 py-1.5 text-slate-300">{d.slam?.loops ?? '--'}</td>
                <td className="px-2 py-1.5 text-slate-300">{num(d.pos_err, 2)}</td>
                <td className="px-2 py-1.5 text-[#94A3B8]">{num(d.lat, 5)}</td>
                <td className="px-2 py-1.5 text-[#94A3B8]">{num(d.lon, 5)}</td>
              </tr>
            ))}
            {s.list.length === 0 && (
              <tr><td colSpan={13} className="px-2 py-4 text-slate-600">No agents — power on the simulation.</td></tr>
            )}
          </tbody>
        </table>
      </Overlay>
    );
  }

  return null;
}

function LogOverlay({ onClose }) {
  const s = useConsole();
  const [q, setQ] = useState('');
  const [sev, setSev] = useState('ALL');
  const rows = s.events.filter(
    (e) => (sev === 'ALL' || e.severity === sev) && (!q || `${e.source} ${e.text}`.toLowerCase().includes(q.toLowerCase()))
  );

  const exportLog = () => {
    const text = rows.slice().reverse().map((e) => `${fmtHMS(e.t)}\t${e.severity}\t${e.source}\t${e.text}`).join('\n');
    const blob = new Blob([`# AEGIS event log\n${text}\n`], { type: 'text/plain' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `aegis-log-${new Date().toISOString().replace(/[:.]/g, '-')}.txt`;
    a.click();
    URL.revokeObjectURL(a.href);
  };

  return (
    <Overlay title="EVENT LOG" subtitle={`${rows.length} of ${s.events.length}`} onClose={onClose} wide>
      <div className="p-3 flex gap-2 border-b border-slate-800 sticky top-0 bg-[#111722]">
        <input value={q} onChange={(e) => setQ(e.target.value)} placeholder="filter text…" className={`${INPUT} flex-1 normal-case tracking-normal`} />
        {['ALL', 'INFO', 'WARNING', 'ERROR'].map((f) => (
          <Chip key={f} onClick={() => setSev(f)} className={`px-2 py-1 text-[9px] tracking-wider ${sev === f ? 'border-cyan-600 text-cyan-300' : ''}`}>{f}</Chip>
        ))}
        <Chip onClick={exportLog} className="px-2 py-1 text-[9px] tracking-wider">EXPORT</Chip>
      </div>
      <div className="p-3 flex flex-col gap-1 text-[10px] font-mono normal-case tracking-normal">
        {rows.map((e) => (
          <div key={e.id} className={`grid grid-cols-[60px_90px_1fr] gap-2 ${SEV_TEXT[e.severity] || 'text-slate-300'}`}>
            <span className="text-[#94A3B8] tnum">{fmtHMS(e.t)}</span>
            <span className="text-[#94A3B8]">{e.source}</span>
            <span>{e.text}</span>
          </div>
        ))}
      </div>
    </Overlay>
  );
}

function PlanOverlay({ onClose }) {
  const s = useConsole();
  const p = s.lastPlan;
  let pretty = '';
  if (p) {
    try {
      pretty = JSON.stringify(typeof p.plan === 'string' ? JSON.parse(p.plan) : p.plan, null, 2);
    } catch {
      pretty = String(p.plan ?? '');
    }
  }
  return (
    <Overlay title="MISSION PLAN — JSON" subtitle={p ? `validated plan · ${fmtHMS(p.t)}` : 'no plan yet'} onClose={onClose} wide>
      {!p && (
        <div className="p-4 text-[#94A3B8] text-[10px] normal-case tracking-normal">
          No mission plan yet. Issue a natural-language command; the validated plan appears here.
        </div>
      )}
      {p && (
        <div className="p-3 flex flex-col gap-3">
          {p.unsupported?.length > 0 && (
            <div className="text-[10px] text-[#F97316] normal-case tracking-normal">Unsupported: {p.unsupported.join(', ')}</div>
          )}
          <pre className="text-[10px] font-mono text-slate-300 bg-[#0c111a] border border-slate-800 rounded p-3 overflow-auto max-h-[60vh] whitespace-pre normal-case tracking-normal">
            {pretty}
          </pre>
        </div>
      )}
    </Overlay>
  );
}

function StackOverlay({ onClose }) {
  const s = useConsole();
  const api = useApi();
  return (
    <Overlay title="SIMULATION STACK" subtitle={`${s.sim.mode.toUpperCase()} · ${s.sim.world}${s.sim.moving && s.sim.moving !== 'none' ? ` · ${s.sim.moving}` : ''}`} onClose={onClose}>
      <div className="p-4 flex flex-col gap-3 text-[10px]">
        {!s.apiUp && <div className="text-red-400">Control API offline (http://localhost:8000). Start the AEGIS backend.</div>}
        {(s.stack.services || []).map((svc) => (
          <div key={svc.name} className="flex items-center gap-3 border border-slate-800 rounded px-2 py-1.5">
            <Dot tone={svc.running ? 'ok' : 'idle'} />
            <span className="w-24 text-slate-200 uppercase">{svc.name}</span>
            <span className="flex-1 text-[#94A3B8] normal-case tracking-normal truncate">{svc.detail || ''}</span>
            <span className={svc.running ? 'text-[#4ADE80]' : 'text-[#94A3B8]'}>{svc.running ? 'RUNNING' : 'STOPPED'}</span>
          </div>
        ))}
        {(!s.stack.services || s.stack.services.length === 0) && <div className="text-slate-600">No service data.</div>}
        <div className="flex gap-2 pt-2 border-t border-slate-800">
          <Chip onClick={api.powerToggle} disabled={!s.apiUp || s.stack.busy} className="flex-1 py-1.5 text-[9px] font-semibold tracking-wider">
            {s.stack.running ? 'STOP STACK' : 'START STACK'}
          </Chip>
          <Chip onClick={api.cleanup} disabled={!s.apiUp} className="flex-1 py-1.5 text-[9px] tracking-wider">REAP ORPHANS</Chip>
        </div>
      </div>
    </Overlay>
  );
}
