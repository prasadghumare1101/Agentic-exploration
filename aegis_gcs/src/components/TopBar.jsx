import React from 'react';
import { useApi, useConsole } from '../state/store.jsx';
import { fmtClockZ, fmtDateUTC } from '../lib/format.js';

const SAFETY_COLOR = {
  HEALTHY: 'text-[#4ADE80]',
  CAUTION: 'text-[#F97316]',
  CRITICAL: 'text-red-400',
  'NO TELEMETRY': 'text-red-400',
};

const Divider = () => <div className="w-px h-6 bg-slate-800"></div>;

const Field = ({ label, children, title }) => (
  <div className="flex flex-col justify-center" title={title}>
    <span className="text-[#94A3B8] text-[10px]">{label}</span>
    {children}
  </div>
);

const SELECT =
  'bg-[#182030] border border-slate-700 rounded text-[10px] text-slate-200 px-1.5 py-1 focus:outline-none focus:border-[#22D3EE] disabled:opacity-40 disabled:cursor-not-allowed';

export default function TopBar() {
  const s = useConsole();
  const api = useApi();
  const locked = s.stack.running || s.stack.busy;

  return (
    <header className="flex-none h-14 border-b border-slate-800 bg-[#0E131F] flex items-center px-4 justify-between">
      {/* Identity */}
      <div className="flex items-center gap-4 border-r border-slate-800 pr-6 h-full">
        <div className="w-8 h-8 rounded-full border border-slate-600 flex items-center justify-center bg-slate-800/50">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" className="text-slate-300"><path d="M12 2L2 7l10 5 10-5-10-5zM2 17l10 5 10-5M2 12l10 5 10-5"/></svg>
        </div>
        <div>
          <div className="text-[#F1F5F9] font-bold text-sm tracking-widest">AEGIS OPS CONSOLE</div>
          <div className="text-[#94A3B8] text-[10px] mt-0.5 tracking-[0.15em]">ROS2 | PX4 | LLM | SWARM | SLAM</div>
        </div>
      </div>

      {/* Mission context — live */}
      <div className="flex flex-1 items-center justify-center gap-8 h-full">
        <Field label="MISSION">
          <span className="text-[#F1F5F9] text-sm font-semibold">{s.mission.name}</span>
          {/* executor progress: COMPLETE is the signal that the plan finished, not stalled */}
          {s.mission.state === 'COMPLETE' && (
            <span className="ml-2 px-1.5 py-0.5 text-[9px] font-semibold tracking-wider rounded border border-[#4ADE80] text-[#4ADE80] bg-green-950/30">
              COMPLETE
            </span>
          )}
          {s.mission.state === 'EXECUTING' && (
            <span className="ml-2 px-1.5 py-0.5 text-[9px] font-semibold tracking-wider rounded border border-[#22D3EE] text-[#22D3EE] bg-cyan-950/30">
              EXECUTING
            </span>
          )}
        </Field>
        <Divider />
        <Field label="UTC TIME">
          <span className="text-[#F1F5F9] text-sm font-semibold tracking-normal tnum">
            {fmtClockZ(s.now)}
            <span className="text-[#94A3B8] text-xs ml-1 font-normal uppercase">{fmtDateUTC(s.now)}</span>
          </span>
        </Field>
        <Divider />
        <Field label="FLIGHT MODE">
          <span className="text-[#F1F5F9] text-sm font-semibold">{s.mission.flightMode || '--'}</span>
        </Field>
        <Divider />
        <Field label="SAFETY STATE" title={s.safety.reason}>
          <span className={`text-sm font-semibold ${SAFETY_COLOR[s.safety.state] || 'text-slate-300'}`}>
            {s.safety.state}
          </span>
        </Field>
        <Divider />
        <Field label="AUTONOMY STATE">
          <span className="text-[#22D3EE] text-sm font-semibold">{s.mission.autonomy}</span>
        </Field>
      </div>

      {/* Mission setup + controls */}
      <div className="flex items-center gap-3 pl-6 border-l border-slate-800 h-full text-[#94A3B8]">
        {/* Mission type: single vs swarm */}
        <select
          value={s.sim.mode}
          onChange={(e) =>
            api.setSim({ mode: e.target.value, drones: e.target.value === 'swarm' ? 3 : 1 })
          }
          disabled={locked}
          title="Mission type — single drone or swarm (applies on power-on)"
          className={SELECT}
        >
          <option value="single">SINGLE</option>
          <option value="swarm">SWARM</option>
        </select>

        {/* World */}
        <select
          value={s.sim.world}
          onChange={(e) => api.setSim({ world: e.target.value })}
          disabled={locked}
          title="Gazebo world (applies on power-on)"
          className={`${SELECT} max-w-[120px]`}
        >
          {(s.sim.worlds?.length ? s.sim.worlds : [s.sim.world]).map((w) => (
            <option key={w} value={w}>{w}</option>
          ))}
        </select>

        {/* Moving target model */}
        <select
          value={s.sim.moving}
          onChange={(e) => api.setSim({ moving: e.target.value })}
          disabled={locked}
          title="Moving object to spawn as the tracking target"
          className={`${SELECT} max-w-[110px]`}
        >
          {(s.sim.movings?.length ? s.sim.movings : [s.sim.moving || 'none']).map((m) => (
            <option key={m} value={m}>{m}</option>
          ))}
        </select>

        {/* DECENTRALISED swarm. ON: each drone runs a drone_agent that self-assigns
            formation slots by distributed auction and re-bids when a peer drops, plus
            mission_supervisor for LLM replanning. OFF: the coordinator assigns centrally.
            Launch-time setting, so it locks once the stack is up. */}
        <button
          onClick={() => api.setSim({ agentic: !s.sim.agentic })}
          disabled={locked || s.sim.mode !== 'swarm'}
          title={
            s.sim.mode !== 'swarm'
              ? 'Decentralised coordination applies to swarm missions'
              : s.sim.agentic
              ? 'Decentralised ON — agents self-assign slots via distributed auction'
              : 'Decentralised OFF — coordinator assigns every slot centrally'
          }
          className={`flex items-center gap-1 px-1.5 py-1 rounded border text-[9px] font-semibold tracking-widest transition-colors disabled:opacity-40 disabled:cursor-not-allowed ${
            s.sim.agentic
              ? 'border-[#22D3EE] text-[#22D3EE] bg-cyan-950/30'
              : 'border-slate-700 text-[#94A3B8] hover:text-[#F1F5F9]'
          }`}
        >
          <span className={`w-1.5 h-1.5 rounded-full ${s.sim.agentic ? 'bg-[#22D3EE]' : 'bg-slate-600'}`} />
          DECENTRALISED {s.sim.agentic ? 'ON' : 'OFF'}
        </button>

        {/* Alerts */}
        <button
          className="relative cursor-pointer hover:text-slate-200"
          title={`${s.unacked.length} unacknowledged alerts`}
          onClick={() => api.setOverlay('alerts')}
        >
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"><path d="M18 8A6 6 0 0 0 6 8c0 7-3 9-3 9h18s-3-2-3-9"></path><path d="M13.73 21a2 2 0 0 1-3.46 0"></path></svg>
          {s.unacked.length > 0 && (
            <span className="absolute -top-1 -right-1 w-3.5 h-3.5 bg-red-500 text-white rounded-full text-[8px] flex items-center justify-center font-bold tnum">
              {s.unacked.length > 9 ? '9+' : s.unacked.length}
            </span>
          )}
        </button>

        {/* Settings / stack detail */}
        <button
          className="cursor-pointer hover:text-slate-200"
          title="Stack services and maintenance"
          onClick={() => api.setOverlay('stack')}
        >
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"><circle cx="12" cy="12" r="3"></circle><path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z"></path></svg>
        </button>

        {/* POWER — starts / stops the real simulation stack */}
        <button
          className={`cursor-pointer ${
            s.stack.busy
              ? 'text-amber-400 animate-pulse'
              : s.stack.running
              ? 'text-[#4ADE80] hover:text-green-300'
              : s.apiUp
              ? 'text-red-400 hover:text-red-300'
              : 'text-slate-600'
          }`}
          title={
            !s.apiUp
              ? 'Control API offline — start the AEGIS backend'
              : s.stack.busy
              ? 'Working…'
              : s.stack.running
              ? 'Power off — stop the simulation'
              : `Power on — launch ${s.sim.mode} mission in ${s.sim.world}`
          }
          disabled={!s.apiUp || s.stack.busy}
          onClick={api.powerToggle}
        >
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"><path d="M18.36 6.64a9 9 0 1 1-12.73 0"></path><line x1="12" y1="2" x2="12" y2="12"></line></svg>
        </button>
      </div>
    </header>
  );
}
