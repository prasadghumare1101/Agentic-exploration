import React from 'react';
import { useApi, useConsole } from '../state/store.jsx';
import { Panel, PanelHeader } from './common.jsx';
import { num } from '../lib/format.js';

/**
 * Adaptive overview: a row of agent cards for a swarm mission, or one focused card for a
 * single-drone mission. Same card markup either way, so the design is unchanged.
 */
export default function SwarmOverview() {
  const s = useConsole();
  const api = useApi();
  const single = !s.isSwarm;
  const nominal = s.list.filter((d) => !s.stale && (d.battery ?? 0) >= 30 && (d.link_quality ?? 0) >= 60).length;

  return (
    <Panel className="h-[180px] flex-none overflow-hidden">
      <PanelHeader
        title={single ? 'SYSTEM OVERVIEW' : 'SWARM OVERVIEW'}
        className="py-2"
        right={
          <>
            <span className="text-[#94A3B8] text-[9px] tnum">
              {single ? 'SINGLE DRONE' : `${s.list.length} AGENTS`} · {nominal} NOMINAL
            </span>
            <button onClick={() => api.setOverlay('swarm')} className="text-slate-300 hover:text-white flex items-center gap-1 text-[9px]">
              VIEW ALL
              <svg width="10" height="10" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><polyline points="9 18 15 12 9 6"></polyline></svg>
            </button>
          </>
        }
      />
      <div className={`flex-1 p-3 flex gap-3 min-h-0 ${single ? '' : 'overflow-x-auto'}`}>
        {s.list.length === 0 && (
          <div className="flex-1 flex items-center justify-center text-slate-600 text-[10px] tracking-wider">
            NO AGENTS — POWER ON THE SIMULATION
          </div>
        )}

        {s.list.map((d) => {
          const stale = s.stale;
          const warn = stale || (d.battery ?? 0) < 30 || (d.link_quality ?? 0) < 60;
          const selected = d.id === s.selected;
          return (
            <button
              key={d.id}
              onClick={() => api.select(d.id)}
              title={`Show ${d.id} on the status, SLAM and video panels`}
              className={`min-w-[130px] flex-1 bg-[#18202d] border rounded p-2.5 flex flex-col justify-between relative overflow-hidden text-left ${
                selected ? 'border-[#22D3EE]' : 'border-slate-700 hover:border-slate-500'
              }`}
            >
              <div className={`absolute top-0 left-0 w-full h-[2px] ${warn ? 'bg-[#F97316]' : selected ? 'bg-[#22D3EE]' : 'bg-slate-600'}`}></div>

              <div>
                <div className="flex items-center justify-between">
                  <div className="text-[#F1F5F9] text-[11px] font-bold">{d.id.toUpperCase()}</div>
                  {selected && <div className="text-[#22D3EE] text-[8px] font-bold tracking-widest">ACTIVE</div>}
                </div>
                <div className="text-[#94A3B8] text-[9px] mb-2">{d.role || (d.follow_state ?? 'AGENT')}</div>
                <div className="flex justify-between items-end mb-3">
                  <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" className="text-slate-300">
                    <path d="M4 10h16M12 6v8M8 6h8M6 14l2 2h8l2-2" strokeLinecap="round" strokeLinejoin="round"/>
                    <circle cx="6" cy="10" r="1.5" fill="currentColor"/><circle cx="18" cy="10" r="1.5" fill="currentColor"/>
                  </svg>
                  <div className="flex flex-col items-end">
                    <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" className={`${warn ? 'text-[#F97316]' : 'text-[#4ADE80]'} mb-0.5`}><path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"></path><polyline points="22 4 12 14.01 9 11.01"></polyline></svg>
                    <span className={`text-[8px] font-bold tracking-widest ${warn ? 'text-[#F97316]' : 'text-[#4ADE80]'}`}>
                      {stale ? 'STALE' : warn ? 'WARNING' : d.armed ? 'ARMED' : 'ONLINE'}
                    </span>
                  </div>
                </div>
              </div>

              <div className="flex flex-col gap-1 text-[10px]">
                <div className="flex justify-between"><span className="text-[#94A3B8]">BATTERY</span><span className={`font-mono tnum ${(d.battery ?? 0) < 30 ? 'text-[#F97316] font-bold' : 'text-[#F1F5F9]'}`}>{num(d.battery, 0)}%</span></div>
                <div className="flex justify-between"><span className="text-[#94A3B8]">ALT</span><span className="text-[#F1F5F9] font-mono tnum">{num(d.altitude, 0)} m</span></div>
                <div className="flex justify-between"><span className="text-[#94A3B8]">LINK</span><span className={`font-mono tnum ${(d.link_quality ?? 0) < 60 ? 'text-[#F97316]' : 'text-[#F1F5F9]'}`}>{num(d.link_quality, 0)}%</span></div>
              </div>
            </button>
          );
        })}
      </div>
    </Panel>
  );
}
