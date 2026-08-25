import React from 'react';
import { useApi, useConsole } from '../state/store.jsx';
import { Panel, PanelHeader, Chip, Sparkline } from './common.jsx';
import { fmtHMS, num } from '../lib/format.js';

const SEV_TEXT = { INFO: 'text-[#F1F5F9]', WARNING: 'text-[#F97316]', ERROR: 'text-red-400' };
const FILTERS = ['ALL', 'INFO', 'WARNING', 'ERROR'];

/**
 * Timeline indicator dot. One consistent language across the interface:
 *   green  online / healthy      amber  warning        red  error / offline
 *   cyan   active mission        grey   idle
 */
function EventDot({ e }) {
  const src = (e.source || '').toUpperCase();
  const mission = src.includes('LLM') || src.includes('SIM') || src.includes('MISSION');
  const cls =
    e.severity === 'ERROR'
      ? 'bg-red-500'
      : e.severity === 'WARNING'
      ? 'bg-[#F97316]'
      : mission
      ? 'bg-[#22D3EE]'          // active mission
      : src === 'SYSTEM' || src === ''
      ? 'bg-slate-500'          // idle / informational
      : 'bg-[#4ADE80]';         // healthy vehicle report
  return <span className={`w-1.5 h-1.5 rounded-full shrink-0 mt-1 ${cls}`} />;
}

export function EventTimeline() {
  const s = useConsole();
  const api = useApi();
  const rows = s.events.filter((e) => s.eventFilter === 'ALL' || e.severity === s.eventFilter);

  return (
    <Panel className="flex-1 overflow-hidden">
      <PanelHeader
        title="EVENT TIMELINE"
        className="py-2"
        right={
          <div className="flex gap-3 text-[9px] font-bold">
            {FILTERS.map((f) => (
              <button
                key={f}
                onClick={() => api.setFilter(f)}
                className={`cursor-pointer ${
                  s.eventFilter === f
                    ? 'text-slate-200 border-b border-slate-200 pb-0.5'
                    : f === 'INFO'
                    ? 'text-cyan-500 hover:text-cyan-300'
                    : f === 'WARNING'
                    ? 'text-[#F97316] hover:text-orange-300'
                    : f === 'ERROR'
                    ? 'text-red-500 hover:text-red-300'
                    : 'text-[#94A3B8] hover:text-slate-200'
                }`}
              >
                {f}
              </button>
            ))}
          </div>
        }
      />
      <div className="flex-1 p-3 flex flex-col overflow-hidden min-h-0">
        {/* Sans for the message text; mono only for the timestamp (a numeric value). */}
        <div className="flex-1 overflow-y-auto pr-2 flex flex-col gap-1.5 text-[10px] tracking-normal normal-case min-h-0">
          {rows.length === 0 && <div className="text-[#94A3B8]">No events.</div>}
          {rows.map((e) => (
            <div key={e.id} className={`grid grid-cols-[10px_50px_70px_1fr] gap-2 items-start ${SEV_TEXT[e.severity] || 'text-[#F1F5F9]'}`}>
              <EventDot e={e} />
              <span className="text-[#94A3B8] font-mono tnum">{fmtHMS(e.t)}</span>
              <span className="text-[#94A3B8] truncate">{e.source}</span>
              <span className="break-words">{e.text}</span>
            </div>
          ))}
        </div>
        <div className="mt-2 pt-2 border-t border-slate-800/50">
          <Chip onClick={() => api.setOverlay('log')} className="w-full py-1.5 text-[9px] font-semibold tracking-wider">
            VIEW FULL LOG
          </Chip>
        </div>
      </div>
    </Panel>
  );
}

/** One metric column: live value + axis-labelled sparkline of the real sampled series. */
function Metric({ label, sub, value, unit, data, min, max, color, axis }) {
  return (
    <div className="flex flex-col justify-between px-3 h-full min-w-0">
      <div>
        <div className="text-[10px] text-[#94A3B8] mb-0.5 truncate">{label}</div>
        <div className="text-[9px] text-[#94A3B8] normal-case tracking-normal mb-1">{sub}</div>
        <div className="text-2xl text-[#F1F5F9] font-normal tracking-normal flex justify-end tnum">
          {value}
          <span className="text-lg ml-0.5">{unit}</span>
        </div>
      </div>
      <div className="h-16 relative mt-2 text-[8px] text-[#94A3B8] font-mono">
        <div className="absolute left-0 top-0">{axis[2]}</div>
        <div className="absolute left-0 top-1/2 -translate-y-1/2">{axis[1]}</div>
        <div className="absolute left-0 bottom-4">{axis[0]}</div>
        <div className="absolute bottom-0 w-full flex justify-between pl-6 pr-1 border-t border-slate-800 pt-1">
          <span>-60s</span><span>-45s</span><span>-30s</span><span>-15s</span><span>Now</span>
        </div>
        <Sparkline
          data={data}
          min={min}
          max={max}
          color={color}
          className="absolute bottom-5 left-6 w-[calc(100%-24px)] h-[calc(100%-20px)]"
        />
      </div>
    </div>
  );
}

export function SystemPerformance() {
  const s = useConsole();
  const last = (a) => (a.length ? a[a.length - 1] : 0);

  return (
    <Panel className="flex-[1.8] overflow-hidden">
      <PanelHeader title="SYSTEM PERFORMANCE" className="py-2" />
      {/* Memory removed — CPU, link quality and position error are the metrics that
          actually indicate whether the swarm is healthy. */}
      <div className="flex-1 grid grid-cols-3 divide-x divide-slate-800 p-3 min-h-0">
        <Metric
          label="CPU LOAD" sub="(All Agents)" value={num(last(s.perf.cpu), 0)} unit="%"
          data={s.perf.cpu} min={0} max={100} color="#22D3EE" axis={['0%', '50%', '100%']}
        />
        <Metric
          label="LINK QUALITY" sub="(Average)" value={num(last(s.perf.link), 0)} unit="%"
          data={s.perf.link} min={0} max={100} color="#4ADE80" axis={['0%', '50%', '100%']}
        />
        <Metric
          label="POSITION ERROR (RMS)" sub="(m)" value={num(last(s.perf.posErr), 2)} unit=" m"
          data={s.perf.posErr} min={0} max={1} color="#22D3EE" axis={['0.0', '0.5', '1.0']}
        />
      </div>
    </Panel>
  );
}
