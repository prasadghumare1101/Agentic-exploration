import React, { useEffect, useRef, useState } from 'react';
import { useApi, useConsole } from '../state/store.jsx';
import { Panel, PanelHeader, Chip } from './common.jsx';
import { fmtHMS } from '../lib/format.js';

const ROLE_LABEL = { SYSTEM: 'SYSTEM', YOU: 'YOU', AGENT: 'LLM AGENT' };
const ROLE_STYLE = { SYSTEM: 'text-[#94A3B8]', YOU: 'text-slate-300', AGENT: 'text-[#22D3EE] font-bold' };

/**
 * Report actions. These are QUESTIONS about live state — they are answered from telemetry,
 * never sent to the mission planner (which would try to fly "report vehicle status").
 */
const QUICK = [
  ['SUGGEST ACTION', 'suggest'],
  ['SUMMARIZE', 'summary'],
  ['EXPLAIN', 'explain'],
  ['CHECK STATUS', 'status'],
];

export default function LlmPanel() {
  const s = useConsole();
  const api = useApi();
  const [draft, setDraft] = useState('');
  const [recallIdx, setRecallIdx] = useState(-1);
  const scroller = useRef(null);
  const inputRef = useRef(null);
  const fileRef = useRef(null);

  // Target images are registered BEFORE a mission; the LLM and the agents use them when the
  // mission is issued. Uploading alone never arms anything.
  const onFile = (e) => {
    const files = Array.from(e.target.files || []);
    files.forEach((f) => api.uploadTarget(f));
    e.target.value = '';
  };
  const pinned = useRef(true);

  useEffect(() => {
    const el = scroller.current;
    if (el && pinned.current) el.scrollTop = el.scrollHeight;
  }, [s.chat]);

  useEffect(() => {
    const h = (e) => {
      if (e.key === '/' && document.activeElement !== inputRef.current) {
        e.preventDefault();
        inputRef.current?.focus();
      }
    };
    window.addEventListener('keydown', h);
    return () => window.removeEventListener('keydown', h);
  }, []);

  const send = () => {
    if (!draft.trim() || s.llmBusy) return;
    api.submitCommand(draft);
    setDraft('');
    setRecallIdx(-1);
  };

  const onKey = (e) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      send();
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      const next = Math.min(recallIdx + 1, s.commandHistory.length - 1);
      if (next >= 0 && s.commandHistory[next]) {
        setRecallIdx(next);
        setDraft(s.commandHistory[next].text);
      }
    } else if (e.key === 'ArrowDown') {
      e.preventDefault();
      const next = recallIdx - 1;
      setRecallIdx(next);
      setDraft(next >= 0 && s.commandHistory[next] ? s.commandHistory[next].text : '');
    }
  };

  return (
    <Panel className="flex-1 min-h-0 overflow-hidden">
      <PanelHeader
        title="LLM COMMAND PANEL"
        right={
          <span className="text-[#94A3B8]">
            STATUS:{' '}
            <span className={`font-bold tracking-widest ${s.apiUp ? 'text-[#22D3EE]' : 'text-red-400'}`}>
              {s.llmBusy ? 'PLANNING…' : s.apiUp ? 'CONNECTED' : 'OFFLINE'}
            </span>
          </span>
        }
      />

      <div className="flex flex-1 min-h-0">
        {/* Transcript */}
        <div className="flex-[1.5] flex flex-col border-r border-slate-800 min-w-0">
          <div
            ref={scroller}
            onScroll={(e) => {
              const el = e.currentTarget;
              pinned.current = el.scrollHeight - el.scrollTop - el.clientHeight < 24;
            }}
            className="flex-1 p-3 overflow-y-auto flex flex-col gap-4 text-[11px] min-h-0 tracking-normal normal-case"
          >
            {s.chat.map((m) => (
              <div key={m.id}>
                <div className="flex items-center gap-2 mb-1 uppercase tracking-wider text-[9px]">
                  <span className={ROLE_STYLE[m.role] || 'text-[#94A3B8]'}>{ROLE_LABEL[m.role] || m.role}</span>
                  <span className="text-slate-600 tnum">{fmtHMS(m.t)}</span>
                </div>
                <div
                  className={`whitespace-pre-wrap break-words leading-relaxed ${
                    m.role === 'YOU'
                      ? 'bg-slate-800/50 p-2 rounded text-[#F1F5F9] border border-slate-700/50'
                      : 'text-slate-300'
                  }`}
                >
                  {m.text}
                </div>
              </div>
            ))}
          </div>

          <div className="flex-none p-3 bg-[#0c111a] border-t border-slate-800 flex flex-col gap-2">
            {s.targets.length > 0 && (
              <div className="flex items-center gap-1.5 flex-wrap text-[9px]">
                <span className="text-[#94A3B8] tracking-widest">TARGETS:</span>
                {s.targets.map((t, i) => (
                  <span key={i} className="px-1.5 py-0.5 rounded border border-[#22D3EE]/60 text-[#22D3EE] bg-cyan-950/30 normal-case tracking-normal">
                    {t.label} <span className="font-mono tnum">{Math.round((t.conf || 0) * 100)}%</span>
                  </span>
                ))}
                <button onClick={api.clearTargets} title="Clear targets and return detectors to idle" className="text-[#94A3B8] hover:text-[#F97316] ml-1">clear</button>
              </div>
            )}
            <div className="relative">
              <input
                ref={inputRef}
                type="text"
                value={draft}
                onChange={(e) => setDraft(e.target.value)}
                onKeyDown={onKey}
                disabled={s.llmBusy}
                placeholder={s.apiUp ? 'Type command or request…' : 'Control API offline — start the backend'}
                className="w-full bg-[#182030] border border-slate-700 rounded p-2 pr-14 text-[11px] text-slate-200 placeholder-slate-500 focus:outline-none focus:border-[#22D3EE] tracking-normal normal-case disabled:opacity-50"
              />
              <div className="absolute right-2 top-1/2 -translate-y-1/2 flex items-center gap-2">
                <input ref={fileRef} type="file" accept="image/*" multiple onChange={onFile} className="hidden" />
                <button
                  onClick={() => fileRef.current?.click()}
                  title="Upload target image(s) — the LLM and agents use these when you issue the mission"
                  className="text-[#94A3B8] hover:text-[#22D3EE]"
                >
                  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"></path><polyline points="17 8 12 3 7 8"></polyline><line x1="12" y1="3" x2="12" y2="15"></line></svg>
                </button>
                <button onClick={send} disabled={s.llmBusy} title="Send to the LLM planner" className="text-[#94A3B8] hover:text-[#22D3EE] disabled:opacity-40">
                  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><line x1="22" y1="2" x2="11" y2="13"></line><polygon points="22 2 15 22 11 13 2 9 22 2"></polygon></svg>
                </button>
              </div>
            </div>
            <div className="flex gap-2">
              {QUICK.map(([label, kind]) => (
                <Chip key={label} onClick={() => api.report(kind)} className="flex-1 py-1 text-[9px]">
                  {label}
                </Chip>
              ))}
            </div>
          </div>
        </div>

        {/* History */}
        <div className="flex-1 flex flex-col bg-[#0f1520] min-w-0">
          <div className="flex-none px-3 py-2 text-[10px] text-[#94A3B8] font-semibold border-b border-slate-800/50 flex justify-between">
            <span>COMMAND HISTORY</span>
            <span className="text-slate-600 tnum">{s.commandHistory.length}</span>
          </div>
          <div className="flex-1 overflow-y-auto p-2 flex flex-col gap-1 text-[10px] min-h-0 tracking-normal normal-case">
            {s.commandHistory.length === 0 && <div className="text-slate-600 p-1.5">No commands issued.</div>}
            {s.commandHistory.slice(0, 40).map((h) => (
              <button
                key={h.id}
                onClick={() => {
                  setDraft(h.text);
                  inputRef.current?.focus();
                }}
                title={`${h.status}: ${h.summary}`}
                className="flex gap-2 p-1.5 hover:bg-slate-800/50 rounded cursor-pointer text-left"
              >
                <span className="text-[#94A3B8] shrink-0 tnum">{fmtHMS(h.t)}</span>
                <span className={`truncate ${h.status === 'ok' ? 'text-slate-300' : h.status === 'pending' ? 'text-[#94A3B8]' : 'text-[#F97316]'}`}>
                  {h.text}
                </span>
              </button>
            ))}
          </div>
          <div className="flex-none p-2 border-t border-slate-800/50 flex gap-2">
            <Chip onClick={() => api.setOverlay('plan')} className="flex-1 py-1.5 text-[9px] font-semibold">PLAN JSON</Chip>
            <Chip onClick={api.returnHome} className="flex-1 py-1.5 text-[9px] font-semibold text-[#22D3EE] hover:text-cyan-200">RETURN HOME</Chip>
            <Chip onClick={api.abortMission} className="flex-1 py-1.5 text-[9px] font-semibold text-red-300 hover:text-red-200">ABORT</Chip>
          </div>
        </div>
      </div>
    </Panel>
  );
}
