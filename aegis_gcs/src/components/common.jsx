import React, { useEffect } from 'react';

/** Panel shell — matches the console's slate-navy surface + hairline border. */
export function Panel({ className = '', children }) {
  return (
    <div className={`bg-[#111722] border border-slate-700/60 rounded flex flex-col ${className}`}>
      {children}
    </div>
  );
}

/** Panel header — uppercase tracked label with an optional right-hand slot. */
export function PanelHeader({ title, right, className = '' }) {
  return (
    <div className={`flex-none flex justify-between items-center gap-3 px-4 py-2.5 border-b border-slate-800 ${className}`}>
      <h2 className="text-[11px] text-slate-300 font-semibold tracking-wider truncate">{title}</h2>
      {right && <div className="flex items-center gap-3 text-[10px] shrink-0">{right}</div>}
    </div>
  );
}

/** Standard action button used across the panels. */
export function Chip({ children, className = '', ...props }) {
  return (
    <button
      {...props}
      className={`bg-slate-800/50 hover:bg-slate-700 border border-slate-700 rounded text-slate-300 hover:text-[#F1F5F9] transition-colors disabled:opacity-40 disabled:cursor-not-allowed ${className}`}
    >
      {children}
    </button>
  );
}

export function Dot({ tone }) {
  const bg =
    tone === 'ok'
      ? 'bg-[#4ADE80]'
      : tone === 'warn'
      ? 'bg-[#F97316]'
      : tone === 'err'
      ? 'bg-red-500'
      : 'bg-slate-600';
  return <div className={`w-1.5 h-1.5 rounded-full ${bg}`}></div>;
}

/** Status text colour for a tone. */
export const toneText = (t) =>
  t === 'ok' ? 'text-[#4ADE80]' : t === 'warn' ? 'text-[#F97316]' : t === 'err' ? 'text-red-400' : 'text-[#94A3B8]';

/** Sparkline scaled to the data it is given (axis labels in the panels match these bounds). */
export function Sparkline({ data, min, max, color, className }) {
  if (!data || data.length < 2) return null;
  const span = max - min || 1;
  const n = data.length - 1;
  const pts = data
    .map((v, i) => {
      const x = (i / n) * 100;
      const y = 100 - Math.min(100, Math.max(0, ((v - min) / span) * 100));
      return `${x.toFixed(2)},${y.toFixed(2)}`;
    })
    .join(' ');
  return (
    <svg className={className} viewBox="0 0 100 100" preserveAspectRatio="none">
      <polyline points={pts} fill="none" stroke={color} strokeWidth="1.5" vectorEffect="non-scaling-stroke" />
    </svg>
  );
}

/** Modal overlay shell — Esc closes, click outside closes. */
export function Overlay({ title, subtitle, onClose, children, wide }) {
  useEffect(() => {
    const h = (e) => e.key === 'Escape' && onClose();
    window.addEventListener('keydown', h);
    return () => window.removeEventListener('keydown', h);
  }, [onClose]);

  return (
    <div
      className="absolute inset-0 z-50 bg-black/70 flex items-center justify-center p-8"
      onClick={onClose}
    >
      <div
        onClick={(e) => e.stopPropagation()}
        className={`bg-[#111722] border border-slate-700 rounded shadow-2xl flex flex-col max-h-full overflow-hidden ${
          wide ? 'w-[1100px]' : 'w-[560px]'
        }`}
      >
        <div className="flex-none flex justify-between items-center px-4 py-2.5 border-b border-slate-800">
          <div>
            <h2 className="text-[11px] text-slate-200 font-semibold tracking-wider">{title}</h2>
            {subtitle && <div className="text-[9px] text-[#94A3B8] normal-case tracking-normal mt-0.5">{subtitle}</div>}
          </div>
          <Chip onClick={onClose} className="px-2 py-1 text-[9px] tracking-wider">
            CLOSE (ESC)
          </Chip>
        </div>
        <div className="overflow-y-auto">{children}</div>
      </div>
    </div>
  );
}
