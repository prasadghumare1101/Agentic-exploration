import React from 'react';
import { ConsoleProvider } from './state/store.jsx';
import TopBar from './components/TopBar.jsx';
import { DroneStatus, SlamHealth } from './components/LeftSidebar.jsx';
import VideoFeed from './components/VideoFeed.jsx';
import LlmPanel from './components/LlmPanel.jsx';
import SwarmOverview from './components/SwarmOverview.jsx';
import { EventTimeline, SystemPerformance } from './components/LowerRow.jsx';
import Footer from './components/Footer.jsx';
import Overlays from './components/Overlays.jsx';

/**
 * AEGIS OPS CONSOLE — layout preserved exactly as designed; every panel is bound to live
 * state from the store (rosbridge telemetry + control API).
 */
const AegisConsole = () => (
  <ConsoleProvider>
    <div className="h-screen w-screen bg-[#0B101A] text-slate-200 font-sans overflow-hidden flex flex-col selection:bg-cyan-900 selection:text-cyan-100 uppercase text-xs tracking-wider">
      <TopBar />

      <main className="flex-1 flex flex-row p-3 gap-3 overflow-hidden">
        {/* --- LEFT SIDEBAR --- */}
        <aside className="w-[300px] flex flex-col gap-3 flex-none">
          <DroneStatus />
          <SlamHealth />
        </aside>

        {/* --- CENTER & RIGHT AREA --- */}
        <div className="flex-1 flex flex-col gap-3 min-w-0">
          <div className="flex-[1.8] flex flex-row gap-3 min-h-0">
            <VideoFeed />

            <div className="flex-1 flex flex-col gap-3 min-w-[340px]">
              <LlmPanel />
              <SwarmOverview />
            </div>
          </div>

          <div className="flex-none h-[220px] flex flex-row gap-3">
            <EventTimeline />
            <SystemPerformance />
          </div>
        </div>
      </main>

      <Footer />
      <Overlays />
    </div>
  </ConsoleProvider>
);

export default AegisConsole;
