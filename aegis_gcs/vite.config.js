import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import tailwindcss from '@tailwindcss/vite';

export default defineConfig({
  plugins: [react(), tailwindcss()],
  // 5200 avoids the ports already in use on this machine (5173 = another app).
  server: { host: true, port: 5200 },
});
