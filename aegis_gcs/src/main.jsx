import React from 'react';
import { createRoot } from 'react-dom/client';
import './index.css';
import AegisConsole from './AegisConsole.jsx';

createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <AegisConsole />
  </React.StrictMode>
);
