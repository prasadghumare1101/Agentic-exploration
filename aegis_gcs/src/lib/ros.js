/**
 * Live telemetry over rosbridge.
 *
 * The backend publishes ONE consolidated JSON document on /gcs/consolidated_telemetry at 10 Hz
 * (see aero_gcs backend_services/telemetry_node.py). One topic -> one setState per tick: this is
 * what keeps the console light when PX4 publishes far faster than the browser can paint.
 *
 * Bursts are absorbed in a ref and flushed on a fixed cadence by the consumer, so DDS rate never
 * drives React re-renders.
 */
// roslib ships a namespace, not a default export (same import style aero_gcs uses).
import * as ROSLIB from 'roslib';
import { ROSBRIDGE_URL } from './api.js';

const TELEMETRY_TOPIC = '/gcs/consolidated_telemetry';
const EVENTS_TOPIC = '/gcs/events';

/**
 * Connect to rosbridge and stream telemetry + events.
 * Reconnects on its own; callers just get onState / onEvent / onStatus callbacks.
 */
export function connectRos({ onState, onEvent, onStatus }) {
  let ros = null;
  let retry = null;
  let disposed = false;

  const open = () => {
    if (disposed) return;
    onStatus?.({ status: 'connecting' });
    ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

    ros.on('connection', () => {
      if (disposed) return;
      onStatus?.({ status: 'connected' });

      new ROSLIB.Topic({
        ros,
        name: TELEMETRY_TOPIC,
        messageType: 'std_msgs/String',
        throttle_rate: 100,
        queue_length: 1,
      }).subscribe((m) => {
        try {
          onState?.(JSON.parse(m.data));
        } catch {
          /* ignore a malformed frame rather than killing the stream */
        }
      });

      new ROSLIB.Topic({
        ros,
        name: EVENTS_TOPIC,
        messageType: 'std_msgs/String',
        queue_length: 20,
      }).subscribe((m) => {
        try {
          onEvent?.(JSON.parse(m.data));
        } catch {
          /* ignore */
        }
      });
    });

    // rosbridge fires 'error' on a failed dial; 'close' owns reconnection so the
    // browser console stays clean.
    ros.on('error', () => {});
    ros.on('close', () => {
      if (disposed) return;
      onStatus?.({ status: 'disconnected' });
      retry = setTimeout(open, 2000);
    });
  };

  open();

  return {
    close() {
      disposed = true;
      clearTimeout(retry);
      try {
        ros?.close();
      } catch {
        /* already gone */
      }
    },
  };
}
