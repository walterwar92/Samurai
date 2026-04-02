#!/usr/bin/env python3
"""
perf_monitor_node — Diagnose freezes and performance issues.

Subscribes to ALL robot topics, measures message rates and gaps.
Detects when any topic goes silent (potential freeze).
Logs detailed timing to console and publishes summary to MQTT.

Usage:
    python3 -m pi_nodes.nodes.perf_monitor_node --broker 192.168.4.1

Publishes:
    samurai/{robot_id}/perf/monitor — aggregated stats JSON @ 0.2 Hz
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode

# Topics to monitor with expected minimum rates (Hz)
MONITORED = {
    'odom':        10.0,
    'imu':         30.0,
    'camera':       5.0,
    'range':        3.0,
    'cmd_vel':      0.0,   # 0 = optional, no rate warning
    'battery':      0.1,
    'temperature':  0.1,
    'watchdog':     0.3,
    'ball_detection': 0.0,  # laptop topic, may not exist
}

# Gap thresholds (seconds) — warn if no message for this long
GAP_WARN_S = 3.0
GAP_ERROR_S = 10.0

REPORT_INTERVAL = 5.0  # summary every 5s


class PerfMonitorNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('perf_monitor', **kwargs)

        self._stats = {}
        for topic in MONITORED:
            self._stats[topic] = {
                'count': 0,
                'bytes': 0,
                'last_ts': 0.0,
                'max_gap_ms': 0.0,
                'last_gap_ms': 0.0,
                'min_rate': MONITORED[topic],
            }
            self.subscribe(topic, self._make_cb(topic), parse_json=False)

        # Also subscribe to perf reports from other nodes
        self.subscribe_raw(
            f'samurai/{self._robot_id}/perf/+',
            self._perf_cb, parse_json=True)

        self._node_perfs = {}
        self._start_time = time.monotonic()

        self.create_timer(REPORT_INTERVAL, self._report)
        self.log_info('Performance monitor started — tracking %d topics', len(MONITORED))

    def _make_cb(self, topic_name):
        def _cb(topic, data):
            now = time.monotonic()
            s = self._stats[topic_name]
            s['count'] += 1
            s['bytes'] += len(data) if isinstance(data, (bytes, bytearray)) else len(str(data))

            if s['last_ts'] > 0:
                gap_ms = (now - s['last_ts']) * 1000
                s['last_gap_ms'] = gap_ms
                if gap_ms > s['max_gap_ms']:
                    s['max_gap_ms'] = gap_ms
                    if gap_ms > GAP_ERROR_S * 1000:
                        self.log_error(
                            'FREEZE %s — gap %.0fms (%.1fs!)',
                            topic_name, gap_ms, gap_ms / 1000)
                    elif gap_ms > GAP_WARN_S * 1000:
                        self.log_warn(
                            'GAP %s — %.0fms since last message',
                            topic_name, gap_ms)
            s['last_ts'] = now
        return _cb

    def _perf_cb(self, topic, data):
        if isinstance(data, dict):
            node = data.get('node', topic.split('/')[-1])
            self._node_perfs[node] = data

    def _report(self):
        now = time.monotonic()
        elapsed = now - self._start_time
        if elapsed < 5.0:
            return  # grace period

        lines = ['─── PERF MONITOR ───']
        report = {}

        for topic, s in self._stats.items():
            rate = s['count'] / max(REPORT_INTERVAL, 0.1)
            avg_bytes = s['bytes'] / max(s['count'], 1)
            alive = s['last_ts'] > 0 and (now - s['last_ts']) < GAP_WARN_S

            status = 'OK' if alive else ('DEAD' if s['last_ts'] > 0 else 'NEVER')
            if s['min_rate'] > 0 and rate < s['min_rate'] * 0.5 and alive:
                status = 'SLOW'

            line = (f'  {topic:20s} {status:5s} | '
                    f'rate={rate:5.1f}Hz  avg={avg_bytes:6.0f}B  '
                    f'gap_max={s["max_gap_ms"]:7.0f}ms')
            lines.append(line)

            report[topic] = {
                'status': status,
                'rate_hz': round(rate, 1),
                'avg_bytes': round(avg_bytes),
                'max_gap_ms': round(s['max_gap_ms']),
                'count': s['count'],
            }

            # Reset for next interval
            s['count'] = 0
            s['bytes'] = 0
            s['max_gap_ms'] = 0.0

        # Node perf summaries (from other nodes' _log_perf_report)
        if self._node_perfs:
            lines.append('  ── Node perf reports ──')
            for node, p in self._node_perfs.items():
                st = p.get('slow_timers', 0)
                sc = p.get('slow_callbacks', 0)
                sp = p.get('slow_publishes', 0)
                dc = p.get('mqtt_disconnects', 0)
                tm = p.get('timer_max_ms', 0)
                if st + sc + sp + dc > 0:
                    lines.append(
                        f'  {node:20s} slow_t={st} slow_cb={sc} '
                        f'slow_pub={sp} dc={dc} timer_max={tm:.0f}ms')

        for line in lines:
            self._log.info(line)

        # Publish summary
        if self._mqtt_connected:
            report['ts'] = time.time()
            self.publish('perf/monitor', report)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Performance monitor')
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    args = parser.parse_args()
    node = PerfMonitorNode(broker=args.broker, port=args.port,
                           robot_id=args.robot_id)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
