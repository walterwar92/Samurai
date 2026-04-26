#!/usr/bin/env python3
"""
perf_monitor_node — Diagnose freezes and performance issues.

Subscribes to ALL robot topics, measures message rates and gaps.
Detects when any topic goes silent (potential freeze).
Logs detailed timing to console and publishes summary to MQTT.

Also exposes an HTTP server for external monitoring:
    GET /health  — JSON, returns 200 if all critical topics alive, 503 otherwise
    GET /metrics — Prometheus text exposition format (for Grafana/Alertmanager)

Usage:
    python3 -m pi_nodes.nodes.perf_monitor_node --broker 192.168.4.1

Publishes:
    samurai/{robot_id}/perf/monitor — aggregated stats JSON @ 0.2 Hz
"""

import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

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

# Critical topics for /health endpoint — same set the watchdog uses for e-stop.
# /health returns 503 if any are dead, 200 otherwise.
HEALTH_CRITICAL_TOPICS = {'odom', 'imu'}

DEFAULT_HTTP_PORT = 8000


class PerfMonitorNode(MqttNode):
    def __init__(self, http_port: int = DEFAULT_HTTP_PORT, **kwargs):
        super().__init__('perf_monitor', **kwargs)

        self._stats = {}
        # Cumulative counters (Prometheus convention: counters never reset).
        # The per-interval _stats counters DO reset every REPORT_INTERVAL.
        self._total_count = {topic: 0 for topic in MONITORED}
        self._total_bytes = {topic: 0 for topic in MONITORED}
        self._last_rate_hz = {topic: 0.0 for topic in MONITORED}
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
        self._stats_lock = threading.Lock()

        self.create_timer(REPORT_INTERVAL, self._report)

        # HTTP server for /health and /metrics. Daemon thread so node shutdown
        # doesn't block on it. Bind to all interfaces — the Pi's MQTT and HTTP
        # are on the same trusted LAN; the dashboard scrapes the metrics from
        # the laptop side.
        self._http_port = http_port
        self._http_server = None
        if http_port > 0:
            self._start_http_server()

        self.log_info('Performance monitor started — tracking %d topics, '
                      'HTTP :%d', len(MONITORED), http_port)

    def _make_cb(self, topic_name):
        def _cb(topic, data):
            now = time.monotonic()
            size = len(data) if isinstance(data, (bytes, bytearray)) else len(str(data))
            with self._stats_lock:
                s = self._stats[topic_name]
                s['count'] += 1
                s['bytes'] += size
                self._total_count[topic_name] += 1
                self._total_bytes[topic_name] += size

                if s['last_ts'] > 0:
                    gap_ms = (now - s['last_ts']) * 1000
                    s['last_gap_ms'] = gap_ms
                    if gap_ms > s['max_gap_ms']:
                        s['max_gap_ms'] = gap_ms
                        log_call = None
                        if gap_ms > GAP_ERROR_S * 1000:
                            log_call = (self.log_error,
                                        'FREEZE %s — gap %.0fms (%.1fs!)',
                                        (topic_name, gap_ms, gap_ms / 1000))
                        elif gap_ms > GAP_WARN_S * 1000:
                            log_call = (self.log_warn,
                                        'GAP %s — %.0fms since last message',
                                        (topic_name, gap_ms))
                s['last_ts'] = now
            # Log outside the lock so logging IO can't deadlock with reader threads.
            if 'log_call' in locals() and log_call:
                fn, fmt, args = log_call
                fn(fmt, *args)
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

        with self._stats_lock:
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
                self._last_rate_hz[topic] = rate

                # Reset for next interval (cumulative counters in _total_* persist)
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

    # ── HTTP server: /health, /metrics ──────────────────────────
    def _start_http_server(self):
        node_ref = self

        class Handler(BaseHTTPRequestHandler):
            # Silence default access log spam — we don't need every scrape line.
            def log_message(self, format, *args):
                pass

            def do_GET(self):
                if self.path == '/health':
                    body, code = node_ref._render_health()
                    ctype = 'application/json'
                elif self.path == '/metrics':
                    body = node_ref._render_metrics()
                    code = 200
                    # Prometheus exposition format MIME (text/plain version 0.0.4)
                    ctype = 'text/plain; version=0.0.4; charset=utf-8'
                else:
                    body = b'Not Found'
                    code = 404
                    ctype = 'text/plain'
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        try:
            self._http_server = ThreadingHTTPServer(('0.0.0.0', self._http_port),
                                                    Handler)
        except OSError as e:
            self.log_error('HTTP server bind failed (port %d): %s — disabled',
                           self._http_port, e)
            self._http_server = None
            return

        t = threading.Thread(target=self._http_server.serve_forever,
                             name='perf_monitor_http', daemon=True)
        t.start()

    def _render_health(self):
        now = time.monotonic()
        with self._stats_lock:
            critical_status = {}
            all_ok = True
            for topic in HEALTH_CRITICAL_TOPICS:
                s = self._stats.get(topic, {})
                last = s.get('last_ts', 0.0)
                alive = last > 0 and (now - last) < GAP_WARN_S
                critical_status[topic] = {
                    'alive': alive,
                    'age_s': round(now - last, 2) if last > 0 else None,
                }
                if not alive:
                    all_ok = False
            uptime = now - self._start_time
        body = {
            'status': 'ok' if all_ok else 'degraded',
            'uptime_s': round(uptime, 1),
            'critical': critical_status,
        }
        code = 200 if all_ok else 503
        return json.dumps(body, separators=(',', ':')).encode('utf-8'), code

    def _render_metrics(self) -> bytes:
        """Prometheus text exposition format. Snake_case names, _total counters."""
        now = time.monotonic()
        rid = self._robot_id
        out = []

        def metric(name: str, mtype: str, help_text: str, samples: list[str]):
            out.append(f'# HELP {name} {help_text}')
            out.append(f'# TYPE {name} {mtype}')
            out.extend(samples)

        with self._stats_lock:
            uptime = now - self._start_time
            metric('samurai_uptime_seconds', 'gauge',
                   'Seconds since perf_monitor started',
                   [f'samurai_uptime_seconds{{robot_id="{rid}"}} {uptime:.1f}'])

            count_samples = []
            bytes_samples = []
            rate_samples = []
            alive_samples = []
            gap_samples = []
            for topic in MONITORED:
                lbl = f'{{robot_id="{rid}",topic="{topic}"}}'
                s = self._stats[topic]
                last = s['last_ts']
                alive = 1 if (last > 0 and (now - last) < GAP_WARN_S) else 0
                age_s = (now - last) if last > 0 else -1.0

                count_samples.append(f'samurai_topic_messages_total{lbl} {self._total_count[topic]}')
                bytes_samples.append(f'samurai_topic_bytes_total{lbl} {self._total_bytes[topic]}')
                rate_samples.append(f'samurai_topic_rate_hz{lbl} {self._last_rate_hz[topic]:.2f}')
                alive_samples.append(f'samurai_topic_alive{lbl} {alive}')
                gap_samples.append(f'samurai_topic_age_seconds{lbl} {age_s:.2f}')

            metric('samurai_topic_messages_total', 'counter',
                   'Total messages received per topic since startup',
                   count_samples)
            metric('samurai_topic_bytes_total', 'counter',
                   'Total payload bytes received per topic since startup',
                   bytes_samples)
            metric('samurai_topic_rate_hz', 'gauge',
                   'Messages per second observed in the last report interval',
                   rate_samples)
            metric('samurai_topic_alive', 'gauge',
                   '1 if topic seen within GAP_WARN window, 0 otherwise',
                   alive_samples)
            metric('samurai_topic_age_seconds', 'gauge',
                   'Seconds since last message; -1 if never seen',
                   gap_samples)

            # Per-node perf samples (slow callbacks etc., from _node_perfs)
            slow_t = []
            slow_cb = []
            slow_pub = []
            mqtt_dc = []
            for node, p in self._node_perfs.items():
                lbl = f'{{robot_id="{rid}",node="{node}"}}'
                slow_t.append(f'samurai_node_slow_timers{lbl} {p.get("slow_timers", 0)}')
                slow_cb.append(f'samurai_node_slow_callbacks{lbl} {p.get("slow_callbacks", 0)}')
                slow_pub.append(f'samurai_node_slow_publishes{lbl} {p.get("slow_publishes", 0)}')
                mqtt_dc.append(f'samurai_node_mqtt_disconnects{lbl} {p.get("mqtt_disconnects", 0)}')
            if slow_t:
                metric('samurai_node_slow_timers', 'gauge',
                       'Slow timer ticks in the last node report interval', slow_t)
                metric('samurai_node_slow_callbacks', 'gauge',
                       'Slow MQTT callbacks in the last node report interval', slow_cb)
                metric('samurai_node_slow_publishes', 'gauge',
                       'Slow publishes in the last node report interval', slow_pub)
                metric('samurai_node_mqtt_disconnects', 'gauge',
                       'MQTT disconnects in the last node report interval', mqtt_dc)

        body = '\n'.join(out) + '\n'
        return body.encode('utf-8')

    def on_shutdown(self):
        if self._http_server:
            try:
                self._http_server.shutdown()
            except Exception:
                pass


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Performance monitor')
    parser.add_argument('--broker', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--robot-id', default='robot1')
    parser.add_argument('--http-port', type=int, default=DEFAULT_HTTP_PORT,
                        help='HTTP /health + /metrics port (0 = disabled)')
    args = parser.parse_args()
    node = PerfMonitorNode(broker=args.broker, port=args.port,
                           robot_id=args.robot_id,
                           http_port=args.http_port)
    node.start()
    node.spin()


if __name__ == '__main__':
    main()
