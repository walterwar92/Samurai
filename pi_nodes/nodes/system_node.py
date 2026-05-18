#!/usr/bin/env python3
"""
system_node — Handle global system commands from dashboard.

Subscribed:
    samurai/{robot_id}/system/shutdown  — kill robot_launcher (всё умрёт каскадно)
"""

import os
import signal
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from pi_nodes.mqtt_node import MqttNode


class SystemNode(MqttNode):
    def __init__(self, **kwargs):
        super().__init__('system_node', **kwargs)
        self.subscribe('system/shutdown', self._on_shutdown)
        self._log.info('SystemNode ready: подписан на system/shutdown')

    def _on_shutdown(self, payload):
        source = (payload or {}).get('source', 'unknown')
        self._log.warning(
            'Shutdown requested from %s — SIGTERM в robot_launcher через 200мс',
            source,
        )
        threading.Timer(0.2, self._kill_launcher).start()

    def _kill_launcher(self):
        parent_pid = os.getppid()
        self._log.warning('SIGTERM → PID %d (launcher)', parent_pid)
        os.kill(parent_pid, signal.SIGTERM)


if __name__ == '__main__':
    SystemNode().run()
