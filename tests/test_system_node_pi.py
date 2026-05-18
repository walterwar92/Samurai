"""Тесты для pi_nodes/nodes/system_node.py.

SystemNode подписывается на samurai/{robot_id}/system/shutdown и
по получении шлёт SIGTERM родителю (robot_launcher).
"""
from __future__ import annotations

import os
import sys
from unittest.mock import patch

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def test_system_node_subscribes_to_shutdown_topic():
    """SystemNode в __init__ подписывается на system/shutdown."""
    from pi_nodes.mqtt_node import MqttNode

    with patch('pi_nodes.mqtt_node.mqtt.Client'), \
         patch.object(MqttNode, 'subscribe') as mock_sub:
        from pi_nodes.nodes.system_node import SystemNode
        SystemNode()
        topics = [c.args[0] for c in mock_sub.call_args_list]
        assert 'system/shutdown' in topics, (
            f'SystemNode должен subscribe на system/shutdown, было: {topics}'
        )


def test_system_node_kills_parent_on_shutdown():
    """При получении shutdown — SystemNode шлёт SIGTERM родителю."""
    import signal as _signal

    with patch('pi_nodes.mqtt_node.mqtt.Client'):
        from pi_nodes.nodes.system_node import SystemNode
        node = SystemNode()

        with patch('pi_nodes.nodes.system_node.os.kill') as mock_kill, \
             patch('pi_nodes.nodes.system_node.os.getppid', return_value=12345):
            # Вызвать handler напрямую — без 200мс таймера
            node._kill_launcher()

        assert mock_kill.call_args.args == (12345, _signal.SIGTERM)
