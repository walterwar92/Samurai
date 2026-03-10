#!/usr/bin/env python3
"""
robot_launcher.py — Start all Pi-side MQTT nodes.
Replaces ros2 launch robot_pkg robot_bringup.launch.py.

Usage:
    python3 -m pi_nodes.robot_launcher
    python3 pi_nodes/robot_launcher.py --broker 192.168.1.50
    python3 pi_nodes/robot_launcher.py --nodes motor,imu,fsm
"""

import argparse
import importlib
import logging
import multiprocessing
import signal
import sys
import time

logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] [launcher] %(message)s',
)
log = logging.getLogger('launcher')

# {name: 'module.path.ClassName'}
NODE_REGISTRY = {
    'motor':        'pi_nodes.nodes.motor_node.MotorNode',
    'imu':          'pi_nodes.nodes.imu_node.IMUNode',
    'camera':       'pi_nodes.nodes.camera_node.CameraNode',
    'ultrasonic':   'pi_nodes.nodes.ultrasonic_node.UltrasonicNode',
    'battery':      'pi_nodes.nodes.battery_node.BatteryNode',
    'temperature':  'pi_nodes.nodes.temperature_node.TemperatureNode',
    'servo':        'pi_nodes.nodes.servo_node.ServoNode',
    'laser':        'pi_nodes.nodes.laser_node.LaserNode',
    'fsm':          'pi_nodes.nodes.fsm_node.FSMNode',
    'watchdog':     'pi_nodes.nodes.watchdog_node.WatchdogNode',
    'voice':        'pi_nodes.nodes.voice_node.VoiceNode',
    'fallback_nav': 'pi_nodes.nodes.fallback_nav_node.FallbackNavNode',
}

DEFAULT_NODES = [
    'motor', 'imu', 'camera', 'ultrasonic',
    'battery', 'temperature', 'servo', 'laser',
    'fsm', 'watchdog', 'fallback_nav',
]


def _run_node(class_path: str, broker: str, port: int, robot_id: str):
    """Entry point for each child process."""
    module_path, class_name = class_path.rsplit('.', 1)
    module = importlib.import_module(module_path)
    node_class = getattr(module, class_name)
    node = node_class(broker=broker, port=port, robot_id=robot_id)
    node.start()
    node.spin()


def main():
    parser = argparse.ArgumentParser(description='Samurai Robot Launcher')
    parser.add_argument('--broker', default='127.0.0.1',
                        help='MQTT broker IP (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=1883,
                        help='MQTT broker port (default: 1883)')
    parser.add_argument('--robot-id', default='robot1',
                        help='Robot identifier (default: robot1)')
    parser.add_argument('--nodes', default='',
                        help='Comma-separated node names (default: all)')
    args = parser.parse_args()

    node_names = [n.strip() for n in args.nodes.split(',') if n.strip()] \
        if args.nodes else DEFAULT_NODES

    log.info('Starting %d nodes (broker=%s:%d, id=%s)',
             len(node_names), args.broker, args.port, args.robot_id)

    processes: list[tuple[str, multiprocessing.Process]] = []

    for name in node_names:
        if name not in NODE_REGISTRY:
            log.error('Unknown node: %s (available: %s)',
                      name, ', '.join(NODE_REGISTRY))
            continue
        p = multiprocessing.Process(
            target=_run_node,
            args=(NODE_REGISTRY[name], args.broker, args.port, args.robot_id),
            name=f'node_{name}',
            daemon=False,
        )
        p.start()
        processes.append((name, p))
        log.info('  [+] %s (pid=%d)', name, p.pid)
        time.sleep(0.1)  # stagger to avoid I2C bus contention

    # Graceful shutdown on SIGINT/SIGTERM
    def _shutdown(signum, frame):
        log.info('Shutdown signal — stopping all nodes...')
        for name, p in processes:
            if p.is_alive():
                p.terminate()
        for name, p in processes:
            p.join(timeout=5.0)
            if p.is_alive():
                log.warning('Force killing %s', name)
                p.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # Monitor and auto-restart crashed nodes
    while True:
        for i, (name, p) in enumerate(processes):
            if not p.is_alive() and p.exitcode != 0:
                log.error('Node %s crashed (exit=%s) — restarting...',
                          name, p.exitcode)
                new_p = multiprocessing.Process(
                    target=_run_node,
                    args=(NODE_REGISTRY[name], args.broker, args.port,
                          args.robot_id),
                    name=f'node_{name}',
                    daemon=False,
                )
                new_p.start()
                processes[i] = (name, new_p)
                log.info('  [+] %s restarted (pid=%d)', name, new_p.pid)
        time.sleep(2.0)


if __name__ == '__main__':
    main()
