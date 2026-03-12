#!/usr/bin/env python3
"""
MQTT Diagnostic Tool — полная проверка связи ноутбук ↔ робот.

Подключается к MQTT-брокеру на Pi, подписывается на все топики,
отправляет тестовые команды, логирует всё в консоль.

Использование:
    python3 tools/mqtt_diagnostic.py <BROKER_IP> [--robot-id robot1]

Пример:
    python3 tools/mqtt_diagnostic.py 100.99.98.32
    python3 tools/mqtt_diagnostic.py 10.251.191.41
"""

import argparse
import json
import sys
import time
import threading
from datetime import datetime
from collections import defaultdict

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("ERROR: paho-mqtt не установлен. pip install paho-mqtt")
    sys.exit(1)


# ═══════════════════════════════════════════════════════════════
# Конфигурация
# ═══════════════════════════════════════════════════════════════

SUBSCRIBE_TIMEOUT = 15      # секунд ждать данные с каждого топика
COMMAND_DELAY = 1.0         # задержка между командами
TOTAL_LISTEN_TIME = 20      # секунд слушать все топики

# Все известные MQTT-топики Samurai
SENSOR_TOPICS = [
    'odom',
    'imu',
    'range',
    'camera',
    'battery',
    'temperature',
    'claw/state',
    'watchdog',
]

COMMAND_TOPICS = [
    'cmd_vel',
    'speed_profile',
    'claw/command',
    'laser/command',
    'voice_command',
    'goal_pose',
]

FSM_TOPICS = [
    'status',
    'ball_detection',
    'detections',
    'gesture/command',
]

MULTI_ROBOT_TOPICS = [
    'call_robot',
    'patrol/command',
    'follow_me/command',
    'path_recorder/command',
]

ALL_TOPICS = SENSOR_TOPICS + COMMAND_TOPICS + FSM_TOPICS + MULTI_ROBOT_TOPICS


# ═══════════════════════════════════════════════════════════════
# Диагностика
# ═══════════════════════════════════════════════════════════════

class MqttDiagnostic:
    def __init__(self, broker: str, port: int, robot_id: str):
        self.broker = broker
        self.port = port
        self.robot_id = robot_id
        self.prefix = f"samurai/{robot_id}"

        self.client = mqtt.Client(
            client_id=f"diagnostic-{int(time.time())}",
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        )
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect

        self.connected = False
        self.connect_rc = None
        self.messages = defaultdict(list)  # topic -> [(timestamp, payload_preview)]
        self.msg_counts = defaultdict(int)
        self.first_msg_time = {}
        self.lock = threading.Lock()
        self.start_time = None

    # ── MQTT callbacks ──────────────────────────────────────

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        self.connected = (rc == 0)
        self.connect_rc = rc
        if rc == 0:
            log_ok(f"MQTT подключен к {self.broker}:{self.port}")
            # Подписка на ВСЕ топики робота
            wildcard = f"{self.prefix}/#"
            client.subscribe(wildcard, qos=0)
            log_info(f"Подписка: {wildcard}")
        else:
            log_err(f"MQTT ошибка подключения rc={rc}")

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload
        now = time.time()

        # Определяем тип данных
        short_topic = topic.replace(f"{self.prefix}/", "")

        with self.lock:
            self.msg_counts[short_topic] += 1
            count = self.msg_counts[short_topic]

            if short_topic not in self.first_msg_time:
                self.first_msg_time[short_topic] = now

            # Сохраняем preview (не для binary)
            if short_topic == 'camera':
                preview = f"<binary {len(payload)} bytes>"
            else:
                try:
                    preview = payload.decode('utf-8')[:200]
                except Exception:
                    preview = f"<binary {len(payload)} bytes>"

            self.messages[short_topic].append((now, preview))

            # Логируем первые 3 сообщения и потом каждое 10-е
            if count <= 3 or count % 10 == 0:
                elapsed = now - self.start_time if self.start_time else 0
                if short_topic == 'camera':
                    log_data(f"[{elapsed:6.1f}s] {short_topic}: frame #{count}, {len(payload)} bytes")
                else:
                    log_data(f"[{elapsed:6.1f}s] {short_topic} #{count}: {preview[:120]}")

    def _on_disconnect(self, client, userdata, rc, *args):
        self.connected = False
        if rc != 0:
            log_warn(f"MQTT отключён неожиданно (rc={rc})")

    # ── Подключение ─────────────────────────────────────────

    def connect(self) -> bool:
        log_header("ПОДКЛЮЧЕНИЕ К MQTT BROKER")
        log_info(f"Broker: {self.broker}:{self.port}")
        log_info(f"Robot ID: {self.robot_id}")
        log_info(f"Prefix: {self.prefix}")

        try:
            self.client.connect(self.broker, self.port, keepalive=60)
            self.client.loop_start()
        except Exception as e:
            log_err(f"Не удалось подключиться: {e}")
            return False

        # Ждём подключения
        for _ in range(50):
            if self.connected:
                return True
            time.sleep(0.1)

        log_err(f"Таймаут подключения (5с). rc={self.connect_rc}")
        return False

    # ── Слушаем все топики ──────────────────────────────────

    def listen_all(self, duration: int):
        log_header(f"СЛУШАЕМ ВСЕ ТОПИКИ ({duration} секунд)")
        self.start_time = time.time()
        log_info("Ожидаем данные от робота...")
        log_info("(камера, ультразвук, IMU, одометрия, батарея, FSM...)")
        print()

        try:
            time.sleep(duration)
        except KeyboardInterrupt:
            log_warn("Прервано пользователем")

    # ── Отправка команд ─────────────────────────────────────

    def send_commands(self):
        log_header("ТЕСТОВЫЕ КОМАНДЫ")

        tests = [
            # (описание, топик, payload)
            # ── cmd_vel: motor_node ожидает {linear_x, angular_z} ──
            (
                "cmd_vel: стоп (linear_x=0, angular_z=0)",
                "cmd_vel",
                json.dumps({"linear_x": 0.0, "angular_z": 0.0}),
            ),
            (
                "cmd_vel: вперёд медленно (linear_x=0.1)",
                "cmd_vel",
                json.dumps({"linear_x": 0.1, "angular_z": 0.0}),
            ),
            (
                "cmd_vel: стоп",
                "cmd_vel",
                json.dumps({"linear_x": 0.0, "angular_z": 0.0}),
            ),
            # ── speed_profile: fsm_node ожидает строку ──
            (
                "speed_profile: slow",
                "speed_profile",
                "slow",
            ),
            # ── voice_command: fsm_node ожидает строку ──
            (
                "voice_command: статус",
                "voice_command",
                "статус",
            ),
            # ── claw: servo_node ожидает plain string "open"/"close" ──
            (
                "claw/command: open",
                "claw/command",
                "open",
            ),
            (
                "claw/command: close (через 1с)",
                "claw/command",
                "close",
            ),
            # ── laser: laser_node ожидает true/false ──
            (
                "laser/command: on (true)",
                "laser/command",
                "true",
            ),
            (
                "laser/command: off (false)",
                "laser/command",
                "false",
            ),
            # ── detections heartbeat: чтобы fallback_nav не включался ──
            (
                "detections: heartbeat (пустой массив — laptop alive)",
                "detections",
                json.dumps([]),
            ),
        ]

        for desc, topic, payload in tests:
            full_topic = f"{self.prefix}/{topic}"
            log_info(f"TX → {full_topic}")
            log_info(f"     {desc}")
            log_info(f"     payload: {payload}")

            result = self.client.publish(full_topic, payload, qos=0)
            if result.rc == 0:
                log_ok(f"  Отправлено (mid={result.mid})")
            else:
                log_err(f"  Ошибка отправки rc={result.rc}")

            time.sleep(COMMAND_DELAY)
            print()

    # ── Проверка отдельных данных ───────────────────────────

    def check_specific_data(self):
        log_header("ПРОВЕРКА КОНКРЕТНЫХ ДАННЫХ")

        checks = {
            'odom': 'Одометрия (x, y, yaw, скорость)',
            'imu': 'IMU (ориентация, гироскоп)',
            'range': 'Ультразвук (расстояние в метрах)',
            'camera': 'Камера (JPEG кадры)',
            'battery': 'Батарея (напряжение)',
            'temperature': 'Температура CPU',
            'status': 'FSM статус (состояние робота)',
            'watchdog': 'Watchdog (живые/мёртвые ноды)',
            'claw/state': 'Клешня (открыта/закрыта)',
            'ball_detection': 'Детекция мяча',
            'detections': 'YOLO детекции',
        }

        for topic, desc in checks.items():
            with self.lock:
                count = self.msg_counts.get(topic, 0)
                msgs = self.messages.get(topic, [])

            if count > 0:
                last_preview = msgs[-1][1] if msgs else "?"
                log_ok(f"{topic}: {count} сообщений — {desc}")
                log_info(f"  Последнее: {last_preview[:150]}")
            else:
                log_err(f"{topic}: 0 сообщений — {desc}")
            print()

    # ── Итоговый отчёт ──────────────────────────────────────

    def report(self):
        log_header("ИТОГОВЫЙ ОТЧЁТ")

        with self.lock:
            total = sum(self.msg_counts.values())
            topics_received = sorted(self.msg_counts.keys())
            topics_missing = [t for t in SENSOR_TOPICS + FSM_TOPICS
                              if t not in self.msg_counts]

        print(f"  Broker:           {self.broker}:{self.port}")
        print(f"  Robot ID:         {self.robot_id}")
        print(f"  Подключение:      {'OK' if self.connected else 'FAIL'}")
        print(f"  Всего сообщений:  {total}")
        print(f"  Активных топиков: {len(topics_received)}")
        print()

        if topics_received:
            log_ok("Топики с данными:")
            for t in topics_received:
                c = self.msg_counts[t]
                msgs = self.messages[t]
                if msgs:
                    first_t = msgs[0][0] - self.start_time
                    last_t = msgs[-1][0] - self.start_time
                    rate = c / max(last_t - first_t, 0.1) if c > 1 else 0
                    print(f"    {t:30s}  {c:5d} msgs  ({rate:.1f} Hz)")
                else:
                    print(f"    {t:30s}  {c:5d} msgs")
            print()

        if topics_missing:
            log_err("Топики БЕЗ данных (возможные проблемы):")
            for t in topics_missing:
                print(f"    ✗ {t}")
            print()

        # Конкретные рекомендации
        log_header("РЕКОМЕНДАЦИИ")

        if 'camera' not in self.msg_counts:
            log_warn("Камера не отправляет кадры → camera_node на Pi не работает?")
        if 'range' not in self.msg_counts:
            log_warn("Ультразвук не отправляет данные → ultrasonic_node на Pi?")
        if 'odom' not in self.msg_counts:
            log_warn("Одометрия не приходит → motor_node на Pi?")
        if 'status' not in self.msg_counts:
            log_warn("FSM статус не приходит → fsm_node на Pi?")
        if 'imu' not in self.msg_counts:
            log_warn("IMU не отправляет данные → imu_node на Pi?")

        if total == 0:
            log_err("НИКАКИХ данных не получено!")
            log_warn("Проверь:")
            log_warn("  1. robot_launcher.py запущен на Pi?")
            log_warn("  2. MQTT broker (mosquitto) работает на Pi?")
            log_warn("  3. Правильный IP broker-а?")
            log_warn(f"  4. Firewall не блокирует порт {self.port}?")

    # ── Disconnect ──────────────────────────────────────────

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()
        log_info("MQTT отключён")


# ═══════════════════════════════════════════════════════════════
# Логирование (цветное)
# ═══════════════════════════════════════════════════════════════

GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
CYAN = '\033[96m'
BOLD = '\033[1m'
DIM = '\033[2m'
NC = '\033[0m'


def ts():
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]


def log_header(text):
    print()
    print(f"{BOLD}{'═' * 60}{NC}")
    print(f"{BOLD}  {text}{NC}")
    print(f"{BOLD}{'═' * 60}{NC}")
    print()


def log_ok(text):
    print(f"  {GREEN}[✓]{NC} {text}")


def log_err(text):
    print(f"  {RED}[✗]{NC} {text}")


def log_warn(text):
    print(f"  {YELLOW}[!]{NC} {text}")


def log_info(text):
    print(f"  {CYAN}[→]{NC} {text}")


def log_data(text):
    print(f"  {DIM}{ts()}{NC} {text}")


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='MQTT Diagnostic Tool для Samurai Robot')
    parser.add_argument('broker', help='IP адрес MQTT broker (Pi)')
    parser.add_argument('--port', type=int, default=1883, help='MQTT порт (default: 1883)')
    parser.add_argument('--robot-id', default='robot1', help='Robot ID (default: robot1)')
    parser.add_argument('--listen', type=int, default=20, help='Секунд слушать (default: 20)')
    parser.add_argument('--no-commands', action='store_true', help='Не отправлять тестовые команды')
    args = parser.parse_args()

    print()
    print(f"{BOLD}  ╔═══════════════════════════════════════════╗{NC}")
    print(f"{BOLD}  ║   SAMURAI MQTT DIAGNOSTIC TOOL            ║{NC}")
    print(f"{BOLD}  ║   Полная проверка связи ноутбук ↔ робот   ║{NC}")
    print(f"{BOLD}  ╚═══════════════════════════════════════════╝{NC}")

    diag = MqttDiagnostic(args.broker, args.port, args.robot_id)

    # 1. Подключение
    if not diag.connect():
        log_err("Не удалось подключиться к MQTT. Выход.")
        sys.exit(1)

    try:
        # 2. Слушаем все топики
        diag.listen_all(args.listen)

        # 3. Проверка данных
        diag.check_specific_data()

        # 4. Отправляем тестовые команды
        if not args.no_commands:
            diag.send_commands()

            # Слушаем ещё 5 секунд после команд
            log_info("Ожидаем ответы на команды (5с)...")
            time.sleep(5)

        # 5. Итоговый отчёт
        diag.report()

    except KeyboardInterrupt:
        print()
        log_warn("Прервано — генерируем отчёт...")
        diag.report()
    finally:
        diag.disconnect()


if __name__ == '__main__':
    main()
