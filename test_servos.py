#!/usr/bin/env python3
"""
test_servos.py — Тест всех 5 серво-приводов по очереди.

Серво-маппинг (PCA9685):
  CH0 — Основание клешни (arm joint 1)
  CH1 — Сустав 2         (arm joint 2)
  CH2 — Сустав 3         (arm joint 3)
  CH3 — Клешня           (arm joint 4)
  CH4 — Голова/камера     (head)

Режимы:
  --api http://localhost:5000   — через HTTP API (симулятор или dashboard)
  --mqtt 192.168.1.100          — напрямую через MQTT (реальный робот)

Использование:
  python test_servos.py --api http://localhost:5000
  python test_servos.py --mqtt 192.168.1.100
  python test_servos.py --mqtt 127.0.0.1 --servo 4   # только голова
"""

import argparse
import json
import time
import sys

# ── Названия серво ──────────────────────────────────────────────
SERVO_NAMES = {
    0: 'CH0 — Основание клешни',
    1: 'CH1 — Сустав 2',
    2: 'CH2 — Сустав 3',
    3: 'CH3 — Клешня (захват)',
    4: 'CH4 — Голова (камера)',
}

# ── Цвета для терминала ─────────────────────────────────────────
G = '\033[92m'   # green
Y = '\033[93m'   # yellow
C = '\033[96m'   # cyan
R = '\033[91m'   # red
B = '\033[1m'    # bold
N = '\033[0m'    # reset


def log(msg):
    print(f'  {G}[✓]{N} {msg}')


def log_move(servo_id, angle):
    name = SERVO_NAMES.get(servo_id, f'CH{servo_id}')
    bar_len = int(angle / 180 * 30)
    bar = '█' * bar_len + '░' * (30 - bar_len)
    print(f'  {C}[→]{N} {name:30s}  {Y}{angle:6.1f}°{N}  [{bar}]')


# ═══════════════════════════════════════════════════════════════
#  HTTP API mode (симулятор / dashboard)
# ═══════════════════════════════════════════════════════════════

def api_set_arm_joint(base_url, joint_1idx, angle):
    """Set arm joint via POST /api/actuators/arm. joint is 1-indexed."""
    import urllib.request
    url = f'{base_url}/api/actuators/arm'
    data = json.dumps({'joint': joint_1idx, 'angle': angle}).encode()
    req = urllib.request.Request(url, data=data,
                                headers={'Content-Type': 'application/json'})
    try:
        resp = urllib.request.urlopen(req, timeout=5)
        return resp.status == 200
    except Exception as e:
        print(f'  {R}[✗]{N} API error: {e}')
        return False


def api_set_head(base_url, angle):
    """Set head angle via POST /api/actuators/head."""
    import urllib.request
    url = f'{base_url}/api/actuators/head'
    data = json.dumps({'angle': angle}).encode()
    req = urllib.request.Request(url, data=data,
                                headers={'Content-Type': 'application/json'})
    try:
        resp = urllib.request.urlopen(req, timeout=5)
        return resp.status == 200
    except Exception as e:
        print(f'  {R}[✗]{N} API error: {e}')
        return False


def api_get_state(base_url):
    """Get current servo states."""
    import urllib.request
    try:
        # Arm
        resp = urllib.request.urlopen(f'{base_url}/api/actuators/arm', timeout=5)
        arm = json.loads(resp.read())
        # Head
        resp = urllib.request.urlopen(f'{base_url}/api/actuators/head', timeout=5)
        head = json.loads(resp.read())
        return arm, head
    except Exception as e:
        print(f'  {R}[✗]{N} Не удалось получить стейт: {e}')
        return None, None


def test_via_api(base_url, servo_filter=None, speed=0.5):
    """Тест серво через HTTP API."""
    print(f'\n{B}  ═══ Тест серво через API: {base_url} ═══{N}\n')

    # Проверка соединения
    arm, head = api_get_state(base_url)
    if arm is None:
        print(f'  {R}[✗]{N} Не удалось подключиться к {base_url}')
        return False
    log(f'Подключено к {base_url}')
    print(f'  {C}    Arm:{N} {arm}')
    print(f'  {C}    Head:{N} {head}')
    print()

    servos_to_test = list(range(5))
    if servo_filter is not None:
        servos_to_test = [servo_filter]

    for servo_id in servos_to_test:
        name = SERVO_NAMES[servo_id]
        print(f'\n  {B}── Тест: {name} ──{N}')

        # Последовательность: home → min → max → home
        angles = [90, 30, 150, 90]

        for angle in angles:
            if servo_id <= 3:
                # Arm joint (1-indexed in API)
                ok = api_set_arm_joint(base_url, servo_id + 1, angle)
            else:
                # Head
                ok = api_set_head(base_url, angle)

            if ok:
                log_move(servo_id, angle)
            else:
                print(f'  {R}[✗]{N} Ошибка отправки!')

            time.sleep(speed)

    # Финал: всё в home
    print(f'\n  {B}── Возврат в HOME ──{N}')
    for i in range(4):
        api_set_arm_joint(base_url, i + 1, 90)
    api_set_head(base_url, 90)
    log('Все серво → 90° (HOME)')

    print(f'\n  {G}{B}✓ Тест завершён!{N}\n')
    return True


# ═══════════════════════════════════════════════════════════════
#  MQTT mode (реальный робот)
# ═══════════════════════════════════════════════════════════════

def test_via_mqtt(broker, port=1883, robot_id='robot1',
                  servo_filter=None, speed=0.5):
    """Тест серво через MQTT напрямую к Pi."""
    try:
        import paho.mqtt.client as mqtt_client
    except ImportError:
        print(f'  {R}[✗]{N} paho-mqtt не установлен: pip install paho-mqtt')
        return False

    prefix = f'samurai/{robot_id}'
    print(f'\n{B}  ═══ Тест серво через MQTT: {broker}:{port} ═══{N}')
    print(f'  {C}    Prefix:{N} {prefix}')
    print()

    # Подключение
    connected = [False]
    latest_arm = [None]
    latest_head = [None]

    def on_connect(client, userdata, flags, rc, *args):
        if rc == 0:
            connected[0] = True
            client.subscribe(f'{prefix}/arm/state')
            client.subscribe(f'{prefix}/head/state')
        else:
            print(f'  {R}[✗]{N} MQTT connect failed: rc={rc}')

    def on_message(client, userdata, msg):
        try:
            data = json.loads(msg.payload)
            if msg.topic.endswith('arm/state'):
                latest_arm[0] = data
            elif msg.topic.endswith('head/state'):
                latest_head[0] = data
        except Exception:
            pass

    client = mqtt_client.Client(client_id='servo_test')
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(broker, port, keepalive=10)
    except Exception as e:
        print(f'  {R}[✗]{N} Не удалось подключиться: {e}')
        return False

    client.loop_start()

    # Ждём подключения
    for _ in range(30):
        if connected[0]:
            break
        time.sleep(0.1)

    if not connected[0]:
        print(f'  {R}[✗]{N} Таймаут подключения к MQTT')
        client.loop_stop()
        return False

    log(f'MQTT подключён к {broker}:{port}')

    # Ждём первый стейт
    time.sleep(1.0)
    if latest_arm[0]:
        print(f'  {C}    Arm state:{N} {latest_arm[0]}')
    if latest_head[0]:
        print(f'  {C}    Head state:{N} {latest_head[0]}')

    def mqtt_set_arm_joint(joint_1idx, angle):
        payload = json.dumps({'joint': joint_1idx, 'angle': angle})
        client.publish(f'{prefix}/arm/command', payload, qos=1)

    def mqtt_set_head(angle):
        payload = json.dumps({'angle': angle})
        client.publish(f'{prefix}/head/command', payload, qos=1)

    servos_to_test = list(range(5))
    if servo_filter is not None:
        servos_to_test = [servo_filter]

    for servo_id in servos_to_test:
        name = SERVO_NAMES[servo_id]
        print(f'\n  {B}── Тест: {name} ──{N}')

        angles = [90, 30, 150, 90]

        for angle in angles:
            if servo_id <= 3:
                mqtt_set_arm_joint(servo_id + 1, angle)
            else:
                mqtt_set_head(angle)

            log_move(servo_id, angle)
            time.sleep(speed)

            # Показать реальный стейт
            if servo_id <= 3 and latest_arm[0]:
                key = f'j{servo_id + 1}'
                real = latest_arm[0].get(key, '?')
                print(f'           {C}Реальный угол {key}:{N} {real}°')
            elif servo_id == 4 and latest_head[0]:
                real = latest_head[0].get('angle', '?')
                print(f'           {C}Реальный угол:{N} {real}°')

    # Home
    print(f'\n  {B}── Возврат в HOME ──{N}')
    for i in range(4):
        mqtt_set_arm_joint(i + 1, 90)
    mqtt_set_head(90)
    time.sleep(0.5)
    log('Все серво → 90° (HOME)')

    client.loop_stop()
    client.disconnect()

    print(f'\n  {G}{B}✓ Тест завершён!{N}\n')
    return True


# ═══════════════════════════════════════════════════════════════
#  Интерактивный режим
# ═══════════════════════════════════════════════════════════════

def interactive_mode(set_arm_fn, set_head_fn):
    """Ручное управление серво из консоли."""
    print(f'\n{B}  ═══ Интерактивный режим ═══{N}')
    print(f'  Команды:')
    print(f'    {Y}0-3 <angle>{N}  — задать угол сустава руки (0-180)')
    print(f'    {Y}4 <angle>{N}    — задать угол головы (0-180)')
    print(f'    {Y}home{N}         — все серво в 90°')
    print(f'    {Y}wave{N}         — помахать клешнёй')
    print(f'    {Y}nod{N}          — кивнуть головой')
    print(f'    {Y}q{N}            — выход')
    print()

    while True:
        try:
            cmd = input(f'  {C}servo>{N} ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd in ('q', 'quit', 'exit'):
            break

        if cmd == 'home':
            for i in range(4):
                set_arm_fn(i + 1, 90)
            set_head_fn(90)
            log('HOME — все серво → 90°')
            continue

        if cmd == 'wave':
            print(f'  {Y}Машу клешнёй...{N}')
            for _ in range(3):
                set_arm_fn(4, 30)   # open claw
                time.sleep(0.3)
                set_arm_fn(4, 130)  # close claw
                time.sleep(0.3)
            set_arm_fn(4, 90)
            log('Wave done')
            continue

        if cmd == 'nod':
            print(f'  {Y}Киваю головой...{N}')
            for _ in range(3):
                set_head_fn(60)
                time.sleep(0.3)
                set_head_fn(120)
                time.sleep(0.3)
            set_head_fn(90)
            log('Nod done')
            continue

        parts = cmd.split()
        if len(parts) == 2:
            try:
                servo_id = int(parts[0])
                angle = float(parts[1])
                if servo_id < 0 or servo_id > 4:
                    print(f'  {R}Серво ID должен быть 0-4{N}')
                    continue
                angle = max(0, min(180, angle))
                if servo_id <= 3:
                    set_arm_fn(servo_id + 1, angle)
                else:
                    set_head_fn(angle)
                log_move(servo_id, angle)
            except ValueError:
                print(f'  {R}Формат: <servo_id 0-4> <angle 0-180>{N}')
        else:
            print(f'  {R}Неизвестная команда. Формат: <id> <angle> | home | wave | nod | q{N}')


# ═══════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='Тест серво-приводов робота Samurai',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Примеры:
  python test_servos.py --api http://localhost:5000           # симулятор
  python test_servos.py --mqtt 192.168.1.100                  # реальный робот
  python test_servos.py --mqtt 192.168.1.100 --servo 4        # только голова
  python test_servos.py --api http://localhost:5000 -i         # интерактив
  python test_servos.py --mqtt 192.168.1.100 --speed 1.0      # медленнее
        ''')

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--api', metavar='URL',
                       help='HTTP API URL (например http://localhost:5000)')
    group.add_argument('--mqtt', metavar='BROKER',
                       help='MQTT брокер IP (например 192.168.1.100)')

    parser.add_argument('--port', type=int, default=1883,
                        help='MQTT порт (по умолчанию 1883)')
    parser.add_argument('--robot-id', default='robot1',
                        help='Robot ID (по умолчанию robot1)')
    parser.add_argument('--servo', type=int, choices=[0, 1, 2, 3, 4],
                        help='Тестировать только один серво (0-4)')
    parser.add_argument('--speed', type=float, default=0.5,
                        help='Пауза между движениями в секундах (по умолчанию 0.5)')
    parser.add_argument('-i', '--interactive', action='store_true',
                        help='Интерактивный режим (ручное управление)')

    args = parser.parse_args()

    print(f'\n{B}{C}  ╔═══════════════════════════════════════╗{N}')
    print(f'{B}{C}  ║    SAMURAI — Тест серво-приводов      ║{N}')
    print(f'{B}{C}  ╠═══════════════════════════════════════╣{N}')
    print(f'{B}{C}  ║  CH0  Основание клешни                ║{N}')
    print(f'{B}{C}  ║  CH1  Сустав 2                        ║{N}')
    print(f'{B}{C}  ║  CH2  Сустав 3                        ║{N}')
    print(f'{B}{C}  ║  CH3  Клешня (захват)                 ║{N}')
    print(f'{B}{C}  ║  CH4  Голова (камера)                 ║{N}')
    print(f'{B}{C}  ╚═══════════════════════════════════════╝{N}')

    if args.api:
        if args.interactive:
            interactive_mode(
                lambda j, a: api_set_arm_joint(args.api, j, a),
                lambda a: api_set_head(args.api, a),
            )
        else:
            test_via_api(args.api, servo_filter=args.servo, speed=args.speed)
    else:
        if args.interactive:
            import paho.mqtt.client as mqtt_client
            prefix = f'samurai/{args.robot_id}'
            client = mqtt_client.Client(client_id='servo_test_interactive')
            client.connect(args.mqtt, args.port)
            client.loop_start()

            def mqtt_arm(j, a):
                client.publish(f'{prefix}/arm/command',
                               json.dumps({'joint': j, 'angle': a}), qos=1)

            def mqtt_head(a):
                client.publish(f'{prefix}/head/command',
                               json.dumps({'angle': a}), qos=1)

            interactive_mode(mqtt_arm, mqtt_head)
            client.loop_stop()
            client.disconnect()
        else:
            test_via_mqtt(args.mqtt, port=args.port, robot_id=args.robot_id,
                          servo_filter=args.servo, speed=args.speed)


if __name__ == '__main__':
    main()
