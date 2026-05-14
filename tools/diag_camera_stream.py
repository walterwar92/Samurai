#!/usr/bin/env python3
"""
diag_camera_stream.py — пошаговая диагностика H.264 видеопотока робота.

Цепочка видео:
  Pi camera_node (TCP-сервер :8554)
    -> [WiFi / сеть]
    -> dashboard-контейнер (compute_node)
    -> /ws/h264 WebSocket-прокси  (asyncio.open_connection)
    -> браузер (WebCodecs)

Симптом в браузере: «Нет видеопотока / Камера недоступна: TCP connect failed:».
Пустая строка после "failed:" — это asyncio.TimeoutError (у него пустой str()).

Скрипт проверяет КАЖДОЕ звено по отдельности и в конце печатает диагноз.
Запускать ОБА раза — на хосте и внутри контейнера — и сравнить:

  python3 tools/diag_camera_stream.py
  docker exec samurai_compute python3 /root/Samurai/tools/diag_camera_stream.py

Если на хосте OK, а в контейнере FAIL — проблема в сети контейнера
(Docker Desktop / WSL2 NAT, VPN/прокси-туннели на хосте), не в Pi и не в коде.
"""
from __future__ import annotations

import argparse
import asyncio
import errno
import json
import os
import select
import socket
import sys
import threading
import time


def _hr(title: str) -> None:
    pad = max(4, 64 - len(title))
    print(f"\n---- {title} {'-' * pad}")


# Неблокирующий connect_ex возвращает разный код «в процессе» на разных ОС:
# Linux/macOS — EINPROGRESS, Windows — WSAEWOULDBLOCK.
_CONNECT_IN_PROGRESS = {0, errno.EINPROGRESS}
if hasattr(errno, "WSAEWOULDBLOCK"):
    _CONNECT_IN_PROGRESS.add(errno.WSAEWOULDBLOCK)


# ---------------------------------------------------------------------------
# 0. MQTT discovery — что Pi реально опубликовал в retained camera/endpoint
# ---------------------------------------------------------------------------
def test_mqtt_discovery(broker: str, mqtt_port: int, robot_id: str):
    _hr(f"0. MQTT discovery  <-  {broker}:{mqtt_port}  (samurai/{robot_id}/camera/endpoint)")
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("  SKIP: paho-mqtt не установлен (host/port возьмём из --host / env)")
        return None

    got: dict = {}
    done = threading.Event()

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(f"samurai/{robot_id}/camera/endpoint", qos=1)
        else:
            print(f"  FAIL: MQTT connect rc={rc}")
            done.set()

    def on_message(client, userdata, msg):
        if msg.payload:
            try:
                got.update(json.loads(msg.payload))
            except Exception as exc:  # noqa: BLE001
                print(f"  FAIL: битый JSON в endpoint: {exc}")
        else:
            got["_empty"] = True
        done.set()

    try:
        client = mqtt.Client()
    except Exception:  # paho 2.x — другая сигнатура
        from paho.mqtt.client import CallbackAPIVersion  # type: ignore
        client = mqtt.Client(CallbackAPIVersion.VERSION1)
    client.on_connect = on_connect
    client.on_message = on_message
    user = os.environ.get("SAMURAI_MQTT_USER") or os.environ.get("MQTT_USER")
    pwd = os.environ.get("SAMURAI_MQTT_PASS") or os.environ.get("MQTT_PASS")
    if user:
        client.username_pw_set(user, pwd)

    t0 = time.time()
    try:
        client.connect(broker, mqtt_port, keepalive=10)
    except Exception as exc:  # noqa: BLE001
        print(f"  FAIL: MQTT connect: {type(exc).__name__}: {exc}")
        return None
    client.loop_start()
    done.wait(timeout=8.0)
    client.loop_stop()
    client.disconnect()

    if not got:
        print(f"  FAIL: нет retained camera/endpoint за {time.time() - t0:.1f}s "
              "(camera_node на Pi не запущен или не опубликовал)")
        return None
    if got.get("_empty"):
        print("  FAIL: retained endpoint пустой — Pi объявил камеру offline")
        return None
    print(f"  OK: endpoint = {json.dumps(got, ensure_ascii=False)}")
    return got


# ---------------------------------------------------------------------------
# 1. Блокирующий connect — так работает paho-mqtt и detector frame_sources
# ---------------------------------------------------------------------------
def test_blocking_connect(host: str, port: int, iterations: int, timeout: float) -> float:
    _hr(f"1. Блокирующий TCP connect  ->  {host}:{port}   ({iterations} попыток)")
    ok = 0
    for i in range(iterations):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        t0 = time.time()
        try:
            sock.connect((host, port))
            dt = time.time() - t0
            ok += 1
            print(f"  #{i + 1}: OK   {dt * 1000:.0f} ms")
        except Exception as exc:  # noqa: BLE001
            dt = time.time() - t0
            print(f"  #{i + 1}: FAIL {dt:.1f}s  {type(exc).__name__}: {exc}")
        finally:
            sock.close()
        time.sleep(0.2)
    rate = ok / iterations
    print(f"  -> успешно {ok}/{iterations} ({rate * 100:.0f}%)")
    return rate


# ---------------------------------------------------------------------------
# 2. Неблокирующий connect + select() — так работает asyncio под капотом
# ---------------------------------------------------------------------------
def test_nonblocking_connect(host: str, port: int, iterations: int, timeout: float) -> float:
    _hr(f"2. Неблокирующий connect + select()  ->  {host}:{port}   ({iterations} попыток)")
    ok = 0
    for i in range(iterations):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setblocking(False)
        t0 = time.time()
        rc = sock.connect_ex((host, port))
        if rc not in _CONNECT_IN_PROGRESS:
            print(f"  #{i + 1}: FAIL connect_ex -> {errno.errorcode.get(rc, rc)}")
            sock.close()
            time.sleep(0.2)
            continue
        _, writable, exceptional = select.select([], [sock], [sock], timeout)
        dt = time.time() - t0
        if writable:
            so_err = sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
            if so_err == 0:
                ok += 1
                print(f"  #{i + 1}: OK   writable за {dt * 1000:.0f} ms")
            else:
                print(f"  #{i + 1}: FAIL SO_ERROR={errno.errorcode.get(so_err, so_err)}")
        elif exceptional:
            print(f"  #{i + 1}: FAIL сокет в exceptfds за {dt:.1f}s")
        else:
            print(f"  #{i + 1}: FAIL select НЕ writable за {dt:.1f}s "
                  "(уведомление о connect потеряно)")
        sock.close()
        time.sleep(0.2)
    rate = ok / iterations
    print(f"  -> успешно {ok}/{iterations} ({rate * 100:.0f}%)")
    return rate


# ---------------------------------------------------------------------------
# 3. asyncio.open_connection — ровно то, что делает dashboard /ws/h264
# ---------------------------------------------------------------------------
def test_asyncio_connect(host: str, port: int, timeout: float) -> bool:
    _hr(f"3. asyncio.open_connection  ->  {host}:{port}   (как в /ws/h264)")

    async def _go() -> bool:
        t0 = time.time()
        try:
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(host, port), timeout=timeout)
            print(f"  OK: подключился за {(time.time() - t0) * 1000:.0f} ms")
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:  # noqa: BLE001
                pass
            return True
        except asyncio.TimeoutError:
            print(f"  FAIL: asyncio.TimeoutError за {time.time() - t0:.1f}s")
            print("        ^ именно это браузер показывает как 'TCP connect failed:'")
            return False
        except Exception as exc:  # noqa: BLE001
            print(f"  FAIL: {type(exc).__name__}: {exc}")
            return False

    return asyncio.run(_go())


# ---------------------------------------------------------------------------
# 4. Фикс-вариант A: блокирующий connect в потоке -> asyncio читает сокет
# ---------------------------------------------------------------------------
def test_blocking_connect_async_read(host: str, port: int, timeout: float):
    _hr("4. Блокирующий connect (executor) -> asyncio читает сокет  (фикс-вариант A)")

    async def _go():
        loop = asyncio.get_running_loop()
        try:
            sock = await loop.run_in_executor(
                None, lambda: socket.create_connection((host, port), timeout=timeout))
        except OSError as exc:
            print(f"  SKIP: блокирующий connect не удался: {type(exc).__name__}: {exc}")
            return None
        print("  блокирующий connect (в executor) OK")
        try:
            reader, writer = await asyncio.open_connection(sock=sock)
        except Exception as exc:  # noqa: BLE001
            print(f"  FAIL: asyncio.open_connection(sock=...): {type(exc).__name__}: {exc}")
            sock.close()
            return False
        try:
            data = await asyncio.wait_for(reader.read(64 * 1024), timeout=timeout)
            if data:
                nal = data.count(b"\x00\x00\x01")
                print(f"  OK: asyncio прочитал {len(data)} байт, ~{nal} NAL start-codes")
                return True
            print("  FAIL: asyncio прочитал 0 байт (EOF сразу)")
            return False
        except asyncio.TimeoutError:
            print(f"  FAIL: asyncio.read таймаут {timeout:.0f}s — неблокирующее ЧТЕНИЕ сломано")
            return False
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:  # noqa: BLE001
                pass

    return asyncio.run(_go())


# ---------------------------------------------------------------------------
# 5. Фикс-вариант B: всё на блокирующем сокете в потоке (как frame_sources.py)
# ---------------------------------------------------------------------------
def test_fully_blocking(host: str, port: int, timeout: float):
    _hr("5. Полностью блокирующий connect + recv  (фикс-вариант B)")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect((host, port))
    except Exception as exc:  # noqa: BLE001
        print(f"  SKIP: блокирующий connect не удался: {type(exc).__name__}: {exc}")
        sock.close()
        return None
    print("  блокирующий connect OK")
    sock.settimeout(timeout)
    try:
        data = sock.recv(64 * 1024)
    except Exception as exc:  # noqa: BLE001
        print(f"  FAIL: блокирующий recv: {type(exc).__name__}: {exc}")
        sock.close()
        return False
    finally:
        sock.close()
    if not data:
        print("  FAIL: recv вернул 0 байт (EOF) — TCP-сервер есть, но потока нет "
              "(encoder/libcamera на Pi мог упасть)")
        return False
    nal = data.count(b"\x00\x00\x01")
    print(f"  OK: блокирующий recv {len(data)} байт, ~{nal} NAL start-codes -> поток ЖИВОЙ")
    return True


# ---------------------------------------------------------------------------
# Диагноз
# ---------------------------------------------------------------------------
def diagnose(r: dict) -> int:
    _hr("ДИАГНОЗ")
    blk = r.get("blocking_rate")
    nblk = r.get("nonblocking_rate")
    aio = r.get("asyncio_ok")
    fix_a = r.get("fix_a_ok")
    fix_b = r.get("fix_b_ok")

    if blk is None:
        print("  Нечего тестировать — не задан host (нет --host и MQTT discovery пуст).")
        return 2

    if blk == 0:
        print("  X  Даже блокирующий connect не проходит ни разу.")
        print("     Pi camera_node не слушает :8554, либо сеть до Pi полностью лежит.")
        print("     Проверь: запущен ли './samurai.sh robot'; ufw на Pi (allow 8554/tcp);")
        print("     та ли WiFi-сеть (hotspot Pi); ./samurai.sh status.")
        return 1

    if aio:
        print("  OK  asyncio.open_connection здесь работает — проблема НЕ воспроизводится.")
        print("      Если браузер всё равно без потока — запусти скрипт ВНУТРИ")
        print("      dashboard-контейнера (docker exec ...), там и проявится.")
        return 0

    print(f"  X  asyncio.open_connection НЕ работает  "
          f"(blocking {blk * 100:.0f}%, nonblocking {(nblk or 0) * 100:.0f}%).")
    print()

    if (nblk or 0) == 0:
        print("  КОРЕНЬ: неблокирующий connect никогда не получает уведомление о")
        print("  завершении — select()/epoll не видит сокет writable, хотя блокирующий")
        print("  connect к тому же адресу проходит. asyncio целиком построен на")
        print("  неблокирующих сокетах, поэтому /ws/h264 прокси таймаутит на 5 секунд.")
        print()
        print("  Типичная причина: Docker Desktop (Windows/WSL2) + VPN/прокси-адаптеры")
        print("  на хосте (Clash/FlClash, Tailscale, ZeroTier, Radmin, Outline).")
        print("  Блокирующий I/O ядро тащит само, асинхронные уведомления теряются")
        print("  в NAT-прослойке между контейнером и физическим адаптером хоста.")
        print()

    if fix_a:
        print("  ФИКС (минимальный, вариант A): неблокирующее ЧТЕНИЕ работает.")
        print("  Достаточно делать только connect блокирующим, в executor-потоке,")
        print("  а готовый сокет отдавать в asyncio.open_connection(sock=...).")
    elif fix_b:
        print("  ФИКС (полный, вариант B): неблокирующее чтение ТОЖЕ сломано.")
        print("  Весь TCP->WS прокси должен работать на блокирующем сокете в потоке —")
        print("  ровно так, как уже сделано в compute_node/detectors/frame_sources.py.")
    else:
        print("  Ни один фикс-вариант не подтвердился: либо connect флакает, либо")
        print("  Pi не отдаёт данные. Сначала убедись, что поток вообще идёт")
        print("  (тест 5 должен видеть NAL start-codes).")

    print()
    print("  Файл для правки: compute_node/dashboard/routers/camera.py  (ws_h264).")
    print("  Обходной путь без кода: запускать compute-стек на Linux (--net=host),")
    print("  либо отключить VPN/прокси-туннели на хосте на время работы с роботом.")
    return 1


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=None,
                    help="IP Pi camera (по умолчанию — из MQTT discovery / env MQTT_BROKER)")
    ap.add_argument("--port", type=int, default=None,
                    help="TCP-порт H.264 (по умолчанию из discovery, иначе 8554)")
    ap.add_argument("--broker", default=None,
                    help="MQTT broker (по умолчанию env MQTT_BROKER, иначе --host)")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--robot-id", default=None,
                    help="robot_id (по умолчанию env ROBOT_ID, иначе robot1)")
    ap.add_argument("--iterations", type=int, default=5,
                    help="сколько раз повторить connect-тесты (ловим флакость)")
    ap.add_argument("--timeout", type=float, default=5.0,
                    help="таймаут одного connect — как в dashboard (5с)")
    args = ap.parse_args()

    robot_id = args.robot_id or os.environ.get("ROBOT_ID", "robot1")
    broker = args.broker or os.environ.get("MQTT_BROKER") or args.host

    print("=" * 72)
    print(" ДИАГНОСТИКА H.264 ВИДЕОПОТОКА   (Pi camera_node -> dashboard /ws/h264)")
    print("=" * 72)
    where = "контейнер" if os.path.exists("/.dockerenv") else "хост"
    print(f" Запущено в: {where}     Python: {sys.version.split()[0]}")
    try:
        import uvloop  # noqa: F401
        print(" uvloop: установлен (uvicorn использует его в проде)")
    except ImportError:
        print(" uvloop: не установлен")

    results: dict = {}

    endpoint = test_mqtt_discovery(broker, args.mqtt_port, robot_id) if broker else None

    host = args.host
    port = args.port
    if endpoint:
        host = host or endpoint.get("host")
        port = port or int(endpoint.get("port", 8554))
    host = host or os.environ.get("MQTT_BROKER")
    port = port or 8554

    if not host:
        print("\n  Не задан --host и MQTT discovery ничего не дал — нечего тестировать.")
        print("  Пример: python3 tools/diag_camera_stream.py --host 192.168.4.1")
        return 2

    print(f"\n Цель: H.264 TCP-сервер {host}:{port}")

    results["blocking_rate"] = test_blocking_connect(host, port, args.iterations, args.timeout)
    results["nonblocking_rate"] = test_nonblocking_connect(host, port, args.iterations, args.timeout)
    results["asyncio_ok"] = test_asyncio_connect(host, port, args.timeout)
    results["fix_a_ok"] = test_blocking_connect_async_read(host, port, args.timeout)
    if not results["fix_a_ok"]:
        results["fix_b_ok"] = test_fully_blocking(host, port, args.timeout)
    else:
        results["fix_b_ok"] = None

    return diagnose(results)


if __name__ == "__main__":
    sys.exit(main())
