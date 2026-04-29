"""
LlmVoiceNode — MQTT-нода для парсинга голоса через LLM (#2, 2026-04).

Подписки:
    samurai/{robot_id}/voice_command   — raw текст от Pi voice_node (Vosk)

Публикации:
    samurai/{robot_id}/voice/intent    — JSON VoiceIntent
    samurai/{robot_id}/voice/llm/status — health: backend, last_latency_ms

Backend выбирается через CLI / ENV:
    --backend ollama --model qwen2.5:7b
    --backend mock                       # для разработки без Ollama

Inference запускается в отдельном thread'е (через ThreadPoolExecutor),
чтобы не блокировать paho-mqtt loop. На запросы поступающие во время
обработки — публикуется intent с action=idle, confidence=0 (LLM busy).
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

import paho.mqtt.client as mqtt_client

from .backends import LlmBackend, MockBackend, OllamaBackend
from .schema import VoiceIntent

log = logging.getLogger(__name__)

_MAX_TEXT_LEN = 200


def _resolve_mqtt_creds() -> tuple[Optional[str], Optional[str]]:
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
        from config_loader import get_mqtt_credentials
        return get_mqtt_credentials()
    except ImportError:
        u = os.environ.get('SAMURAI_MQTT_USER', '').strip()
        p = os.environ.get('SAMURAI_MQTT_PASS', '')
        return (u, p) if u and p else (None, None)


class LlmVoiceNode:
    """Слушает voice_command, парсит через LLM, публикует voice/intent."""

    def __init__(
        self,
        broker: str,
        port: int = 1883,
        robot_id: str = 'robot1',
        backend: Optional[LlmBackend] = None,
        client_id: str = 'samurai_llm_voice',
    ):
        self._broker = broker
        self._port = port
        self._robot_id = robot_id
        self._prefix = f'samurai/{robot_id}'
        self._backend: LlmBackend = backend or MockBackend()

        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='llm-')
        self._busy = threading.Lock()

        self._client = mqtt_client.Client(client_id=client_id)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.reconnect_delay_set(min_delay=0.5, max_delay=5)
        user, pwd = _resolve_mqtt_creds()
        if user:
            self._client.username_pw_set(user, pwd or '')
        self._auth_str = f' user={user}' if user else ' anonymous'

    # ── Lifecycle ──────────────────────────────────────────────────
    def start(self) -> None:
        self._client.connect_async(self._broker, self._port, keepalive=15)
        self._client.loop_start()
        log.info(
            'LlmVoice connecting → %s:%d%s (backend=%s)',
            self._broker, self._port, self._auth_str, self._backend.name,
        )

    def stop(self) -> None:
        try:
            self._executor.shutdown(wait=False, cancel_futures=True)
            self._client.loop_stop()
            self._client.disconnect()
        except Exception:
            pass

    def spin(self) -> None:
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass

    # ── MQTT callbacks ─────────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            log.error('MQTT connect failed rc=%s', rc)
            return
        client.subscribe(f'{self._prefix}/voice_command', qos=1)
        log.info('Subscribed to voice_command')
        # Объявим себя живым
        self._publish_status(state='ready')

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8', errors='ignore')
        except Exception:
            return
        # voice_command может быть raw text или JSON {text: "..."}
        text = payload.strip()
        if text.startswith('{'):
            try:
                data = json.loads(text)
                text = str(data.get('text', '')).strip()
            except json.JSONDecodeError:
                pass
        if not text or len(text) > _MAX_TEXT_LEN:
            log.warning('voice_command rejected (length=%d)', len(text))
            return

        # Запускаем парсинг в worker thread — не блокируем paho loop
        if not self._busy.acquire(blocking=False):
            # LLM ещё работает над предыдущим запросом → пропускаем
            log.warning('LLM busy, dropping: "%s"', text[:60])
            self._publish_intent(VoiceIntent(
                action='idle', raw_text=text, confidence=0.0,
                source='llm',
            ))
            return
        self._executor.submit(self._process, text)

    # ── Worker ─────────────────────────────────────────────────────
    def _process(self, text: str) -> None:
        try:
            t0 = time.time()
            intent = self._backend.parse_intent(text)
            dt_ms = (time.time() - t0) * 1000.0
            if intent is None:
                log.warning('Backend returned no intent for: "%s"', text[:60])
                # Idle с low confidence — fsm применит regex-fallback
                intent = VoiceIntent(
                    action='idle', raw_text=text, confidence=0.0,
                    source='llm',
                )
            log.info(
                'intent: "%s" → %s (colour=%s dir=%s) [%.0fms, %s]',
                text, intent.action, intent.colour, intent.direction,
                dt_ms, intent.source,
            )
            self._publish_intent(intent)
            self._publish_status(state='ready', last_latency_ms=dt_ms)
        finally:
            self._busy.release()

    # ── Publishing ─────────────────────────────────────────────────
    def _publish_intent(self, intent: VoiceIntent) -> None:
        self._client.publish(
            f'{self._prefix}/voice/intent',
            intent.model_dump_json(),
            qos=1,
        )

    def _publish_status(
        self, state: str = 'ready',
        last_latency_ms: Optional[float] = None,
    ) -> None:
        payload = {
            'state': state,
            'backend': self._backend.name,
            'ts': time.time(),
        }
        if last_latency_ms is not None:
            payload['last_latency_ms'] = round(last_latency_ms, 1)
        self._client.publish(
            f'{self._prefix}/voice/llm/status',
            json.dumps(payload),
            qos=0,
            retain=True,
        )


# ── Entry point ────────────────────────────────────────────────────────
def _make_backend(name: str, model: str, host: str) -> LlmBackend:
    if name == 'ollama':
        return OllamaBackend(model=model, host=host)
    if name == 'mock':
        return MockBackend()
    raise SystemExit(f'Unknown backend: {name} (use ollama|mock)')


def main() -> int:
    parser = argparse.ArgumentParser(description='LLM voice intent parser.')
    parser.add_argument('--broker', default=os.environ.get('MQTT_BROKER', '127.0.0.1'))
    parser.add_argument('--port', type=int, default=int(os.environ.get('MQTT_PORT', '1883')))
    parser.add_argument('--robot-id', default=os.environ.get('ROBOT_ID', 'robot1'))
    parser.add_argument('--backend', choices=['ollama', 'mock'], default='ollama')
    parser.add_argument('--model', default='qwen2.5:7b',
                        help='Ollama model tag (default: qwen2.5:7b)')
    parser.add_argument('--ollama-host', default='http://localhost:11434')
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )

    backend = _make_backend(args.backend, args.model, args.ollama_host)
    node = LlmVoiceNode(
        broker=args.broker, port=args.port,
        robot_id=args.robot_id, backend=backend,
    )
    node.start()
    try:
        node.spin()
    finally:
        node.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
