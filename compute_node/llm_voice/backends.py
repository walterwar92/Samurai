"""
Backend protocol + реализации для LLM-инференса.

Backends:
  OllamaBackend — HTTP клиент к localhost:11434/api/chat (или REMOTE_HOST).
                  Совместим с Ollama (`ollama serve` + `ollama pull qwen2.5:7b`).
  MockBackend   — Возвращает заранее заданный intent. Для тестов и dev-mode
                  без LLM.

Все backends синхронные — node.py гарантирует что вызовы делаются в
отдельном потоке от MQTT loop'а.
"""
from __future__ import annotations

import logging
from typing import Optional, Protocol

from .parser import build_prompt, parse_response
from .schema import VoiceIntent

log = logging.getLogger(__name__)


class LlmBackend(Protocol):
    """Протокол для любого LLM-бэкенда."""

    name: str

    def parse_intent(self, text: str) -> Optional[VoiceIntent]:
        """Парсит голосовую команду в structured intent.

        Returns None если backend недоступен / парсинг провалился —
        FSM тогда применит regex-fallback.
        """
        ...


# ── OllamaBackend ──────────────────────────────────────────────────────
class OllamaBackend:
    """LLM через Ollama HTTP API.

    Setup:
        # На ноуте (один раз):
        curl -fsSL https://ollama.com/install.sh | sh
        ollama pull qwen2.5:7b
        ollama serve   # listening на :11434

        samurai voice-llm   # nodes уже подключатся
    """

    name = 'ollama'

    def __init__(
        self,
        model: str = 'qwen2.5:7b',
        host: str = 'http://localhost:11434',
        timeout_s: float = 10.0,
    ):
        self._model = model
        self._host = host.rstrip('/')
        self._timeout = timeout_s

    def parse_intent(self, text: str) -> Optional[VoiceIntent]:
        try:
            import httpx  # type: ignore[import-untyped]
        except ImportError:
            log.error('httpx not installed — cannot use OllamaBackend')
            return None

        system, user = build_prompt(text)
        payload = {
            'model': self._model,
            'messages': [
                {'role': 'system', 'content': system},
                {'role': 'user', 'content': user},
            ],
            'stream': False,
            # Низкая температура — мы хотим детерминированный JSON
            'options': {'temperature': 0.0, 'num_ctx': 2048},
            # Качественный JSON-форматер у Ollama (если поддерживается)
            'format': 'json',
        }
        try:
            with httpx.Client(timeout=self._timeout) as client:
                r = client.post(f'{self._host}/api/chat', json=payload)
            r.raise_for_status()
        except Exception as exc:
            log.warning('Ollama request failed: %s', exc)
            return None

        try:
            data = r.json()
            content = data.get('message', {}).get('content', '')
        except Exception as exc:
            log.warning('Ollama response not JSON: %s', exc)
            return None

        if not content:
            return None
        intent = parse_response(content, raw_text=text, source='llm')
        if intent is None:
            log.warning('LLM response did not parse to VoiceIntent: %s',
                        content[:200])
        return intent


# ── MockBackend ────────────────────────────────────────────────────────
class MockBackend:
    """Backend для unit-тестов и dev — возвращает заранее заданный intent.

    Если configure'нют через `set_response` — отдаёт его. Иначе пытается
    распознать через regex-набор похожий на fsm_node._P_*.
    """

    name = 'mock'

    def __init__(self) -> None:
        self._stub: Optional[VoiceIntent] = None

    def set_response(self, intent: Optional[VoiceIntent]) -> None:
        self._stub = intent

    def parse_intent(self, text: str) -> Optional[VoiceIntent]:
        if self._stub is not None:
            # Подменяем raw_text на актуальный
            data = self._stub.model_dump()
            data['raw_text'] = text
            data['source'] = 'mock'
            return VoiceIntent(**data)

        # Очень простой regex для базовой dev-функциональности.
        t = text.lower().strip()
        if not t:
            return VoiceIntent(action='idle', raw_text=text, source='mock')
        if 'стоп' in t or 'остановись' in t:
            return VoiceIntent(action='stop', raw_text=text, source='mock',
                               confidence=0.9)
        # Directions проверяем РАНЬШЕ home, иначе 'повернись' содержит
        # substring 'вернись' и матчит home по ошибке.
        if 'налево' in t or 'влево' in t:
            return VoiceIntent(action='move', direction='left',
                               raw_text=text, source='mock', confidence=0.85)
        if 'направо' in t or 'вправо' in t:
            return VoiceIntent(action='move', direction='right',
                               raw_text=text, source='mock', confidence=0.85)
        if 'вперёд' in t or 'вперед' in t:
            return VoiceIntent(action='move', direction='forward',
                               raw_text=text, source='mock', confidence=0.85)
        if 'назад' in t:
            return VoiceIntent(action='move', direction='back',
                               raw_text=text, source='mock', confidence=0.85)
        if 'домой' in t or 'вернись' in t:
            return VoiceIntent(action='home', raw_text=text, source='mock',
                               confidence=0.9)
        if 'найди' in t or 'возьми' in t or 'получи' in t:
            colour = None
            for rus, eng in (
                ('красн', 'red'), ('син', 'blue'), ('зелен', 'green'),
                ('жёлт', 'yellow'), ('желт', 'yellow'),
                ('оранж', 'orange'), ('бел', 'white'), ('чёрн', 'black'),
                ('черн', 'black'),
            ):
                if rus in t:
                    colour = eng
                    break
            return VoiceIntent(action='grab', colour=colour, raw_text=text,
                               source='mock', confidence=0.85)
        if 'патрул' in t:
            return VoiceIntent(action='patrol', raw_text=text, source='mock',
                               confidence=0.9)
        if 'следуй' in t:
            return VoiceIntent(action='follow', raw_text=text, source='mock',
                               confidence=0.9)
        return VoiceIntent(action='idle', raw_text=text, source='mock',
                           confidence=0.0)
