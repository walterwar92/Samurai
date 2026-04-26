"""
Промпт для LLM + парсер JSON-ответа в VoiceIntent.

Изолированы от backends чтобы можно было unit-тестировать без LLM:
  - _build_prompt(text) — формирует system+user prompt для Qwen-like моделей
  - parse_response(text, raw_text) — извлекает JSON из ответа модели
                                       (даже если LLM завернула в ```json...```)

Формат интента описан в schema.VoiceIntent.
"""
from __future__ import annotations

import json
import logging
import re
from typing import Optional

from .schema import VoiceIntent

log = logging.getLogger(__name__)


# Системный промпт. Лаконичный — Qwen 7B на CPU не любит длинные контексты.
SYSTEM_PROMPT = """\
Ты — парсер голосовых команд робота-охотника на мячи. Команды на русском.

Доступные actions (выбери один):
  grab               — найти мяч и взять его (нужен colour)
  stop               — немедленно остановиться
  home               — вернуться на (0, 0)
  patrol             — режим патруля
  follow             — режим follow-me
  record_path        — начать запись пути
  replay_path        — воспроизвести записанный путь
  reset_position     — сбросить одометрию
  call_other_robot   — вызвать второго робота
  move               — ручное движение (нужен direction)
  transition         — принудительная смена FSM-состояния (нужен target_state)
  idle               — команда непонятна

colour:    red, blue, green, yellow, orange, white, black или null
direction: forward, back, left, right или null

Верни СТРОГО валидный JSON и ничего больше:
{"action": "...", "colour": null, "direction": null,
 "distance_m": null, "angle_deg": null, "target_state": null,
 "confidence": 0.0..1.0}
"""

# Few-shot примеры — помогают слабым моделям (Qwen 1.5B/3B).
FEW_SHOTS = [
    ('найди красный мяч', {
        'action': 'grab', 'colour': 'red', 'confidence': 0.95,
    }),
    ('стоп', {'action': 'stop', 'confidence': 0.99}),
    ('домой', {'action': 'home', 'confidence': 0.98}),
    ('езжай вперёд', {
        'action': 'move', 'direction': 'forward', 'confidence': 0.9,
    }),
    ('начни патруль', {'action': 'patrol', 'confidence': 0.95}),
    ('что-то непонятное бубубу', {
        'action': 'idle', 'confidence': 0.1,
    }),
]


def build_prompt(text: str) -> tuple[str, str]:
    """Возвращает (system, user) промпт для chat-completion API."""
    user_lines = ['Примеры:']
    for in_text, intent in FEW_SHOTS:
        user_lines.append(f'  "{in_text}" → {json.dumps(intent, ensure_ascii=False)}')
    user_lines.append('')
    user_lines.append(f'Команда: "{text}"')
    user_lines.append('JSON:')
    return SYSTEM_PROMPT, '\n'.join(user_lines)


# Регулярка для извлечения JSON из ответа (LLM иногда оборачивает в
# ```json ... ``` или добавляет «Вот ответ:»).
_JSON_BLOCK = re.compile(r'\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}', re.DOTALL)


def parse_response(
    response_text: str,
    raw_text: str,
    source: str = 'llm',
) -> Optional[VoiceIntent]:
    """Извлечь JSON из ответа LLM и сконструировать VoiceIntent.

    Возвращает None если JSON не нашёлся / валидация Pydantic упала.
    Чем «грязнее» ответ, тем агрессивнее regex-fallback.
    """
    text = response_text.strip()
    # Снять markdown ```json...``` обёртку
    text = re.sub(r'^```(?:json)?\s*', '', text, flags=re.IGNORECASE)
    text = re.sub(r'\s*```$', '', text)

    # Попытка №1: целый текст это JSON
    candidates: list[str] = []
    if text.startswith('{') and text.endswith('}'):
        candidates.append(text)

    # Попытка №2: первый JSON-блок в тексте
    match = _JSON_BLOCK.search(text)
    if match:
        candidates.append(match.group(0))

    for candidate in candidates:
        try:
            data = json.loads(candidate)
        except json.JSONDecodeError:
            continue
        try:
            data['raw_text'] = raw_text
            data['source'] = source
            # Pydantic: лишние поля игнорируются (model_config? — здесь
            # дефолтный, но нам ок: VoiceIntent поля уже все известны)
            return VoiceIntent(**{k: v for k, v in data.items()
                                  if k in VoiceIntent.model_fields})
        except Exception as exc:  # ValidationError / TypeError
            log.warning('Intent validation failed: %s (data=%s)', exc, data)
            continue

    return None
