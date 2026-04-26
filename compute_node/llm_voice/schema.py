"""
VoiceIntent — структурированное представление голосовой команды (#2).

Формат, который LLM возвращает после парсинга голосового текста.
Совместим с действиями fsm_node:

  action:
    grab               — найти и взять мяч (опц. цвет)
    stop               — остановить всю активность
    home               — вернуться на (0, 0)
    patrol             — режим патрулирования
    follow             — режим follow-me
    record_path        — начать запись пути
    replay_path        — повторить путь
    reset_position     — сбросить одометрию в (0, 0, 0)
    call_other_robot   — позвать второго робота (samcan)
    move               — ручное движение (требует direction)
    transition         — admin force-transition в указанное состояние
    idle               — нет команды / непонятно (низкий confidence)

  colour:    red|blue|green|yellow|orange|white|black или null
  direction: forward|back|left|right или null
  distance_m / angle_deg: для будущего (move 50 см вперёд) — пока not used
  raw_text:  оригинал голосовой команды
  confidence: 0..1
  source:    'llm' | 'regex_fallback' | 'mock'
"""
from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field

ActionName = Literal[
    'grab', 'stop', 'home', 'patrol', 'follow',
    'record_path', 'replay_path', 'reset_position',
    'call_other_robot', 'move', 'transition', 'idle',
]
ColourName = Literal[
    'red', 'blue', 'green', 'yellow', 'orange',
    'white', 'black',
]
DirectionName = Literal['forward', 'back', 'left', 'right']


class VoiceIntent(BaseModel):
    """Парсенный intent голосовой команды.

    Сериализуется через .model_dump_json() для публикации в MQTT
    samurai/{robot_id}/voice/intent.
    """
    action: ActionName = 'idle'
    colour: Optional[ColourName] = None
    direction: Optional[DirectionName] = None
    distance_m: Optional[float] = Field(default=None, ge=0.0, le=10.0)
    angle_deg: Optional[float] = Field(default=None, ge=-360.0, le=360.0)
    target_state: Optional[str] = Field(
        default=None,
        description='Имя FSM-состояния для action=transition '
                    '(IDLE/SEARCHING/...)',
    )
    raw_text: str = ''
    confidence: float = Field(default=0.0, ge=0.0, le=1.0)
    source: Literal['llm', 'regex_fallback', 'mock'] = 'llm'
