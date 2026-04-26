"""
compute_node.llm_voice — LLM-парсер голосовых команд (#2, 2026-04).

Запуск:
    python -m compute_node.llm_voice
    samurai voice-llm

Архитектура:
    schema.py    — VoiceIntent (action + colour + direction + ...)
    backends.py  — Backend Protocol + OllamaBackend (HTTP) + MockBackend
    parser.py    — _build_prompt() + _parse_response() (LLM-agnostic helpers)
    node.py      — MQTT-нода: voice_command → LLM → voice/intent
    __main__.py  — entry point

Поток данных:
    Pi voice_node (Vosk) ───MQTT voice_command──→ Compute llm_voice
                                                         │
                                            Ollama qwen2.5:7b
                                                         │
    Pi fsm_node ←───MQTT voice/intent (JSON)─────────────┘
        │
        ├─ Если intent fresh → выполняет structured action (TODO)
        └─ Иначе fallback на regex parsing (как раньше)

Решение #2: LLM на ноутбуке (Qwen 7B через Ollama), regex-fallback в
fsm_node остаётся работать когда LLM offline.
"""
__version__ = '1.0.0'
