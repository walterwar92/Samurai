"""
Юнит-тесты для compute_node/llm_voice/* (#2, 2026-04).

Покрываем:
  - VoiceIntent (Pydantic): валидация, дефолты, сериализация
  - parser.parse_response: чистый JSON, ```json``` обёртка, мусор по краям,
    invalid JSON, validation error
  - MockBackend: regex-recognition, set_response stub
  - OllamaBackend: с замоканным httpx — успех, http error, JSON-ошибка

Запуск:
    pytest tests/test_llm_voice.py -v
"""
from __future__ import annotations

import json
import os
import sys
from unittest.mock import MagicMock, patch

import pytest

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, REPO_ROOT)

from compute_node.llm_voice.backends import MockBackend, OllamaBackend
from compute_node.llm_voice.parser import build_prompt, parse_response
from compute_node.llm_voice.schema import VoiceIntent


# ── VoiceIntent schema ────────────────────────────────────────────────
def test_intent_defaults():
    i = VoiceIntent()
    assert i.action == 'idle'
    assert i.colour is None
    assert i.confidence == 0.0
    assert i.source == 'llm'


def test_intent_full():
    i = VoiceIntent(
        action='grab', colour='red', raw_text='найди красный',
        confidence=0.95, source='llm',
    )
    assert i.action == 'grab'
    assert i.colour == 'red'
    assert i.confidence == 0.95


def test_intent_invalid_action_rejected():
    with pytest.raises(Exception):  # pydantic ValidationError
        VoiceIntent(action='launch_missiles')  # type: ignore[arg-type]


def test_intent_invalid_colour_rejected():
    with pytest.raises(Exception):
        VoiceIntent(action='grab', colour='magenta')  # type: ignore[arg-type]


def test_intent_distance_clamped():
    with pytest.raises(Exception):
        VoiceIntent(action='move', distance_m=100.0)  # >10 запрещено
    with pytest.raises(Exception):
        VoiceIntent(action='move', distance_m=-1.0)


# ── parser.parse_response ─────────────────────────────────────────────
def test_parse_pure_json():
    raw = '{"action": "stop", "confidence": 0.9}'
    i = parse_response(raw, raw_text='стоп')
    assert i is not None
    assert i.action == 'stop'
    assert i.confidence == 0.9
    assert i.raw_text == 'стоп'


def test_parse_json_in_markdown():
    raw = '```json\n{"action": "grab", "colour": "blue"}\n```'
    i = parse_response(raw, raw_text='blue ball')
    assert i is not None
    assert i.action == 'grab'
    assert i.colour == 'blue'


def test_parse_with_garbage_around():
    raw = 'Sure! Here is your JSON:\n{"action": "home", "confidence": 0.95}\nhope this helps!'
    i = parse_response(raw, raw_text='домой')
    assert i is not None
    assert i.action == 'home'


def test_parse_invalid_json_returns_none():
    assert parse_response('not json at all', 'x') is None


def test_parse_json_failing_validation_returns_none():
    # action не из списка → None
    raw = '{"action": "nuke_world"}'
    assert parse_response(raw, 'go') is None


def test_parse_extra_fields_ignored():
    raw = '{"action": "stop", "garbage": 42, "another": "x"}'
    i = parse_response(raw, raw_text='стоп')
    assert i is not None
    assert i.action == 'stop'


def test_parse_source_propagated():
    raw = '{"action": "stop", "confidence": 0.9}'
    i = parse_response(raw, raw_text='', source='regex_fallback')
    assert i is not None
    assert i.source == 'regex_fallback'


# ── build_prompt ──────────────────────────────────────────────────────
def test_build_prompt_includes_text():
    system, user = build_prompt('найди мяч')
    assert 'парсер' in system
    assert 'найди мяч' in user
    assert 'JSON' in user.upper()


def test_build_prompt_includes_few_shots():
    _, user = build_prompt('тест')
    assert 'найди красный мяч' in user
    assert 'grab' in user


# ── MockBackend ───────────────────────────────────────────────────────
def test_mock_recognizes_grab_red():
    m = MockBackend()
    i = m.parse_intent('найди красный мяч')
    assert i is not None
    assert i.action == 'grab'
    assert i.colour == 'red'
    assert i.source == 'mock'


def test_mock_recognizes_directions():
    m = MockBackend()
    cases = [
        ('езжай вперёд', 'forward'),
        ('сдай назад', 'back'),
        ('поверни налево', 'left'),
        ('повернись вправо', 'right'),
    ]
    for text, expected_dir in cases:
        i = m.parse_intent(text)
        assert i is not None
        assert i.action == 'move'
        assert i.direction == expected_dir, f'wrong dir for "{text}"'


def test_mock_idle_for_unknown():
    m = MockBackend()
    i = m.parse_intent('абракадабра')
    assert i is not None
    assert i.action == 'idle'
    assert i.confidence == 0.0


def test_mock_set_response_overrides():
    m = MockBackend()
    stub = VoiceIntent(action='patrol', confidence=0.7)
    m.set_response(stub)
    i = m.parse_intent('что угодно')
    assert i is not None
    assert i.action == 'patrol'
    assert i.confidence == 0.7
    assert i.raw_text == 'что угодно'  # подменяется на актуальный
    assert i.source == 'mock'


# ── OllamaBackend (httpx mock) ────────────────────────────────────────
def _ollama_response(content: dict) -> MagicMock:
    """Создаёт MagicMock похожий на httpx.Response.json()."""
    resp = MagicMock()
    resp.raise_for_status = MagicMock()
    resp.json = MagicMock(return_value={'message': {'content': json.dumps(content)}})
    return resp


def test_ollama_success():
    backend = OllamaBackend(model='qwen2.5:7b')

    fake_resp = _ollama_response({
        'action': 'grab', 'colour': 'green', 'confidence': 0.88,
    })

    with patch('httpx.Client') as MockClient:
        instance = MockClient.return_value.__enter__.return_value
        instance.post = MagicMock(return_value=fake_resp)

        intent = backend.parse_intent('возьми зелёный мяч')

    assert intent is not None
    assert intent.action == 'grab'
    assert intent.colour == 'green'
    assert intent.source == 'llm'
    assert intent.raw_text == 'возьми зелёный мяч'


def test_ollama_http_error_returns_none():
    backend = OllamaBackend()

    with patch('httpx.Client') as MockClient:
        instance = MockClient.return_value.__enter__.return_value
        instance.post = MagicMock(side_effect=Exception('connection refused'))

        intent = backend.parse_intent('hi')

    assert intent is None


def test_ollama_empty_content_returns_none():
    backend = OllamaBackend()
    resp = MagicMock()
    resp.raise_for_status = MagicMock()
    resp.json = MagicMock(return_value={'message': {'content': ''}})

    with patch('httpx.Client') as MockClient:
        instance = MockClient.return_value.__enter__.return_value
        instance.post = MagicMock(return_value=resp)
        intent = backend.parse_intent('hi')

    assert intent is None


def test_ollama_invalid_json_in_content_returns_none():
    backend = OllamaBackend()
    resp = MagicMock()
    resp.raise_for_status = MagicMock()
    resp.json = MagicMock(return_value={'message': {'content': 'sorry, no JSON for you'}})

    with patch('httpx.Client') as MockClient:
        instance = MockClient.return_value.__enter__.return_value
        instance.post = MagicMock(return_value=resp)
        intent = backend.parse_intent('hi')

    assert intent is None
