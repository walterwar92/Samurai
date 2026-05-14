"""
МПС — REST роутер `/api/v1/mps/*`.

Контракт фиксирован в [docs/mps/api.md](../../../docs/mps/api.md);
Pydantic-схемы — в `..schemas.mps`. Все мутации `state.mps` — внутри
`state.lock` через context-manager.

Endpoints (см. api.md §3):
  GET  /matrices                     applied + draft
  POST /matrices                     сохранить как draft
  POST /matrices/apply               apply draft (или body) → MQTT publish
  POST /matrices/reset               загрузить дефолт из config.yaml
  POST /validate                     eigenvalues + step-response
  POST /scenario/run                 sim sync / robot async
  GET  /scenario/{run_id}            poll active run / лезть в history
  POST /scenario/abort               abort current run
  GET  /history                      последние N прогонов
  POST /history/{run_id}/replay      повторить с теми же параметрами
  POST /config/save                  записать applied в config.yaml mps:

Источник дефолтов и safety caps — секция `mps:` в `config.yaml`.
"""
from __future__ import annotations

import logging
import os
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import asyncio
import json as _json
from threading import Lock as _Lock

from fastapi import APIRouter, Body, HTTPException, WebSocket, WebSocketDisconnect

from ..schemas.common import OkResponse
from ..schemas.mps import (
    ComplexNumber,
    MpsConfigSaveResponse,
    MpsHistoryResponse,
    MpsMatrices,
    MpsMatricesGetResponse,
    MpsMatricesSetResponse,
    MpsScenarioAbortResponse,
    MpsScenarioRequest,
    MpsScenarioResult,
    MpsScenarioRunResponse,
    MpsValidateResult,
)
from ._deps import MQTTDep, StateDep

log = logging.getLogger(__name__)

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda key, default=None: default  # type: ignore

router = APIRouter()


# ── Helpers ────────────────────────────────────────────────────────────
def _default_matrices_from_config() -> MpsMatrices:
    """Build a MpsMatrices object from the `mps:` section of config.yaml.

    Raises if the section is missing or shapes are wrong (caller turns
    into HTTP 500).
    """
    A = cfg('mps.matrices.A', None)
    B = cfg('mps.matrices.B', None)
    C = cfg('mps.matrices.C', None)
    D = cfg('mps.matrices.D', None)
    Q = cfg('mps.weights.Q_diag', None)
    R = cfg('mps.weights.R_diag', None)
    horizon_N = cfg('mps.horizon_N', None)
    u_min = cfg('mps.limits.u_min', None)
    u_max = cfg('mps.limits.u_max', None)
    if any(v is None for v in (A, B, Q, R, horizon_N, u_min, u_max)):
        raise RuntimeError(
            'config.yaml: section mps: is incomplete '
            '(need matrices.A/B, weights.Q_diag/R_diag, horizon_N, limits.u_min/u_max)'
        )
    if C is None:
        C = [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)]
    if D is None:
        D = [[0.0, 0.0] for _ in range(5)]
    return MpsMatrices(
        A=A, B=B, C=C, D=D,
        Q_diag=Q, R_diag=R,
        horizon_N=int(horizon_N),
        u_min=u_min, u_max=u_max,
    )


def _ensure_applied(state) -> MpsMatrices:
    """Return state.mps.applied or lazily seed from config defaults."""
    with state.lock:
        if state.mps.applied is not None:
            return state.mps.applied
    seeded = _default_matrices_from_config()
    with state.lock:
        if state.mps.applied is None:
            state.mps.applied = seeded
    state.mark_dirty()
    return seeded


def _scenario_pre_validate(req: MpsScenarioRequest) -> Optional[str]:
    """Pre-validate against config-driven safety caps. Returns error
    message or None if OK."""
    distance_max = float(cfg('mps.scenario.distance_max', 5.0))
    v_target_max = float(cfg('mps.scenario.v_target_max', 0.30))
    if req.distance > distance_max:
        return f'distance > distance_max={distance_max}'
    if req.v_target > v_target_max:
        return f'v_target > v_target_max={v_target_max}'
    return None


# ── /matrices ──────────────────────────────────────────────────────────
@router.get('/matrices', response_model=MpsMatricesGetResponse, tags=['mps'])
async def get_matrices(state: StateDep) -> MpsMatricesGetResponse:
    """Return currently applied matrices + (optional) unsaved draft."""
    applied = _ensure_applied(state)
    with state.lock:
        draft = state.mps.draft
    return MpsMatricesGetResponse(applied=applied, draft=draft)


@router.post('/matrices', response_model=MpsMatricesSetResponse, tags=['mps'])
async def set_matrices_draft(
    matrices: MpsMatrices,
    state: StateDep,
) -> MpsMatricesSetResponse:
    """Save matrices as `draft` — does NOT publish to robot."""
    with state.lock:
        state.mps.draft = matrices
    state.mark_dirty()
    return MpsMatricesSetResponse(status='draft', matrices=matrices)


@router.post('/matrices/apply', response_model=MpsMatricesSetResponse, tags=['mps'])
async def apply_matrices(
    state: StateDep,
    mqtt: MQTTDep,
    matrices: Optional[MpsMatrices] = Body(default=None),
) -> MpsMatricesSetResponse:
    """Apply draft (or supplied matrices). Publishes MQTT `mps/matrices/set`.

    Behaviour:
      • body не пустой → applied = body, draft = None;
      • body пустой и draft есть → applied = draft, draft = None;
      • body пустой и draft пустой → 400.
    """
    with state.lock:
        target = matrices or state.mps.draft
    if target is None:
        raise HTTPException(400, 'no draft and no body — nothing to apply')

    # Publish to robot (best-effort: dashboard остаётся в applied даже
    # если MQTT временно offline — sim продолжит работать).
    try:
        mqtt.publish('mps/matrices/set', target.model_dump(), qos=1)
    except Exception as exc:  # noqa: BLE001
        log.warning('mps/matrices/set publish failed: %s', exc)

    with state.lock:
        state.mps.applied = target
        state.mps.draft = None
    state.mark_dirty()
    return MpsMatricesSetResponse(status='applied', matrices=target)


@router.post('/matrices/reset', response_model=MpsMatricesSetResponse, tags=['mps'])
async def reset_matrices(
    state: StateDep,
    mqtt: MQTTDep,
) -> MpsMatricesSetResponse:
    """Load defaults from config.yaml mps: section. Drops draft.

    Publishes MQTT (best-effort).
    """
    try:
        defaults = _default_matrices_from_config()
    except Exception as exc:
        raise HTTPException(500, f'failed to load mps defaults: {exc}') from exc

    try:
        mqtt.publish('mps/matrices/set', defaults.model_dump(), qos=1)
    except Exception as exc:  # noqa: BLE001
        log.warning('mps/matrices/set (reset) publish failed: %s', exc)

    with state.lock:
        state.mps.applied = defaults
        state.mps.draft = None
    state.mark_dirty()
    return MpsMatricesSetResponse(status='applied', matrices=defaults)


# ── /validate ──────────────────────────────────────────────────────────
@router.post('/validate', response_model=MpsValidateResult, tags=['mps'])
async def validate(
    state: StateDep,
    matrices: Optional[MpsMatrices] = Body(default=None),
) -> MpsValidateResult:
    """Compute λ(Ad), λ(closed-loop), short step-response.

    body=None → используем draft, иначе applied. Никаких побочных
    эффектов на state.
    """
    if matrices is None:
        with state.lock:
            matrices = state.mps.draft or state.mps.applied
        if matrices is None:
            matrices = _ensure_applied(state)

    # Defer import — `compute_node.mps_runner` requires scipy.
    from compute_node.mps_runner import (  # type: ignore[attr-defined]
        closed_loop_eigenvalues,
        short_step_response,
    )

    warnings: list[str] = []
    try:
        eig_open, eig_closed = closed_loop_eigenvalues(matrices)
    except Exception as exc:  # noqa: BLE001
        warnings.append(f'eigenvalue compute failed: {exc}')
        eig_open, eig_closed = [], []

    _STAB_TOL = 1e-6
    is_plant_stable = bool(eig_open) and all(abs(z) < 1.0 - _STAB_TOL for z in eig_open)
    is_closed_stable = bool(eig_closed) and all(
        abs(z) < 1.0 - _STAB_TOL for z in eig_closed
    )
    plant_has_unstable = bool(eig_open) and any(
        abs(z) > 1.0 + _STAB_TOL for z in eig_open
    )

    try:
        step_resp = short_step_response(matrices, duration_s=2.0, dt=0.02)
    except Exception as exc:  # noqa: BLE001
        warnings.append(f'step response failed: {exc}')
        step_resp = []

    if plant_has_unstable:
        warnings.append('Открытая система неустойчива — есть |λ(Ad)| > 1')
    elif not is_plant_stable:
        warnings.append(
            'Открытая система маргинально устойчива: полюса-интеграторы '
            'на |λ|=1 (s, θ, e_int) — норма для канонической модели'
        )
    if not is_closed_stable:
        warnings.append('Замкнутая система НЕ устойчива (|λ(Ad−Bd·K)| ≥ 1)')

    return MpsValidateResult(
        eigenvalues_ad=[ComplexNumber.from_complex(z) for z in eig_open],
        eigenvalues_closed=[ComplexNumber.from_complex(z) for z in eig_closed],
        is_plant_stable=is_plant_stable,
        is_closed_loop_stable=is_closed_stable,
        step_response=step_resp,
        warnings=warnings,
    )


# ── /scenario ──────────────────────────────────────────────────────────
@router.post('/scenario/run', response_model=MpsScenarioRunResponse, tags=['mps'])
async def scenario_run(
    request: MpsScenarioRequest,
    state: StateDep,
    mqtt: MQTTDep,
) -> MpsScenarioRunResponse:
    """Запуск сценария. sim → sync; robot → async (только run_id)."""
    err = _scenario_pre_validate(request)
    if err is not None:
        raise HTTPException(400, err)

    matrices = _ensure_applied(state)

    # Conflict guard: один running прогон за раз.
    with state.lock:
        active = state.mps.active_run
    if active is not None and active.status == 'running':
        raise HTTPException(409, f'run already in progress: {active.run_id}')

    if request.source == 'sim':
        from compute_node.mps_runner import run_scenario_idealized
        result = run_scenario_idealized(matrices, request)
        with state.lock:
            state.mps.active_run = None
            state.mps.history.appendleft(result)
        state.mark_dirty()
        return MpsScenarioRunResponse(run_id=result.run_id, result=result)

    # source == 'robot' — asynchronous via MQTT.
    if not mqtt.connected:
        raise HTTPException(503, 'robot offline — MQTT broker not connected')
    run_id = f"robot-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}-{int(time.time() * 1000) % 1_000_000}"
    payload = {
        'run_id': run_id,
        'request': request.model_dump(),
        'schema_version': matrices.schema_version,
    }
    if not mqtt.publish('mps/scenario/run', payload, qos=1):
        raise HTTPException(503, 'robot offline — MQTT publish failed')

    # Stub running result — mqtt_handlers.py будет апдейтить telemetry/finished.
    pending = MpsScenarioResult(
        run_id=run_id,
        started_at=datetime.now(timezone.utc),
        finished_at=None,
        status='running',
        request=request,
        matrices_snapshot=matrices,
        telemetry=[],
        metrics=None,
    )
    with state.lock:
        state.mps.active_run = pending
        state.mps.last_telemetry.clear()
    state.mark_dirty()
    return MpsScenarioRunResponse(run_id=run_id, result=None)


@router.get('/scenario/{run_id}', response_model=MpsScenarioResult, tags=['mps'])
async def scenario_status(run_id: str, state: StateDep) -> MpsScenarioResult:
    with state.lock:
        active = state.mps.active_run
        if active is not None and active.run_id == run_id:
            return active
        for past in state.mps.history:
            if past.run_id == run_id:
                return past
    raise HTTPException(404, f'run_id {run_id!r} not found')


@router.post('/scenario/abort', response_model=MpsScenarioAbortResponse, tags=['mps'])
async def scenario_abort(state: StateDep, mqtt: MQTTDep) -> MpsScenarioAbortResponse:
    with state.lock:
        active = state.mps.active_run
    if active is None or active.status != 'running':
        return MpsScenarioAbortResponse(aborted=False, run_id=None)

    # Best-effort уведомление Pi. Дальше не зависим от его ответа — иначе
    # любой обрыв MQTT (или mps_node не поднят) залочит UI на 409 Conflict
    # при следующем /scenario/run.
    try:
        mqtt.publish('mps/scenario/abort', {'run_id': active.run_id}, qos=1)
    except Exception as exc:  # noqa: BLE001
        log.warning('mps/scenario/abort publish failed: %s', exc)

    # Локально завершаем run: переложить в history со статусом 'aborted'
    # и обнулить active_run. Если Pi всё-таки пришлёт mps/scenario/finished
    # позже, _h_mps_scenario_finished идемпотентно заменит запись по run_id.
    aborted_run = active.model_copy(update={
        'status': 'aborted',
        'finished_at': datetime.now(timezone.utc),
    })
    with state.lock:
        state.mps.active_run = None
        state.mps.history.appendleft(aborted_run)
        state.mps.last_telemetry.clear()
    state.mark_dirty()

    return MpsScenarioAbortResponse(aborted=True, run_id=active.run_id)


# ── /history ───────────────────────────────────────────────────────────
@router.get('/history', response_model=MpsHistoryResponse, tags=['mps'])
async def history(state: StateDep) -> MpsHistoryResponse:
    with state.lock:
        items = list(state.mps.history)
    return MpsHistoryResponse(history=items)


@router.post('/history/{run_id}/replay', response_model=MpsScenarioRunResponse,
             tags=['mps'])
async def history_replay(
    run_id: str,
    state: StateDep,
    mqtt: MQTTDep,
) -> MpsScenarioRunResponse:
    with state.lock:
        target: Optional[MpsScenarioResult] = None
        for past in state.mps.history:
            if past.run_id == run_id:
                target = past
                break
    if target is None:
        raise HTTPException(404, f'run_id {run_id!r} not in history')
    # Заново прогнать с теми же параметрами + теми же applied матрицами.
    return await scenario_run(target.request, state, mqtt)


# ── /config/save ───────────────────────────────────────────────────────
@router.post('/config/save', response_model=MpsConfigSaveResponse, tags=['mps'])
async def config_save(state: StateDep) -> MpsConfigSaveResponse:
    """Записать текущие applied матрицы в секцию `mps:` config.yaml.

    Race condition: «последний выиграл» — достаточно для учебного MVP
    (см. open question §11.3 спеки).
    """
    applied = _ensure_applied(state)
    cfg_path = Path(os.environ.get('SAMURAI_CONFIG', 'config.yaml'))
    if not cfg_path.is_file():
        raise HTTPException(500, f'config.yaml not found at {cfg_path}')

    try:
        import yaml
    except ImportError as exc:
        raise HTTPException(500, 'PyYAML not available') from exc

    text = cfg_path.read_text(encoding='utf-8')
    data = yaml.safe_load(text) or {}
    mps_block = data.get('mps') or {}
    mps_block.setdefault('enabled', True)
    mps_block.setdefault('scenario', {
        'distance_max': 5.0, 'v_target_max': 0.30,
        'omega_max_in_forward': 0.5,
        'default_distance': 2.0, 'default_v_target': 0.15,
    })
    mps_block.setdefault('history_size', 20)
    mps_block.setdefault('tick_dt', 0.02)
    mps_block['matrices'] = {
        'A': applied.A, 'B': applied.B,
        'C': applied.C, 'D': applied.D,
    }
    mps_block['weights'] = {
        'Q_diag': applied.Q_diag,
        'R_diag': applied.R_diag,
    }
    mps_block['horizon_N'] = applied.horizon_N
    mps_block['limits'] = {
        'u_min': applied.u_min,
        'u_max': applied.u_max,
    }
    data['mps'] = mps_block

    new_text = yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
    # Atomic write: тот же приём что в матлаб-экспорте.
    tmp = cfg_path.with_suffix(cfg_path.suffix + '.tmp')
    tmp.write_text(new_text, encoding='utf-8')
    os.replace(tmp, cfg_path)

    return MpsConfigSaveResponse(written=True, path=str(cfg_path))


# ── stub OK ping for liveness checks from frontend ────────────────────
@router.get('', response_model=OkResponse, tags=['mps'])
async def root() -> OkResponse:
    return OkResponse()


# ─────────────────────────────────────────────────────────────────────
# WebSocket /ws/mps/telemetry — Live режим во время source="robot".
# Сервер аккумулирует frames из mqtt_handlers._broadcast_mps и
# рассылает их подключённым клиентам. Клиент подписывается опционально
# на конкретный run_id; если не указан — получает все frames.
# Контракт: docs/mps/api.md §5.
# ─────────────────────────────────────────────────────────────────────
ws_router = APIRouter()


class _MpsWsBroker:
    """Тонкий fan-out: subscribers, async очередь, broadcast() из MQTT-thread."""

    def __init__(self) -> None:
        self._subs: list[tuple[asyncio.Queue, Optional[str]]] = []
        self._lock = _Lock()
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def attach_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        # Caching the loop lets us call broadcast() from a background
        # MQTT thread (paho is threaded) — call_soon_threadsafe needs it.
        self._loop = loop

    def add(self, queue: asyncio.Queue, run_id: Optional[str]) -> None:
        with self._lock:
            self._subs.append((queue, run_id))

    def remove(self, queue: asyncio.Queue) -> None:
        with self._lock:
            self._subs = [(q, r) for (q, r) in self._subs if q is not queue]

    def broadcast(self, frame: dict) -> None:
        """Called from the MQTT thread. Push frame onto each matching queue
        via call_soon_threadsafe — never blocks the publisher."""
        loop = self._loop
        with self._lock:
            targets = [
                q for (q, rid) in self._subs
                if rid is None or rid == frame.get('run_id')
            ]
        if not targets or loop is None:
            return
        for q in targets:
            loop.call_soon_threadsafe(_safe_put_nowait, q, frame)


def _safe_put_nowait(q: asyncio.Queue, item: dict) -> None:
    try:
        q.put_nowait(item)
    except asyncio.QueueFull:
        # Drop oldest then enqueue newest.
        try:
            q.get_nowait()
        except asyncio.QueueEmpty:
            return
        try:
            q.put_nowait(item)
        except asyncio.QueueFull:
            pass


# Single broker per dashboard process. app.py wires it through mqtt_handlers.
mps_broker = _MpsWsBroker()


@ws_router.websocket('/ws/mps/telemetry')
async def mps_ws(websocket: WebSocket):
    """One subscriber per WS connection. Handshake:
        client → {action:"subscribe", run_id?:string}
    Then server streams telemetry/finished/error frames until close."""
    await websocket.accept()
    if mps_broker._loop is None:
        mps_broker.attach_loop(asyncio.get_event_loop())

    queue: asyncio.Queue[dict] = asyncio.Queue(maxsize=512)
    run_id: Optional[str] = None
    mps_broker.add(queue, None)  # accept-all initially

    try:
        # Wait for at most one subscribe message; then transition to streaming.
        try:
            first = await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
            msg = _json.loads(first)
            if isinstance(msg, dict) and msg.get('action') == 'subscribe':
                run_id = msg.get('run_id') or None
                # Re-subscribe with the chosen run_id filter.
                mps_broker.remove(queue)
                mps_broker.add(queue, run_id)
                # Replay buffered telemetry if subscribing mid-run.
                state = websocket.app.state.dashboard_state  # type: ignore[attr-defined]
                with state.lock:
                    buffered = list(state.mps.last_telemetry)
                    active = state.mps.active_run
                if active is not None and (run_id is None or active.run_id == run_id):
                    for point in buffered:
                        await websocket.send_json({
                            'type': 'telemetry',
                            'run_id': active.run_id,
                            'point': point.model_dump(mode='json'),
                        })
        except (asyncio.TimeoutError, ValueError, _json.JSONDecodeError):
            # Klient никогда не пришлёт subscribe → стрим без фильтра.
            pass

        while True:
            frame = await queue.get()
            await websocket.send_json(frame)
            if frame.get('type') in ('finished', 'error') and run_id is not None:
                # Если подписан на конкретный run_id — закрываемся после finished/error.
                if frame.get('run_id') == run_id:
                    break
    except WebSocketDisconnect:
        pass
    finally:
        mps_broker.remove(queue)
        try:
            await websocket.close()
        except Exception:
            pass
