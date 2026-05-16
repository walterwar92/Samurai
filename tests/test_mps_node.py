"""Tests for pi_nodes.nodes.mps_node.

Stratégie: instead of pulling testcontainers + Mosquitto (heavy, optional),
we mock the MqttNode connection layer directly. Each test instantiates an
MpsNode with patched _client / publish / subscribe, then drives it through
its public callbacks (_on_matrices_set, _on_scenario_run, ...).

Coverage of:
  • matrices/set → reload + rebuild → ack;
  • scenario/run → tick produces telemetry + cmd_vel;
  • scenario/abort → status=aborted + cmd_vel=[0,0];
  • pre-validate (distance > 5, v_target > 0.30) → mps/error;
  • abort during in-flight run.

Note on testcontainers: the spec (§8.1) prefers real MQTT integration; the
docstring of test_mqtt_integration.py confirms it's optional in CI. PR
description honestly notes the fallback.
"""
from __future__ import annotations

import math
import os
import sys
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

scipy = pytest.importorskip('scipy')


# ── Fixture: an MpsNode with all paho-mqtt mocked out ──────────────────
@pytest.fixture
def mps_node():
    """Build MpsNode with paho client and the create_timer plumbing
    stubbed. Each test drives behaviour through public callbacks."""
    from pi_nodes.nodes.mps_node import MpsNode

    # Patch BEFORE __init__ so connect_async / loop_start / timers are no-ops.
    with patch('pi_nodes.mqtt_node.mqtt.Client') as MockClient:
        mock_client = MagicMock()
        MockClient.return_value = mock_client
        with patch.object(MpsNode, 'create_timer', lambda self, period, cb: None):
            with patch.object(MpsNode, 'subscribe', lambda *a, **kw: None):
                node = MpsNode()
        # Capture publishes for assertions
        node._published: list[tuple[str, dict, int]] = []

        def _capture(suffix, payload, qos=0, retain=False):
            node._published.append((suffix, payload, qos))

        node.publish = _capture  # type: ignore[assignment]
        yield node


# ── matrices/set ───────────────────────────────────────────────────────
def _good_matrices() -> dict:
    """Каноническая НЕПРЕРЫВНАЯ модель [s, v, θ, ω, e_int]."""
    tau_v, tau_w = 0.15, 0.10
    return {
        'A': [
            [0.0,  1.0,        0.0,  0.0,        0.0],
            [0.0, -1.0/tau_v,  0.0,  0.0,        0.0],
            [0.0,  0.0,        0.0,  1.0,        0.0],
            [0.0,  0.0,        0.0, -1.0/tau_w,  0.0],
            [0.0,  0.0,       -1.0,  0.0,        0.0],
        ],
        'B': [
            [0.0,        0.0],
            [1.0/tau_v,  0.0],
            [0.0,        0.0],
            [0.0,        1.0/tau_w],
            [0.0,        0.0],
        ],
        'C': [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)],
        'D': [[0.0, 0.0] for _ in range(5)],
        'Q_diag': [10, 10, 5, 1, 1],
        'R_diag': [1, 1],
        'horizon_N': 10,
        'u_min': [-0.30, -2.0],
        'u_max': [0.30, 2.0],
        'schema_version': '1.0',
    }


def test_matrices_set_acks_with_applied(mps_node):
    mps_node._on_matrices_set('mps/matrices/set', _good_matrices())
    topics = [p[0] for p in mps_node._published]
    assert 'mps/matrices/applied' in topics
    # No error
    assert 'mps/error' not in topics


def test_matrices_set_bad_payload_publishes_error(mps_node):
    mps_node._on_matrices_set('mps/matrices/set', "not a dict")
    topics = [p[0] for p in mps_node._published]
    assert topics == ['mps/error']
    err = mps_node._published[0][1]
    assert err['error_type'] == 'precondition'


def test_matrices_set_during_run_rejected(mps_node):
    # Start a run first
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r1',
        'request': {'distance': 1.0, 'v_target': 0.10, 'source': 'robot'},
    })
    assert mps_node.is_running
    mps_node._on_matrices_set('mps/matrices/set', _good_matrices())
    err_payloads = [p[1] for p in mps_node._published if p[0] == 'mps/error']
    assert any(e['error_type'] == 'precondition' for e in err_payloads)


def test_matrices_set_discretizes_before_rebuild(mps_node):
    """A/B приходят непрерывными; mpc/plant должны получить ZOH-дискретные
    Ad/Bd, а не сырые непрерывные значения."""
    import numpy as np
    mps_node._on_matrices_set('mps/matrices/set', _good_matrices())
    # Непрерывная A_c[1][1] = -1/0.15 ≈ -6.667; дискретная Ad[1][1] =
    # exp(-6.667·0.02) ≈ 0.8752 — должна быть в (0, 1).
    assert 0.0 < mps_node._mpc.Ad[1][1] < 1.0
    assert 0.0 < mps_node._plant.Ad[1][1] < 1.0
    np.testing.assert_allclose(
        mps_node._mpc.Ad[1][1], np.exp(-1.0 / 0.15 * 0.02), atol=1e-9
    )


def test_bootstrap_builds_canonical_controller_not_legacy(mps_node):
    """На старте (БЕЗ mps/matrices/set) mps_node обязан строить контроллер
    из канонической секции `mps:` config.yaml ([s, v, θ, ω, e_int]),
    а НЕ из legacy-секции `control:` ([px, py, θ, v, ω] +
    control.matrices.K_mpc).

    Регрессия: безаргументные StateSpaceModel()/MPCController() читают
    namespace control.* — bootstrap-MPC трактовал каноническое x[1]=v как
    поперечную координату py, видел py_ref=v_target=0.15 и упирал
    cmd_vel.angular_z ≈ 0.41 рад/с → робот ехал по кругу, хотя симулятор
    (всегда канонические матрицы) ехал прямо.
    """
    # x_meas стоящего ровно робота на старте сценария; x_ref — «ехать
    # вперёд D=2 м, держать v=0.15». У прямого канонического контроллера
    # нет повода поворачивать → angular_z строго 0.
    x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    x_ref = np.array([2.0, 0.15, 0.0, 0.0, 0.0])
    u = mps_node._mpc.step(x, x_ref=x_ref)
    assert abs(u[1]) < 1e-6, (
        f'angular_z={u[1]:.4f} ≠ 0 — bootstrap взял legacy-контроллер '
        f'(control.matrices.K_mpc); робот поедет по кругу'
    )
    assert u[0] > 0.0, f'linear_x={u[0]:.4f} должен толкать вперёд'
    # Канон дискретизируется при mps.plant.Ts=0.02: A_c[1][1]=-1/0.15 →
    # Ad[1][1]=exp(-6.667·0.02)≈0.875 ∈ (0,1). Legacy-discrete дал бы 1.0.
    assert 0.0 < mps_node._mpc.Ad[1][1] < 1.0


# ── scenario/run pre-validate ─────────────────────────────────────────
def test_scenario_run_distance_above_cap_errors(mps_node):
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r1',
        'request': {'distance': 50.0, 'v_target': 0.10, 'source': 'robot'},
    })
    err = [p[1] for p in mps_node._published if p[0] == 'mps/error']
    assert err, 'precondition error expected'
    assert err[0]['error_type'] == 'precondition'
    assert not mps_node.is_running


def test_scenario_run_v_target_above_cap_errors(mps_node):
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r1',
        'request': {'distance': 1.0, 'v_target': 0.50, 'source': 'robot'},
    })
    err = [p[1] for p in mps_node._published if p[0] == 'mps/error']
    assert err
    assert err[0]['error_type'] == 'precondition'


def test_scenario_run_starts_drive_forward_mps_state(mps_node):
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-good',
        'request': {'distance': 1.0, 'v_target': 0.10, 'source': 'robot'},
    })
    assert mps_node.fsm_state == 'DRIVE_FORWARD_MPS'
    assert mps_node.is_running


# ── tick → cmd_vel + telemetry ────────────────────────────────────────
def test_tick_publishes_cmd_vel_and_telemetry(mps_node):
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r1',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    # Provide a non-zero odom so watchdog doesn't trip immediately.
    mps_node._on_odom('odom', {
        'x': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0,
    })
    mps_node._published.clear()

    mps_node._tick()

    topics = [p[0] for p in mps_node._published]
    assert 'cmd_vel' in topics
    assert 'mps/telemetry' in topics
    cmd_vel = next(p[1] for p in mps_node._published if p[0] == 'cmd_vel')
    assert 'linear_x' in cmd_vel
    assert 'angular_z' in cmd_vel
    # Omega is clamped
    assert abs(cmd_vel['angular_z']) <= 0.5 + 1e-9


# ── abort ──────────────────────────────────────────────────────────────
def test_abort_stops_run_and_emits_zero_cmd_vel(mps_node):
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-abort',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    mps_node._on_scenario_abort('mps/scenario/abort', {'run_id': 'r-abort'})

    # Failsafe: 3× cmd_vel = [0, 0]
    zero_cmds = [p[1] for p in mps_node._published
                 if p[0] == 'cmd_vel'
                 and p[1].get('linear_x') == 0.0
                 and p[1].get('angular_z') == 0.0]
    assert len(zero_cmds) >= 3

    finished = [p[1] for p in mps_node._published
                if p[0] == 'mps/scenario/finished']
    assert finished and finished[0]['status'] == 'aborted'

    assert mps_node.fsm_state == 'IDLE'
    assert not mps_node.is_running


# ── tick reaches goal → status='reached' ──────────────────────────────
def test_tick_reaches_goal(mps_node):
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-reach',
        'request': {'distance': 1.0, 'v_target': 0.15, 'source': 'robot'},
    })
    # Inject an odom snapshot AT the goal.
    mps_node._x_meas = np.array([1.0, 0.15, 0.0, 0.0, 0.0])
    import time
    mps_node._x_meas_ts = time.time()
    mps_node._tick()

    # Either reached this tick or next; if reached, scenario/finished
    # with status='reached' should be in published topics.
    finished = [p[1] for p in mps_node._published
                if p[0] == 'mps/scenario/finished']
    assert finished, 'expected scenario/finished after reaching goal'
    assert finished[0]['status'] == 'reached'


# ── FSM state DRIVE_FORWARD_MPS is registered ─────────────────────────
def test_drive_forward_mps_is_in_fsm_states():
    from pi_nodes.nodes.fsm_node import State, _ALL_STATES
    assert State.DRIVE_FORWARD_MPS == 'DRIVE_FORWARD_MPS'
    assert State.DRIVE_FORWARD_MPS in _ALL_STATES


# ── odom unit conversion cm→m ──────────────────────────────────────────
def test_on_odom_converts_cm_to_metres(mps_node):
    """motor_node публикует odom['x'] в САНТИМЕТРАХ (motor_node.py:804).
    _on_odom должен конвертировать см→м БЕЗУСЛОВНО. Старая эвристика
    `if abs(s) > 20` оставляла 0-20 см не сконвертированными → x_meas[0]
    в 100× раз больше → MPC упирал cmd_vel в u_max."""
    mps_node._on_odom('odom', {'x': 10.0, 'vx': 0.05, 'theta': 0.0, 'vz': 0.0})
    assert mps_node._x_meas[0] == pytest.approx(0.10)   # 10 см → 0.10 м
    mps_node._on_odom('odom', {'x': 5.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    assert mps_node._x_meas[0] == pytest.approx(0.05)    # старый код: 5.0
    mps_node._on_odom('odom', {'x': 150.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    assert mps_node._x_meas[0] == pytest.approx(1.50)    # 150 см → 1.5 м


def test_tick_position_is_scenario_relative(mps_node):
    """mps_node снапшотит позицию одометрии на старте сценария: x_meas[0]
    в тике считается ОТНОСИТЕЛЬНО точки старта (s_ref начинается с 0).
    Иначе накопленная dead-reckoning одометрия даёт фантомную ошибку
    позиции на t=0."""
    # Робот стоит на 1.2 м (120 см) по одометрии ДО старта сценария.
    mps_node._on_odom('odom', {'x': 120.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-rel',
        'request': {'distance': 2.0, 'v_target': 0.10, 'source': 'robot'},
    })
    # Ещё odom на той же позиции (робот не двинулся).
    mps_node._on_odom('odom', {'x': 120.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    tel = [p[1] for p in mps_node._published if p[0] == 'mps/telemetry']
    assert tel, 'expected mps/telemetry to be published'
    s_rel = tel[0]['point']['x'][0]
    assert abs(s_rel) < 0.01, f'position should be scenario-relative (~0), got {s_rel}'


def test_tick_heading_is_scenario_relative(mps_node):
    """mps_node снапшотит курс одометрии на старте сценария: x_meas[2] (θ)
    в тике считается ОТНОСИТЕЛЬНО курса старта (θ_ref начинается с 0).
    Иначе MPC трактует x_ref[θ]=0 как абсолютный 0 одометрии и доворачивает
    робота в одну и ту же сторону вместо «ехать прямо куда смотрит»."""
    # Робот стоит под курсом 1.0 рад по одометрии ДО старта сценария.
    mps_node._on_odom('odom', {'x': 0.0, 'vx': 0.0, 'theta': 1.0, 'vz': 0.0})
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-theta',
        'request': {'distance': 2.0, 'v_target': 0.10, 'source': 'robot'},
    })
    # Ещё odom под тем же курсом (робот не повернулся).
    mps_node._on_odom('odom', {'x': 0.0, 'vx': 0.0, 'theta': 1.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    tel = [p[1] for p in mps_node._published if p[0] == 'mps/telemetry']
    assert tel, 'expected mps/telemetry to be published'
    theta_rel = tel[0]['point']['x'][2]
    assert abs(theta_rel) < 0.01, (
        f'heading should be scenario-relative (~0), got {theta_rel}'
    )
    # Робот смотрит «прямо» относительно старта → MPC не должен доворачивать.
    cmd_vel = next(p[1] for p in mps_node._published if p[0] == 'cmd_vel')
    assert abs(cmd_vel['angular_z']) < 1e-6, (
        f"angular_z={cmd_vel['angular_z']:.4f} ≠ 0 — робот доворачивает к "
        f'абсолютному курсу 0 вместо «вперёд куда смотрит»'
    )


def test_on_scenario_run_reads_target_heading(mps_node):
    """_on_scenario_run читает target_heading из request и стартует
    в фазе 'turn'."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-th',
        'request': {'distance': 2.0, 'v_target': 0.10, 'source': 'robot',
                    'target_heading': 0.6},
    })
    assert mps_node.is_running
    assert mps_node._run.target_heading == pytest.approx(0.6)
    assert mps_node._run.phase == 'turn'
    assert mps_node._run.drive_t == 0.0


def test_on_scenario_run_target_heading_defaults_zero(mps_node):
    """Без target_heading в request — дефолт 0.0 (поведение «вперёд»)."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-th0',
        'request': {'distance': 2.0, 'v_target': 0.10, 'source': 'robot'},
    })
    assert mps_node.is_running
    assert mps_node._run.target_heading == 0.0


def test_on_scenario_run_target_heading_out_of_range_rejected(mps_node):
    """target_heading вне [−π, π] → mps/error precondition, run не стартует."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-bad',
        'request': {'distance': 2.0, 'v_target': 0.10, 'source': 'robot',
                    'target_heading': 4.0},
    })
    err = [p[1] for p in mps_node._published if p[0] == 'mps/error']
    assert err and err[0]['error_type'] == 'precondition'
    assert not mps_node.is_running


def test_mps_node_loads_turn_config(mps_node):
    """__init__ читает пороги turn-фазы из config (с дефолтами)."""
    assert isinstance(mps_node._turn_tol, float) and mps_node._turn_tol > 0
    assert isinstance(mps_node._turn_timeout, float) and mps_node._turn_timeout > 0
    assert isinstance(mps_node._omega_max_turn, float) and mps_node._omega_max_turn > 0


def test_tick_turn_rotates_toward_target_heading(mps_node):
    """В фазе TURN робот крутится к target_heading: φ>0 ⇒ angular_z>0
    (CCW), ход linear_x = 0 (чистое вращение)."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-turn',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot',
                    'target_heading': 0.8},
    })
    mps_node._on_odom('odom', {'x': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    assert mps_node._run is not None and mps_node._run.phase == 'turn'
    cmd_vel = next(p[1] for p in mps_node._published if p[0] == 'cmd_vel')
    assert cmd_vel['linear_x'] == 0.0, 'в TURN ход должен быть 0 (чистое вращение)'
    assert cmd_vel['angular_z'] > 0.0, 'φ>0 ⇒ робот крутится CCW'


def test_tick_turn_transitions_to_drive_when_aligned(mps_node):
    """Когда |θ − φ| < turn_tol, фаза переключается на 'drive'."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-trans',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot',
                    'target_heading': 0.8},
    })
    # Одометрия: курс робота уже совпал с целью φ.
    mps_node._on_odom('odom', {'x': 0.0, 'vx': 0.0, 'theta': 0.8, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    assert mps_node._run is not None
    assert mps_node._run.phase == 'drive', 'курс совпал с φ ⇒ переход в DRIVE'


def test_tick_drive_holds_target_heading(mps_node):
    """В фазе DRIVE θ_ref = φ: если курс робота ниже φ, контроллер
    доворачивает ВВЕРХ к φ (angular_z>0), а не вниз к 0."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-hold',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot',
                    'target_heading': 0.8},
    })
    # Перевести в DRIVE: одометрия с курсом = φ.
    mps_node._on_odom('odom', {'x': 0.0, 'vx': 0.0, 'theta': 0.8, 'vz': 0.0})
    mps_node._tick()
    assert mps_node._run.phase == 'drive'
    # Курс робота «сполз» ниже φ (0.6 < 0.8).
    mps_node._on_odom('odom', {'x': 0.0, 'vx': 0.0, 'theta': 0.6, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    cmd_vel = next(p[1] for p in mps_node._published if p[0] == 'cmd_vel')
    assert cmd_vel['angular_z'] > 0.0, (
        'курс 0.6 < φ=0.8 ⇒ доворот вверх к φ; '
        'если бы θ_ref был 0 — angular_z был бы < 0'
    )


def test_tick_turn_timeout(mps_node):
    """Если TURN не сходится за turn_timeout — прогон завершается timeout."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-tto',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot',
                    'target_heading': 3.0},
    })
    max_ticks = int(mps_node._turn_timeout / mps_node._tick_dt) + 10
    for _ in range(max_ticks):
        # Робот «застрял»: курс 0, далеко от φ=3.0 — TURN не сойдётся.
        mps_node._on_odom('odom', {'x': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
        mps_node._tick()
        if not mps_node.is_running:
            break
    finished = [p[1] for p in mps_node._published
                if p[0] == 'mps/scenario/finished']
    assert finished and finished[-1]['status'] == 'timeout'
    assert not mps_node.is_running


def test_tick_transition_publishes_in_same_tick(mps_node):
    """На тике, где TURN завершается, _tick проваливается в _tick_drive
    тем же вызовом и публикует cmd_vel + телеметрию — без «пропущенного»
    тика. Если бы оркестратор делал return после завершения TURN,
    transition-тик не опубликовал бы ничего."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-fallthrough',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot',
                    'target_heading': 0.8},
    })
    # Робот уже под курсом φ → первый же тик: TURN завершается и
    # проваливается в DRIVE тем же вызовом _tick().
    mps_node._on_odom('odom', {'x': 0.0, 'vx': 0.0, 'theta': 0.8, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    assert mps_node._run is not None and mps_node._run.phase == 'drive', (
        'TURN должен завершиться этим тиком'
    )
    topics = [p[0] for p in mps_node._published]
    assert 'cmd_vel' in topics, 'transition-тик обязан опубликовать cmd_vel'
    assert 'mps/telemetry' in topics, 'transition-тик обязан опубликовать телеметрию'


# ── Outer LQR-loop (lateral drift correction) ─────────────────────────
# Проверяет: загрузку config-блока mps.scenario.lateral.*, построение
# LateralLqrController при старте сценария, гейтинг по enabled/v_min,
# знак коррекции при синтетическом сносе, наличие/отсутствие e_y-полей
# в фазах DRIVE/TURN и schema_version='1.1'.

def test_mps_node_loads_lateral_config(mps_node):
    """__init__ читает блок mps.scenario.lateral.* в instance attrs."""
    assert mps_node._lateral_enabled is True      # default в config.yaml
    assert mps_node._lateral_tau_inner > 0
    assert len(mps_node._lateral_q) == 2
    assert len(mps_node._lateral_r) == 1
    assert mps_node._lateral_delta_max > 0
    assert mps_node._lateral_v_min > 0


def test_on_odom_parses_y_field(mps_node):
    """_on_odom извлекает `y` из payload (см→м) и сохраняет в self._y_abs.
    Раньше y игнорировался — outer LQR-петля требует абсолютную (x, y)
    для проекции e_y на ideal-line."""
    mps_node._on_odom('odom', {'x': 100.0, 'y': 25.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    assert mps_node._x_abs == pytest.approx(1.0)
    assert mps_node._y_abs == pytest.approx(0.25)


def test_on_odom_y_defaults_to_zero_when_missing(mps_node):
    """Старая одометрия без поля y: _y_abs = 0.0 (старый клиент не должен
    падать)."""
    mps_node._on_odom('odom', {'x': 50.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    assert mps_node._y_abs == pytest.approx(0.0)


def test_scenario_run_builds_lateral_lqr_when_enabled(mps_node):
    """С enabled=true и v_target>=v_min — outer LQR строится."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-lat',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    assert mps_node._run is not None
    assert mps_node._run.lateral_lqr is not None
    # K_lat имеет форму (1, 2) — один выход, два стейта.
    assert mps_node._run.lateral_lqr.K.shape == (1, 2)
    assert mps_node._run.lateral_lqr.is_stable()


def test_scenario_run_skips_lateral_when_disabled(mps_node):
    """С enabled=false — outer LQR не строится, lateral_lqr=None."""
    mps_node._lateral_enabled = False
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-lat-off',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    assert mps_node._run is not None
    assert mps_node._run.lateral_lqr is None


def test_scenario_run_skips_lateral_when_v_target_below_min(mps_node):
    """v_target < v_min — подсистема неуправляема (v0≈0), outer LQR не
    строится. Гейтинг по v_min защищает от ValueError из __init__."""
    mps_node._lateral_v_min = 0.20    # выше v_target=0.10
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-slow',
        'request': {'distance': 2.0, 'v_target': 0.10, 'source': 'robot'},
    })
    assert mps_node._run is not None
    assert mps_node._run.lateral_lqr is None


def test_scenario_run_snapshots_absolute_xy(mps_node):
    """На старте сценария x_start_abs/y_start_abs снимаются из последней
    одометрии (для проекции e_y на ideal-line)."""
    # Робот на (50 см, 30 см) по одометрии.
    mps_node._on_odom('odom', {'x': 50.0, 'y': 30.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-snap',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    assert mps_node._run.x_start_abs == pytest.approx(0.50)
    assert mps_node._run.y_start_abs == pytest.approx(0.30)
    # line_dir = theta_start + target_heading = 0 + 0 = 0.
    assert mps_node._run.line_dir == pytest.approx(0.0)


def test_scenario_run_line_dir_includes_target_heading(mps_node):
    """line_dir = theta_start_abs + target_heading. Если робот стоит под
    курсом 0.3 и сценарий «вперёд относительно старта + 0.5 рад», то
    абсолютное направление прямой = 0.8 рад."""
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.3, 'vz': 0.0})
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-line',
        'request': {'distance': 1.0, 'v_target': 0.15, 'source': 'robot',
                    'target_heading': 0.5},
    })
    assert mps_node._run.line_dir == pytest.approx(0.8)


def test_tick_drive_publishes_lateral_telemetry_fields(mps_node):
    """В фазе DRIVE точка телеметрии содержит e_y, theta_err, delta_theta
    и schema_version='1.1'."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-tel',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    # target_heading=0, theta=0 ⇒ TURN сразу пройдёт и проваливается в DRIVE.
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    tel = [p for p in mps_node._published if p[0] == 'mps/telemetry']
    assert tel, 'telemetry must be published'
    payload = tel[0][1]
    assert payload['schema_version'] == '1.1'
    point = payload['point']
    assert 'e_y' in point
    assert 'theta_err' in point
    assert 'delta_theta' in point
    # Без сноса: e_y=0, delta_theta=0.
    assert point['e_y'] == pytest.approx(0.0)
    assert point['delta_theta'] == pytest.approx(0.0)


def test_tick_drive_correction_sign_for_positive_lateral_drift(mps_node):
    """Робот съехал влево (e_y > 0) при target_heading=0 ⇒ δθ < 0
    (поворот направо, чтобы вернуться на линию)."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-drift+',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    # (x,y)=(0,0), курс 0 ⇒ TURN мгновенно проходит, фаза → DRIVE.
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._tick()
    assert mps_node._run.phase == 'drive'
    # Робот проехал 10 см вперёд и сместился +5 см вбок (e_y = +0.05).
    mps_node._on_odom('odom', {'x': 10.0, 'y': 5.0, 'vx': 0.10, 'theta': 0.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    point = next(p[1] for p in mps_node._published if p[0] == 'mps/telemetry')['point']
    assert point['e_y'] > 0, f"ожидаем e_y>0, got {point['e_y']}"
    assert point['delta_theta'] < 0, (
        f"e_y>0 ⇒ δθ<0 (поворот направо), got {point['delta_theta']}"
    )


def test_tick_drive_correction_sign_for_negative_lateral_drift(mps_node):
    """Робот съехал вправо (e_y < 0) ⇒ δθ > 0 (поворот налево)."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-drift-',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._tick()    # → DRIVE
    mps_node._on_odom('odom', {'x': 10.0, 'y': -5.0, 'vx': 0.10, 'theta': 0.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    point = next(p[1] for p in mps_node._published if p[0] == 'mps/telemetry')['point']
    assert point['e_y'] < 0
    assert point['delta_theta'] > 0


def test_tick_drive_delta_theta_clipped_to_max(mps_node):
    """Огромный e_y ⇒ δθ клипуется к ±delta_theta_max."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-clip',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._tick()    # → DRIVE
    # 5 метров вбок — синтетический предел.
    mps_node._on_odom('odom', {'x': 10.0, 'y': 500.0, 'vx': 0.10, 'theta': 0.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    point = next(p[1] for p in mps_node._published if p[0] == 'mps/telemetry')['point']
    assert abs(point['delta_theta']) == pytest.approx(mps_node._lateral_delta_max, abs=1e-9)


def test_tick_turn_does_not_populate_lateral_fields(mps_node):
    """TURN-фаза публикует телеметрию без e_y/theta_err/delta_theta —
    эти поля имеют смысл только в DRIVE (при движении прямо)."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-turn-tel',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot',
                    'target_heading': 0.8},
    })
    # Робот ещё далеко от φ=0.8 → TURN продолжается.
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    assert mps_node._run.phase == 'turn'
    point = next(p[1] for p in mps_node._published if p[0] == 'mps/telemetry')['point']
    assert 'e_y' not in point
    assert 'theta_err' not in point
    assert 'delta_theta' not in point


def test_tick_drive_with_disabled_outer_loop_zero_correction(mps_node):
    """С enabled=false поля e_y/theta_err публикуются (diagnostic) но
    delta_theta всегда 0 — outer-петля молча выключена."""
    mps_node._lateral_enabled = False
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-no-outer',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._tick()    # → DRIVE
    mps_node._on_odom('odom', {'x': 10.0, 'y': 5.0, 'vx': 0.10, 'theta': 0.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    point = next(p[1] for p in mps_node._published if p[0] == 'mps/telemetry')['point']
    assert point['e_y'] == pytest.approx(0.05)        # diagnostic — публикуется
    assert point['delta_theta'] == pytest.approx(0.0)  # outer выключен — коррекции нет


# ── Integral action на heading (e_int = ∫(−θ_err) dt) ─────────────────
# Модель MPS включает интегратор курсовой ошибки (A[4,2]=−1), но до этой
# фиксы mps_node никогда не обновлял e_int — он навсегда оставался 0,
# и LQR-закон был чисто пропорциональный. Эти тесты проверяют что
# накопление, reset и anti-windup работают.

def test_e_int_resets_on_scenario_start(mps_node):
    """На старте сценария e_int обнуляется. Без этого остаток с прошлого
    прогона давал фантомную ошибку и MPC «доворачивал» с первого тика."""
    # Симулируем остаток от предыдущего прогона.
    mps_node._x_meas[4] = 0.3
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-int-reset',
        'request': {'distance': 1.0, 'v_target': 0.10, 'source': 'robot'},
    })
    assert mps_node._x_meas[4] == 0.0


def test_e_int_accumulates_in_tick_drive(mps_node):
    """В фазе DRIVE e_int накапливается как ∫(−θ_err) dt. Знак: при
    положительном θ_err (робот развернулся правее цели) интеграл
    уменьшается ⇒ MPC получает дополнительный сигнал отвернуть налево."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-int-acc',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    # theta=0, phi=0 ⇒ TURN сразу пройдёт, переход в DRIVE.
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._tick()
    assert mps_node._run.phase == 'drive'
    initial_eint = mps_node._x_meas[4]
    # Курс ушёл на +0.1 рад от phi=0.
    mps_node._on_odom('odom', {'x': 5.0, 'y': 0.0, 'vx': 0.10, 'theta': 0.1, 'vz': 0.0})
    mps_node._tick()
    new_eint = mps_node._x_meas[4]
    # Ожидаем e_int += −0.1 · tick_dt = −0.002.
    expected_delta = -0.1 * mps_node._tick_dt
    assert abs((new_eint - initial_eint) - expected_delta) < 1e-9, (
        f'e_int delta {new_eint - initial_eint:.6f} ≠ ожидаемое {expected_delta:.6f}'
    )


def test_e_int_does_not_accumulate_in_turn_phase(mps_node):
    """TURN-фаза НЕ должна накапливать e_int — там θ_err большое (по
    определению), и интеграл бы насытился ещё до начала DRIVE."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-int-turn',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot',
                    'target_heading': 1.0},
    })
    # Курс 0, цель 1 рад ⇒ TURN продолжается.
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    initial_eint = mps_node._x_meas[4]
    mps_node._tick()
    assert mps_node._run.phase == 'turn'
    assert mps_node._x_meas[4] == initial_eint, 'TURN не должен трогать e_int'


def test_e_int_clipped_by_anti_windup(mps_node):
    """Под устойчивым θ_err интегратор saturates на ±e_int_max — иначе
    при долгой невозможности дотянуться (трение, насыщение u_max) e_int
    уходит в бесконечность и потом «дребезжит» при выходе из насыщения."""
    mps_node._e_int_max = 0.05    # ужесточаем для быстрого теста
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-windup',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._tick()    # → DRIVE
    # Удерживаем большое постоянное θ_err = +1 рад через повторный _on_odom.
    for _ in range(100):
        mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 1.0, 'vz': 0.0})
        mps_node._tick()
    # При θ_err=+1 и tick_dt=0.02 за 100 тиков «без клипа» получили бы −2 рад·с,
    # с anti-windup e_int_max=0.05 → должно быть −0.05.
    assert mps_node._x_meas[4] == pytest.approx(-mps_node._e_int_max, abs=1e-9)


def test_e_int_max_loaded_from_config(mps_node):
    """mps_node читает e_int_max из mps.scenario.e_int_max (default 0.5)."""
    assert isinstance(mps_node._e_int_max, float) and mps_node._e_int_max > 0


def test_tighter_q_keeps_closed_loop_stable(mps_node):
    """С новыми Q_diag=[10,10,80,5,5] закрытый контур inner-MPC всё ещё
    устойчив (|λ(Ad−Bd·K_first)| < 1). Регрессия для тюнинга весов:
    если кто-то поднимет Q_θ слишком высоко без коррекции R, MPC может
    дать неустойчивый замкнутый контур → тут поймаем."""
    eigs = np.abs(np.linalg.eigvals(
        mps_node._mpc.Ad - mps_node._mpc.Bd @ mps_node._mpc.K_first
    ))
    assert np.all(eigs < 1.0 - 1e-3), f'closed-loop unstable: |λ|={eigs}'


def test_turn_tolerance_is_three_degrees(mps_node):
    """turn_tolerance_rad ≈ 3° (0.05 рад). ±1° на реальном роботе
    недостижимо: шум IMU/трение → колебания вокруг target → TURN
    никогда не сходится → s остаётся 0 → траектория «не двигается»
    на UI. 3° — компромисс: DRIVE стартует с малой ошибкой курса
    (которую дожимает heading hold), а не висит в TURN до timeout."""
    assert mps_node._turn_tol == pytest.approx(0.05, abs=1e-4)


def test_tick_drive_correction_along_rotated_line(mps_node):
    """При target_heading=π/2 ideal-line идёт по оси Y. Робот «сдвинут» от
    линии в направлении +X (т.е. вправо относительно курса) ⇒ e_y < 0
    ⇒ δθ > 0 (поворот налево, чтобы вернуться на линию)."""
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-rot',
        'request': {'distance': 1.0, 'v_target': 0.15, 'source': 'robot',
                    'target_heading': math.pi / 2},
    })
    assert mps_node._run.line_dir == pytest.approx(math.pi / 2)
    # Чтобы пройти TURN — выставляем курс = π/2.
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': math.pi / 2, 'vz': 0.0})
    mps_node._tick()
    assert mps_node._run.phase == 'drive'
    # Робот проехал по Y и сместился +5 см по X (вправо от линии).
    # e_y = -dx*sin(line_dir) + dy*cos(line_dir) = -0.05*1 + 0.10*0 = -0.05.
    mps_node._on_odom('odom', {'x': 5.0, 'y': 10.0, 'vx': 0.10, 'theta': math.pi / 2, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    point = next(p[1] for p in mps_node._published if p[0] == 'mps/telemetry')['point']
    assert point['e_y'] == pytest.approx(-0.05, abs=1e-9)
    assert point['delta_theta'] > 0
