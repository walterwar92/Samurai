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
