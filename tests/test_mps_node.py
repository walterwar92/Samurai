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
import time
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

        # По умолчанию считаем, что свежая одометрия пришла —
        # _on_scenario_run проверяет это с 2026-05-15 (Bug A). Тесты,
        # специально проверяющие реджект stale-odom, перетирают значение.
        node._x_meas_ts = time.time()

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


# ── Bug C: reach_tolerance default 0.02 м (был 0.05 м = D/2 на D=0.10) ──
def test_reach_tolerance_default_is_2cm(mps_node):
    """Дефолт mps.scenario.reach_tolerance_m = 0.02 м.

    Reason (Bug C, diagnostics 2026-05-15): run #1 status='reached' при
    s_end=0.251 на D=0.30 — 84% дистанции засчитывалось как «достиг»
    из-за tolerance 0.05 м. Для D=0.10 это была tolerance D/2 — любой
    short hop проходил. 0.02 м сопоставимо с разрешением dead-reckoning
    одометрии.
    """
    assert mps_node._reach_eps == pytest.approx(0.02)


def test_reach_at_98_percent_with_default_tolerance(mps_node):
    """Граница reach: s=0.985 на D=1.0 → reached (1 - 0.02 = 0.98)."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-edge',
        'request': {'distance': 1.0, 'v_target': 0.10, 'source': 'robot'},
    })
    mps_node._run.phase = 'drive'  # пропускаем TURN для чистоты теста
    mps_node._x_meas = np.array([0.985, 0.10, 0.0, 0.0, 0.0])
    mps_node._x_meas_ts = time.time()
    mps_node._tick()
    finished = [p[1] for p in mps_node._published
                if p[0] == 'mps/scenario/finished']
    assert finished and finished[0]['status'] == 'reached'


def test_no_reach_at_95_percent_with_default_tolerance(mps_node):
    """Граница reach: s=0.95 на D=1.0 → НЕ reached (0.95 < 0.98)."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-no-reach',
        'request': {'distance': 1.0, 'v_target': 0.10, 'source': 'robot'},
    })
    mps_node._run.phase = 'drive'
    mps_node._x_meas = np.array([0.95, 0.10, 0.0, 0.0, 0.0])
    mps_node._x_meas_ts = time.time()
    mps_node._tick()
    finished = [p[1] for p in mps_node._published
                if p[0] == 'mps/scenario/finished'
                and p[1].get('status') == 'reached']
    assert not finished, '0.95 м из 1.0 м (95%) не должно засчитываться как reached'


# ── FSM state DRIVE_FORWARD_MPS is registered ─────────────────────────
def test_drive_forward_mps_is_in_fsm_states():
    from pi_nodes.nodes.fsm_node import State, _ALL_STATES
    assert State.DRIVE_FORWARD_MPS == 'DRIVE_FORWARD_MPS'
    assert State.DRIVE_FORWARD_MPS in _ALL_STATES


# ── odom: prefers s_body (м, body-frame) over legacy x (см, world-frame) ─
def test_on_odom_prefers_s_body_when_present(mps_node):
    """Новый motor_node публикует s_body — body-frame дистанция в метрах,
    знаковая. mps должен брать ИМЕННО её, не world.x.

    Reason: world.x = ∫v·cos(θ_abs)·dt — переворачивается в минус, если
    IMU абсолютный курс ≈ ±π (видели в diagnostics 2026-05-15: u_v>0,
    но x шёл в минус). s_body = ∫v·dt — независим от θ.
    """
    # Намеренно противоречивые поля: s_body=+0.50 м, x=-200 см (world).
    # mps должен взять s_body — не x/100=-2.0.
    mps_node._on_odom('odom', {
        's_body': 0.50, 'x': -200.0, 'vx': 0.10, 'theta': 3.14, 'vz': 0.0,
    })
    assert mps_node._x_meas[0] == pytest.approx(0.50)


def test_on_odom_falls_back_to_x_cm_when_no_s_body(mps_node):
    """Backward-compat: если s_body отсутствует (старый motor_node),
    читаем x в сантиметрах и конвертируем в метры."""
    mps_node._on_odom('odom', {'x': 10.0, 'vx': 0.05, 'theta': 0.0, 'vz': 0.0})
    assert mps_node._x_meas[0] == pytest.approx(0.10)   # 10 см → 0.10 м
    mps_node._on_odom('odom', {'x': 150.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    assert mps_node._x_meas[0] == pytest.approx(1.50)


def test_on_odom_signed_s_body(mps_node):
    """s_body знаковый — отрицательный, если робот реально едет назад."""
    mps_node._on_odom('odom', {
        's_body': -0.30, 'vx': -0.15, 'theta': 0.0, 'vz': 0.0,
    })
    assert mps_node._x_meas[0] == pytest.approx(-0.30)
    assert mps_node._x_meas[1] == pytest.approx(-0.15)


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


def test_scenario_run_rejected_when_no_odom_received(mps_node):
    """Если odom не приходил вовсе (_x_meas_ts == 0) — реджект.

    Reason (Bug A, diagnostics 2026-05-15): без свежей одометрии snapshot
    (s_start, theta_start) берётся из np.zeros() из __init__, а к первому
    тику odom приходит с реальным курсом → relative-θ становится огромным
    → робот застревает в TURN-фазе. Тихий старт с фейковым нулевым snapshot'ом
    хуже явного reject'a — пользователь думает что мпс сломан, на самом деле
    робот-стек ещё не запустился.
    """
    mps_node._x_meas_ts = 0.0  # перетираем дефолт fixture
    mps_node._published.clear()
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-no-odom',
        'request': {'distance': 1.0, 'v_target': 0.10, 'source': 'robot'},
    })
    assert not mps_node.is_running
    err = [p[1] for p in mps_node._published if p[0] == 'mps/error']
    assert err and err[0]['error_type'] == 'precondition'
    assert 'odom' in err[0]['message'].lower()


def test_scenario_run_rejected_when_odom_stale(mps_node):
    """Если последний odom старее odom_max_age (default 0.5 c) — реджект."""
    mps_node._x_meas_ts = time.time() - 5.0  # 5 секунд назад
    mps_node._published.clear()
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-stale',
        'request': {'distance': 1.0, 'v_target': 0.10, 'source': 'robot'},
    })
    assert not mps_node.is_running
    err = [p[1] for p in mps_node._published if p[0] == 'mps/error']
    assert err and err[0]['error_type'] == 'precondition'
    assert 'stale' in err[0]['message'].lower()


def test_scenario_run_accepted_with_fresh_odom(mps_node):
    """Counter-positive: при свежем _x_meas_ts старт идёт нормально."""
    mps_node._x_meas_ts = time.time()
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-fresh',
        'request': {'distance': 1.0, 'v_target': 0.10, 'source': 'robot'},
    })
    assert mps_node.is_running


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
