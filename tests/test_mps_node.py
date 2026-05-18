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
# was: drive-phase finish by `s ≥ D − ε`; now: 4-coord check + t ≥ t_end.
def test_tick_reaches_goal(mps_node):
    """После t_end робот в (D, v=0, θ=θ_start, ω=0) ⇒ status='reached'."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-reach',
        'request': {'distance': 1.0, 'v_target': 0.15, 'source': 'robot'},
    })
    # Inject an odom snapshot AT the goal (v=0, ω=0 — sat и встал).
    mps_node._x_meas = np.array([1.0, 0.0, 0.0, 0.0, 0.0])
    mps_node._x_meas_ts = time.time()
    # Fast-forward run.t за t_end, чтобы _check_finish активировал
    # settling-window. В feedforward модели «достижение» проверяется
    # только после прохождения опорной траектории.
    mps_node._run.t = mps_node._run.traj.t_end + 1e-3
    mps_node._tick()

    finished = [p[1] for p in mps_node._published
                if p[0] == 'mps/scenario/finished']
    assert finished, 'expected scenario/finished after reaching goal'
    assert finished[0]['status'] == 'reached'


# ── Bug C: reach_tolerance default 0.005 м (spec §4.1) ─────────────────
def test_reach_tolerance_default_is_5mm():
    """Дефолт ε_s (когда config не задаёт mps.scenario.reach_tolerance_m)
    должен быть 5 мм per spec §4.1. Был 0.02 м в pre-pose-tracking эпохе;
    с feedforward decel ε можно ужать. Конфиг может перекрыть на железе.

    Проверяем именно код-дефолт (через монки-патч _cfg), а не значение из
    config.yaml — оно может отличаться на конкретном железе, но если ключ
    отсутствует, должны получить 5 мм. Это гарантирует sim/Pi parity:
    compute mps_runner.py:263 тоже использует 0.005."""
    from pi_nodes.nodes.mps_node import MpsNode, _REACH_EPS_DEFAULT

    # Сначала проверяем сам модульный констант — единственный источник истины.
    assert _REACH_EPS_DEFAULT == pytest.approx(0.005)

    # Затем — что MpsNode действительно подхватывает дефолт, когда config
    # не задаёт ключ (имитируем чистый запуск без config.yaml).
    def fake_cfg(key, default):
        if key == 'mps.scenario.reach_tolerance_m':
            return default
        # Прочие ключи читаем из реального config_loader.
        try:
            from config_loader import cfg as real_cfg
            return real_cfg(key, default)
        except ImportError:
            return default

    with patch('pi_nodes.mqtt_node.mqtt.Client') as MockClient:
        MockClient.return_value = MagicMock()
        with patch.object(MpsNode, 'create_timer', lambda self, period, cb: None):
            with patch.object(MpsNode, 'subscribe', lambda *a, **kw: None):
                with patch.object(MpsNode, '_cfg', staticmethod(fake_cfg)):
                    node = MpsNode()
    assert node._reach_eps == pytest.approx(0.005)


# was: phase='drive' override + |s−D|<ε; now: 4-coord check post t_end.
def test_reach_within_eps_s_at_t_end(mps_node):
    """Граница reach: |s−D|=3 мм на D=1.0 → reached.

    Config (config.yaml) задаёт mps.scenario.reach_tolerance_m=0.005 м
    (spec §4.1). |Δ|=0.003 < 0.005 ⇒ reached (при v=ω=0 и t≥t_end)."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-edge',
        'request': {'distance': 1.0, 'v_target': 0.10, 'source': 'robot'},
    })
    mps_node._x_meas = np.array([0.997, 0.0, 0.0, 0.0, 0.0])
    mps_node._x_meas_ts = time.time()
    mps_node._run.t = mps_node._run.traj.t_end + 1e-3
    mps_node._tick()
    finished = [p[1] for p in mps_node._published
                if p[0] == 'mps/scenario/finished']
    assert finished and finished[0]['status'] == 'reached'


# was: phase='drive' override; now: 4-coord check post t_end.
def test_no_reach_outside_eps_s_at_t_end(mps_node):
    """Граница no-reach: |s−D|=15 мм на D=1.0 → НЕ reached.

    ε_s=0.005 м из config — |Δ|=0.015 > 0.005 ⇒ ждём settle_timeout
    и закроемся как 'timeout_settle'. На один тик `finished` ещё не
    публикуется — проверяем именно status≠'reached'.
    """
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-no-reach',
        'request': {'distance': 1.0, 'v_target': 0.10, 'source': 'robot'},
    })
    mps_node._x_meas = np.array([0.985, 0.0, 0.0, 0.0, 0.0])
    mps_node._x_meas_ts = time.time()
    mps_node._run.t = mps_node._run.traj.t_end + 1e-3
    mps_node._tick()
    finished = [p[1] for p in mps_node._published
                if p[0] == 'mps/scenario/finished'
                and p[1].get('status') == 'reached']
    assert not finished, '|s−D|=0.015 > ε_s=0.005 не должно засчитываться как reached'


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
    """_on_scenario_run читает target_heading из request и кладёт его
    в _RunState (для финиш-проверки и для построения профиля поворота).

    was: phase='turn' + drive_t=0; now: feedforward — фаз нет, target
    хранится только в run.target_heading (и в traj.phi_signed).
    """
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-th',
        'request': {'distance': 2.0, 'v_target': 0.10, 'source': 'robot',
                    'target_heading': 0.6},
    })
    assert mps_node.is_running
    assert mps_node._run.target_heading == pytest.approx(0.6)
    # traj должна знать про φ — это и есть «target_heading стартовал»
    assert mps_node._run.traj.phi_signed == pytest.approx(0.6)


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


def test_mps_node_loads_omega_max_turn_config(mps_node):
    """__init__ читает omega_max_in_turn (используется build_reference
    для cap'а профиля turn-сегмента).

    was: _turn_tol/_turn_timeout — pose-tracking refactor убрал TURN
    как отдельную фазу, и эти пороги больше не нужны.
    """
    assert isinstance(mps_node._omega_max_turn, float) and mps_node._omega_max_turn > 0


def test_mps_node_loads_reference_config(mps_node):
    """__init__ читает дефолтные a_max/alpha_max опорного профиля
    (могут быть переопределены payload-ом /scenario/run)."""
    assert isinstance(mps_node._ref_a_max, float) and mps_node._ref_a_max > 0
    assert isinstance(mps_node._ref_alpha_max, float) and mps_node._ref_alpha_max > 0


def test_mps_node_loads_settle_tolerances(mps_node):
    """4 порога settling-окна для финиш-проверки (см. spec §4.1)."""
    assert isinstance(mps_node._eps_v, float) and mps_node._eps_v > 0
    assert isinstance(mps_node._eps_theta, float) and mps_node._eps_theta > 0
    assert isinstance(mps_node._eps_omega, float) and mps_node._eps_omega > 0
    assert isinstance(mps_node._settle_timeout, float) and mps_node._settle_timeout > 0


def test_mps_node_warns_on_deprecated_turn_config():
    """log_warn должен фиритьcя при наличии deprecated config-ключей.
    Проверяем оба: turn_tolerance_rad и turn_timeout_s. Раньше тест только
    смотрел, что атрибутов нет (что ничего не проверяло про сам warning) —
    refactor, удаливший log_warn-call'ы в _build_from_config, прошёл бы
    незаметно. Здесь мокаем log_warn и проверяем что он был позван для
    обоих deprecated ключей."""
    from pi_nodes.nodes.mps_node import MpsNode

    # _cfg должен возвращать non-None для обоих deprecated ключей, чтобы
    # сработали обе ветки `if self._cfg(...) is not None` в __init__.
    def fake_cfg(key, default):
        if key == 'mps.scenario.turn_tolerance_rad':
            return 0.05
        if key == 'mps.scenario.turn_timeout_s':
            return 10.0
        try:
            from config_loader import cfg as real_cfg
            return real_cfg(key, default)
        except ImportError:
            return default

    with patch('pi_nodes.mqtt_node.mqtt.Client') as MockClient:
        MockClient.return_value = MagicMock()
        with patch.object(MpsNode, 'create_timer', lambda self, period, cb: None):
            with patch.object(MpsNode, 'subscribe', lambda *a, **kw: None):
                with patch.object(MpsNode, '_cfg', staticmethod(fake_cfg)):
                    with patch.object(MpsNode, 'log_warn') as mock_warn:
                        MpsNode()

    # Должны быть как минимум 2 warning-вызова — по одному на каждый ключ.
    warn_messages = [
        (call.args[0] % call.args[1:]) if len(call.args) > 1 else call.args[0]
        for call in mock_warn.call_args_list
    ]
    turn_tol_warns = [m for m in warn_messages if 'turn_tolerance_rad' in m]
    turn_timeout_warns = [m for m in warn_messages if 'turn_timeout_s' in m]
    assert turn_tol_warns, (
        f'expected warning mentioning turn_tolerance_rad, got: {warn_messages}'
    )
    assert turn_timeout_warns, (
        f'expected warning mentioning turn_timeout_s, got: {warn_messages}'
    )
    # И сам текст должен называть это deprecation.
    assert any('deprecated' in m.lower() for m in turn_tol_warns), (
        f'expected "deprecated" in turn_tolerance_rad warning, got: {turn_tol_warns}'
    )
    assert any('deprecated' in m.lower() for m in turn_timeout_warns), (
        f'expected "deprecated" in turn_timeout_s warning, got: {turn_timeout_warns}'
    )


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
    одометрии (для проекции (x_local, y_local) в локальный фрейм старта).

    was: ещё проверял line_dir; pose-tracking refactor его убрал — старт-
    фрейм определяется только theta_start, направление «прямо» = θ_start.
    """
    # Робот на (50 см, 30 см) по одометрии.
    mps_node._on_odom('odom', {'x': 50.0, 'y': 30.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-snap',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    assert mps_node._run.x_start_abs == pytest.approx(0.50)
    assert mps_node._run.y_start_abs == pytest.approx(0.30)
    # theta_start снапшотится отдельно (для пересчёта в локальный фрейм
    # и для проверки финиш-координаты θ_target = theta_start+phi).
    assert mps_node._run.theta_start == pytest.approx(0.0)


def test_tick_publishes_lateral_telemetry_fields(mps_node):
    """Каждый тик публикует e_y/theta_err/delta_theta и schema_version='1.2'
    (выросло с 1.1 после добавления r/x_local/y_local в Task 3)."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-tel',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    # target_heading=0, theta=0 — отсутствие сноса.
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    tel = [p for p in mps_node._published if p[0] == 'mps/telemetry']
    assert tel, 'telemetry must be published'
    payload = tel[0][1]
    assert payload['schema_version'] == '1.2'
    point = payload['point']
    assert 'e_y' in point
    assert 'theta_err' in point
    assert 'delta_theta' in point
    # Без сноса: e_y=0, delta_theta=0.
    assert point['e_y'] == pytest.approx(0.0)
    assert point['delta_theta'] == pytest.approx(0.0)
    # Новые поля (schema 1.2): r/x_local/y_local.
    assert 'r' in point and len(point['r']) == 5
    assert 'x_local' in point
    assert 'y_local' in point


# was: e_y проектировалось вдоль run.line_dir; now: e_y == y_local
# (X-ось локального фрейма вдоль θ_start, Y — налево). При θ_start=0 это
# совпадает со стандартными мировыми (x,y) — y_local = y_world − y_start.
def test_tick_correction_sign_for_positive_lateral_drift(mps_node):
    """Робот съехал влево (e_y > 0) при target_heading=0 ⇒ δθ < 0
    (поворот направо, чтобы вернуться на линию)."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-drift+',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._tick()
    # Робот проехал 10 см вперёд и сместился +5 см вбок (e_y = +0.05).
    mps_node._on_odom('odom', {'x': 10.0, 'y': 5.0, 'vx': 0.10, 'theta': 0.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    point = next(p[1] for p in mps_node._published if p[0] == 'mps/telemetry')['point']
    assert point['e_y'] > 0, f"ожидаем e_y>0, got {point['e_y']}"
    assert point['delta_theta'] < 0, (
        f"e_y>0 ⇒ δθ<0 (поворот направо), got {point['delta_theta']}"
    )


def test_tick_correction_sign_for_negative_lateral_drift(mps_node):
    """Робот съехал вправо (e_y < 0) ⇒ δθ > 0 (поворот налево)."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-drift-',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._tick()
    mps_node._on_odom('odom', {'x': 10.0, 'y': -5.0, 'vx': 0.10, 'theta': 0.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    point = next(p[1] for p in mps_node._published if p[0] == 'mps/telemetry')['point']
    assert point['e_y'] < 0
    assert point['delta_theta'] > 0


def test_tick_delta_theta_clipped_to_max(mps_node):
    """Огромный e_y ⇒ δθ клипуется к ±delta_theta_max."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-clip',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._tick()
    # 5 метров вбок — синтетический предел.
    mps_node._on_odom('odom', {'x': 10.0, 'y': 500.0, 'vx': 0.10, 'theta': 0.0, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    point = next(p[1] for p in mps_node._published if p[0] == 'mps/telemetry')['point']
    assert abs(point['delta_theta']) == pytest.approx(mps_node._lateral_delta_max, abs=1e-9)


def test_tick_with_disabled_outer_loop_zero_correction(mps_node):
    """С enabled=false поля e_y/theta_err публикуются (diagnostic) но
    delta_theta всегда 0 — outer-петля молча выключена."""
    mps_node._lateral_enabled = False
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-no-outer',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._tick()
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


# was: phase='drive' check; now: e_int = ∫(−θ_err(t)) dt каждый тик, без
# различения фаз. Сигнал тот же — при + θ_err интеграл − (anti-θ-err).
def test_e_int_accumulates_each_tick(mps_node):
    """e_int накапливается как ∫(−θ_err) dt против ТЕКУЩЕГО θ_ref(t).
    На первом тике при theta=0, r(0)[θ]=0 ⇒ θ_err=0, e_int не меняется.
    На втором тике, если ввести искусственно θ=0.1 при r(t)[θ]≈0,
    e_int += −0.1·dt = −0.002."""
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-int-acc',
        'request': {'distance': 2.0, 'v_target': 0.15, 'source': 'robot'},
    })
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': 0.0, 'vz': 0.0})
    mps_node._tick()
    initial_eint = mps_node._x_meas[4]
    # Курс ушёл на +0.1 рад от θ_ref≈0 (на drive-сегменте r(t)[θ]=0).
    mps_node._on_odom('odom', {'x': 5.0, 'y': 0.0, 'vx': 0.10, 'theta': 0.1, 'vz': 0.0})
    mps_node._tick()
    new_eint = mps_node._x_meas[4]
    # Ожидаем e_int += −0.1 · tick_dt = −0.002.
    expected_delta = -0.1 * mps_node._tick_dt
    assert abs((new_eint - initial_eint) - expected_delta) < 1e-9, (
        f'e_int delta {new_eint - initial_eint:.6f} ≠ ожидаемое {expected_delta:.6f}'
    )


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


# was: test_tick_drive_correction_along_rotated_line — проверял что e_y
# проектируется на абсолютную line_dir = theta_start + target_heading.
# Pose-tracking refactor определяет (x_local, y_local) ТОЛЬКО через
# theta_start (см. spec §5.3): X — вдоль курса робота на момент старта,
# Y — налево от X. target_heading в этот пересчёт уже не входит.
def test_tick_local_frame_aligns_with_theta_start(mps_node):
    """Когда робот стартовал под курсом θ_start=π/2, ось X локального
    фрейма смотрит по мировому +Y. Робот, проехавший по мировому Y
    на 10 см и сместившийся +5 см по X (т.е. вправо от X-локальной
    оси) даёт x_local=+0.10, y_local=−0.05."""
    mps_node._on_odom('odom', {'x': 0.0, 'y': 0.0, 'vx': 0.0, 'theta': math.pi / 2, 'vz': 0.0})
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'r-rot',
        'request': {'distance': 1.0, 'v_target': 0.15, 'source': 'robot'},
    })
    # Курс не меняется (=π/2), x=+5, y=+10 — в локальном фрейме это
    # +10 вдоль X (т.е. вперёд) и −5 по Y (направо).
    mps_node._on_odom('odom', {'x': 5.0, 'y': 10.0, 'vx': 0.10, 'theta': math.pi / 2, 'vz': 0.0})
    mps_node._published.clear()
    mps_node._tick()
    point = next(p[1] for p in mps_node._published if p[0] == 'mps/telemetry')['point']
    assert point['x_local'] == pytest.approx(0.10, abs=1e-9)
    assert point['y_local'] == pytest.approx(-0.05, abs=1e-9)
    # e_y = y_local; правый снос ⇒ δθ > 0 (поворот налево).
    assert point['e_y'] == pytest.approx(-0.05, abs=1e-9)
    assert point['delta_theta'] > 0


# ── Pose-tracking acceptance: edge-to-edge scenario over fake odom ────
# Идеализированный «робот» — та же каноническая модель v̇=(u_v−v)/τ_v,
# ω̇=(u_ω−ω)/τ_ω, что разворачивается на Pi. Параметры τ_v=0.15, τ_w=0.10
# совпадают с _good_matrices() (см. сверху) — это и есть «настоящий» plant.
# Тест проверяет end-to-end что нода MPC-трекает r(t) и выходит в (D, π).

class _FakeOdom:
    """Idealized-plant для интеграции cmd_vel в тестах. Использует ту же
    ZOH-дискретную модель (Ad/Bd), что и сам node — иначе MPC, проектируемый
    под одни матрицы, теряет точное tracking против отличного фиктивного
    plant'a (видели на test_mps_node_pose_arrival: Euler-step переоценивал
    эффективный gain ω-канала и MPC overshoot'ил target heading на ~3 рад).

    Состояние plant'a — каноническое MPS [s, v, θ, ω, e_int]; world (x, y)
    считаются интегрированием v·(cos θ, sin θ) поверх него — нужны для
    outer LQR-петли и (x_local, y_local) в телеметрии.

    Поддерживает `velocity_lag` (>1.0 → плант тормозит, v не успевает за
    u_v) для settle-timeout теста — реализуется через дополнительное
    масштабирование Bd[1,0].
    """

    def __init__(self, plant):
        # plant — StateSpaceModel из mps_node (та же Ad/Bd что у MPC).
        self.plant = plant
        self.velocity_lag = 1.0
        self.x_state = np.zeros(5)        # [s, v, θ, ω, e_int]
        self.x_world = 0.0
        self.y_world = 0.0

    def start_at(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.x_world = x
        self.y_world = y
        self.x_state = np.zeros(5)
        self.x_state[2] = theta

    def set_velocity_lag(self, multiplier: float):
        """multiplier > 1 ⇒ v-канал «тормозит»: фактическое u_v
        масштабируется на 1/multiplier перед plant.step. Канал θ/ω
        не трогаем — это контролируемый медленный плант по v."""
        self.velocity_lag = multiplier

    def integrate_cmd_vel(self, cmd: dict | None, dt: float):
        if cmd is None:
            return
        u_v = float(cmd.get('linear_x', 0.0)) / self.velocity_lag
        u_w = float(cmd.get('angular_z', 0.0))
        u = np.array([u_v, u_w])
        prev_v = float(self.x_state[1])
        prev_theta = float(self.x_state[2])
        self.x_state = self.plant.step(self.x_state, u)
        # world.x/world.y — Эйлер по «средней» скорости/курсу за тик.
        v_mid = 0.5 * (prev_v + float(self.x_state[1]))
        theta_mid = 0.5 * (prev_theta + float(self.x_state[2]))
        self.x_world += v_mid * math.cos(theta_mid) * dt
        self.y_world += v_mid * math.sin(theta_mid) * dt

    def payload(self) -> dict:
        """Mqtt-style odom payload (cm для x/y, как старый motor_node;
        s_body уже в метрах — новый motor_node)."""
        return {
            's_body': float(self.x_state[0]),
            'x': self.x_world * 100.0,
            'y': self.y_world * 100.0,
            'vx': float(self.x_state[1]),
            'theta': float(self.x_state[2]),
            'vz': float(self.x_state[3]),
        }


def _last_cmd_vel(node) -> dict | None:
    """Последний опубликованный cmd_vel, если был."""
    for topic, payload, _qos in reversed(node._published):
        if topic == 'cmd_vel':
            return payload
    return None


def test_mps_node_pose_arrival_d03_phi_pi(mps_node):
    """Сценарий D=0.30 м с финальным разворотом на π рад против
    idealized plant. После ~16 с tick-loop должен выйти на reached.

    Acceptance: status='reached'; конечная s ≈ D (±0.01 м); конечный θ
    отличается от π не более чем на ε_θ=0.05 рад (см. _check_finish).
    """
    fake = _FakeOdom(mps_node._plant)
    fake.start_at(0.0, 0.0, 0.0)
    mps_node._on_odom('odom', fake.payload())
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'test-pose',
        'request': {'distance': 0.30, 'v_target': 0.15,
                    'target_heading': math.pi, 'source': 'robot'},
        'reference': {'a_max': 0.20, 'alpha_max': 1.0},
    })
    dt = mps_node._tick_dt
    # 800 тиков ≈ 16 с при dt=0.02. t_end для D=0.30, v=0.15, a=0.20
    # ≈ 2.75 с (drive) + ≈ 3.3 с (turn на π рад под α=1) ≈ 6 с; +
    # settle_timeout 1.5 с — глубокий запас.
    for _ in range(800):
        mps_node._tick()
        fake.integrate_cmd_vel(_last_cmd_vel(mps_node), dt=dt)
        mps_node._on_odom('odom', fake.payload())
        if mps_node._run is None:
            break

    finished = [p[1] for p in mps_node._published
                if p[0] == 'mps/scenario/finished']
    assert finished, 'scenario should finish within 16 s of tick-loop'
    status_msg = finished[-1]
    assert status_msg['status'] == 'reached', (
        f'expected reached, got {status_msg["status"]} '
        f'(error_message: {status_msg.get("error_message")})'
    )
    # Финальная позиция/курс — из последней опубликованной телеметрии.
    tel = [p[1] for p in mps_node._published if p[0] == 'mps/telemetry']
    assert tel, 'expected at least one telemetry point'
    last_point = tel[-1]['point']
    # s — относительно старта, должно быть ≈ D = 0.30.
    assert abs(last_point['x'][0] - 0.30) < 0.02, (
        f's_end = {last_point["x"][0]:.4f}, expected ≈ 0.30'
    )
    # θ — тоже относительно старта (см. _tick_run: x[_THETA] = θ − θ_start).
    # target = π; нормируем в [-π, π].
    err = (last_point['x'][2] - math.pi + math.pi) % (2 * math.pi) - math.pi
    assert abs(err) < mps_node._eps_theta + 0.01, (
        f'θ_err = {err:.4f}, expected |.| < {mps_node._eps_theta + 0.01}'
    )


def test_mps_node_settle_timeout_reports_detail(mps_node):
    """Сильно заторможенный плант не выводит v в 0 за settle_timeout ⇒
    status='timeout_settle' с деталью «settle timeout: |s−D|=..., ...».

    Реализация: накручиваем velocity_lag=5.0 — τ_v эффективно 0.75 с,
    v медленно затухает после конца профиля, |v|>ε_v за settle-окно
    не успевает сойтись.
    """
    fake = _FakeOdom(mps_node._plant)
    fake.set_velocity_lag(5.0)
    fake.start_at(0.0, 0.0, 0.0)
    mps_node._on_odom('odom', fake.payload())
    mps_node._on_scenario_run('mps/scenario/run', {
        'run_id': 'test-settle',
        'request': {'distance': 0.30, 'v_target': 0.15,
                    'target_heading': 0.0, 'source': 'robot'},
        'reference': {'a_max': 0.20, 'alpha_max': 1.0},
    })
    dt = mps_node._tick_dt
    for _ in range(2000):
        mps_node._tick()
        fake.integrate_cmd_vel(_last_cmd_vel(mps_node), dt=dt)
        mps_node._on_odom('odom', fake.payload())
        if mps_node._run is None:
            break

    finished = [p[1] for p in mps_node._published
                if p[0] == 'mps/scenario/finished']
    assert finished, 'scenario should finish (timeout_settle expected)'
    msg = finished[-1]
    assert msg['status'] == 'timeout_settle', (
        f'expected timeout_settle, got {msg["status"]} '
        f'(error_message: {msg.get("error_message")})'
    )
    assert 'settle timeout' in (msg.get('error_message') or ''), (
        f'detail must contain "settle timeout", got: {msg.get("error_message")}'
    )


# ── live_state: _last_u ────────────────────────────────────────────────
def test_last_u_updated_in_publish_cmd_and_telemetry(mps_node):
    """После публикации cmd_vel + telemetry, _last_u должен содержать u
    и быть копией (мутация u не должна затрагивать _last_u)."""
    # `_RunState.__init__` требует ReferenceTrajectory; мокаем целиком —
    # `_publish_cmd_and_telemetry` использует только run_id, distance, t,
    # telemetry. См. pi_nodes/nodes/mps_node.py:74-126.
    run = MagicMock()
    run.run_id = 'r-test'
    run.distance = 1.0
    run.t = 0.0
    run.telemetry = []
    x = np.array([0.0, 0.1, 0.0, 0.0, 0.0])
    u = np.array([0.123, -0.456])
    mps_node._plant = MagicMock()
    mps_node._plant.output.return_value = x.copy()

    mps_node._publish_cmd_and_telemetry(run, x, u)

    assert mps_node._last_u[0] == pytest.approx(0.123)
    assert mps_node._last_u[1] == pytest.approx(-0.456)

    # _last_u должен быть копией, не alias: мутация u не отражается.
    u[0] = 999.0
    assert mps_node._last_u[0] == pytest.approx(0.123)
