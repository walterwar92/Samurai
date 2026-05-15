"""Tests for pi_nodes.nodes.imu_node — фокус на imu.invert_yaw mounting flag.

Контекст: MPU-6050 на Adeept HAT V3.1 смонтирован осью Z вниз, поэтому raw gz
с регистра приходит с обратным знаком относительно ROS-конвенции
(positive ω = CCW при Z-up). Флаг `imu.invert_yaw: true` инвертирует gz сразу
после _read_raw — до EMA, калибровки и EKF, чтобы все downstream (EKF yaw,
motor_node odom.theta, mps_node MPC heading-loop) работали в согласованной
системе координат и контур курса оставался negative-feedback'ом.

Стратегия: моки paho-mqtt и таймеров (как в test_mps_node.py); _read_raw
подменяется в тесте чтобы скормить ноде известное «чтение чипа», после чего
проверяем, что попало в публикуемый payload.
"""
from __future__ import annotations

import os
import sys
from unittest.mock import MagicMock, patch

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


@pytest.fixture
def imu_node_factory():
    """Фабрика IMUNode с замоканной MQTT-связью, таймерами и I2C.

    Возвращает функцию `_factory(invert_yaw=...)` — каждый тест передаёт
    свой config-флаг до инстанциации (cfg() патчится на время __init__).
    """
    def _factory(invert_yaw: bool = False):
        from pi_nodes.nodes import imu_node as imu_node_module

        def fake_cfg(key, default=None):
            if key == 'imu.invert_yaw':
                return invert_yaw
            return default

        with patch('pi_nodes.mqtt_node.mqtt.Client') as MockClient:
            MockClient.return_value = MagicMock()
            with patch.object(imu_node_module.IMUNode, 'create_timer',
                              lambda self, period, cb: None):
                with patch.object(imu_node_module.IMUNode, 'subscribe',
                                  lambda *a, **kw: None):
                    with patch.object(imu_node_module.IMUNode, '_init_i2c_bus',
                                      lambda self: True):
                        with patch.object(imu_node_module, 'cfg', fake_cfg):
                            node = imu_node_module.IMUNode()
        # Пропускаем фазу калибровки: сценарий "EKF уже работает" — нулевой bias.
        node._calibrated = True
        node._gyro_offset = (0.0, 0.0, 0.0)

        node._published: list[tuple[str, dict]] = []

        def _capture(suffix, payload, qos=0, retain=False):
            node._published.append((suffix, payload))

        node.publish = _capture  # type: ignore[assignment]
        return node

    return _factory


def test_invert_yaw_false_passes_gz_unchanged(imu_node_factory, monkeypatch):
    """invert_yaw=False (дефолт): raw gz приходит в payload как есть.

    Это контракт по умолчанию — старые роботы / ноды без mounting-фикса
    не должны менять поведение от добавления флага.
    """
    node = imu_node_factory(invert_yaw=False)
    # Чип «крутится» в своей локальной системе — gz=+0.5 рад/с.
    monkeypatch.setattr(node, '_read_raw',
                        lambda: (0.0, 0.0, 9.81, 0.0, 0.0, 0.5))
    node._read_and_publish()
    pub = next(p for p in node._published if p[0] == 'imu')
    assert pub[1]['gz'] == pytest.approx(0.5, abs=1e-5)


def test_invert_yaw_true_negates_gz(imu_node_factory, monkeypatch):
    """invert_yaw=True (чип Z-вниз): raw gz инвертируется в payload.

    Конкретно этот случай — Adeept HAT V3.1, где MPU-6050 смонтирован
    «вверх ногами»; raw gz по правилу правой руки даёт обратный знак.
    После инверсии EKF интегрирует gz в нужную сторону, motor_node
    odom.theta растёт при физическом CCW, mps_node MPC heading-loop
    становится negative-feedback'ом.
    """
    node = imu_node_factory(invert_yaw=True)
    # Raw gz=+0.5 с чипа Z-вниз — это физическое CW вращение робота.
    # После инверсии payload должен показать -0.5 (CW по ROS Z-up).
    monkeypatch.setattr(node, '_read_raw',
                        lambda: (0.0, 0.0, 9.81, 0.0, 0.0, 0.5))
    node._read_and_publish()
    pub = next(p for p in node._published if p[0] == 'imu')
    assert pub[1]['gz'] == pytest.approx(-0.5, abs=1e-5)


def test_invert_yaw_true_propagates_to_ekf_yaw(imu_node_factory, monkeypatch):
    """invert_yaw=True: знак gz после инверсии попадает в EKF.

    Защита от регрессии: фикс должен применяться ДО _ekf.predict(), иначе
    интеграл yaw продолжит расти в обратную сторону и MPS-баг вернётся.
    """
    node = imu_node_factory(invert_yaw=True)
    if node._ekf is None:
        pytest.skip('EKF не доступен (нет numpy/ekf_imu) — тест неприменим')

    # Сбросим EKF в детерминированное состояние, чтобы изолировать вклад gz.
    node._ekf.reset()
    monkeypatch.setattr(node, '_read_raw',
                        lambda: (0.0, 0.0, 9.81, 0.0, 0.0, 0.5))
    yaw_before = node._ekf.yaw
    node._read_and_publish()
    yaw_after = node._ekf.yaw
    # Без инверсии yaw_after был бы > yaw_before (gz=+0.5 интегрируется).
    # С инверсией gz уходит в EKF как -0.5 → yaw_after < yaw_before.
    assert yaw_after < yaw_before, (
        f'EKF yaw должен уменьшаться после инверсии gz: '
        f'before={yaw_before}, after={yaw_after}'
    )
