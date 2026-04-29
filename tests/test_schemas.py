"""Unit tests for pi_nodes.schemas (#43)."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pydantic = pytest.importorskip('pydantic')

from pi_nodes.schemas import (
    CmdVel, Range, Imu, Odom, Battery,
    CalibrationSet, PathRecorderCommand, FsmCommand,
    parse_payload,
)


def test_cmd_vel_defaults():
    c = CmdVel()
    assert c.linear_x == 0.0
    assert c.angular_z == 0.0


def test_cmd_vel_parses_full_payload():
    c = parse_payload({'linear_x': 0.3, 'angular_z': -0.5}, CmdVel)
    assert c is not None
    assert c.linear_x == pytest.approx(0.3)
    assert c.angular_z == pytest.approx(-0.5)


def test_cmd_vel_ignores_extra_fields():
    """`_cid` for correlation tracking + future extensions must pass through."""
    c = parse_payload({'linear_x': 0.1, '_cid': 'abc12345', 'extra': 999}, CmdVel)
    assert c is not None
    assert c.linear_x == pytest.approx(0.1)


def test_parse_payload_returns_none_for_non_dict():
    assert parse_payload('not a dict', CmdVel) is None
    assert parse_payload(None, CmdVel) is None
    assert parse_payload([1, 2, 3], CmdVel) is None
    assert parse_payload(42, CmdVel) is None


def test_parse_payload_returns_none_for_invalid_types():
    """A string where a number is expected → None, not an exception."""
    assert parse_payload({'linear_x': 'fast'}, CmdVel) is None


def test_range_range_validation():
    assert parse_payload({'range': 0.5}, Range) is not None
    # Out of range → reject
    assert parse_payload({'range': -1.0}, Range) is None
    assert parse_payload({'range': 100.0}, Range) is None


def test_imu_calibrated_is_bool():
    imu = parse_payload({'ax': 0, 'ay': 0, 'az': 9.81, 'calibrated': True}, Imu)
    assert imu is not None
    assert imu.calibrated is True
    assert imu.az == pytest.approx(9.81)


def test_battery_validation():
    assert parse_payload({'voltage': 7.4, 'percent': 60}, Battery) is not None
    # Negative voltage → reject
    assert parse_payload({'voltage': -1, 'percent': 50}, Battery) is None
    # >100% → reject
    assert parse_payload({'voltage': 7.4, 'percent': 150}, Battery) is None


def test_calibration_partial_update():
    """Calibration commands often update one coefficient; others stay None."""
    c = parse_payload({'scale_fwd': 1.2}, CalibrationSet)
    assert c is not None
    assert c.scale_fwd == pytest.approx(1.2)
    assert c.scale_bwd is None
    assert c.motor_trim is None


def test_calibration_range_clamping():
    """Out-of-band scale → rejected so a typo can't brick the robot."""
    assert parse_payload({'scale_fwd': 100.0}, CalibrationSet) is None
    assert parse_payload({'motor_trim': 99.0}, CalibrationSet) is None


def test_path_recorder_command_required_field():
    assert parse_payload({}, PathRecorderCommand) is None
    cmd = parse_payload({'command': 'replay', 'name': 'home'}, PathRecorderCommand)
    assert cmd is not None
    assert cmd.command == 'replay'
    assert cmd.name == 'home'


def test_path_recorder_command_max_length():
    assert parse_payload({'command': 'x' * 33}, PathRecorderCommand) is None


def test_odom_full_payload():
    odom = parse_payload({
        'x': 12.3, 'y': -4.5, 'theta': 1.57,
        'vx': 0.2, 'vz': 0.1, 'speed': 0.2,
        'accel_x': 0.0, 'accel_y': 0.0,
        'stationary': False, 'ts': 1234567890.123,
    }, Odom)
    assert odom is not None
    assert odom.x == pytest.approx(12.3)
    assert odom.stationary is False


def test_fsm_command_required():
    assert parse_payload({}, FsmCommand) is None
    cmd = parse_payload({'command': 'stop'}, FsmCommand)
    assert cmd is not None
    assert cmd.command == 'stop'
