"""
Pydantic schemas for the MQTT payloads consumed by Pi-side nodes (#43).

Compute side (#7) already has a full schema layer; until now the Pi side
parsed every payload manually with isinstance/dict.get/float() chains
sprinkled across ~20 subscribe callbacks. Each callback re-implemented its
own validation, error handling was inconsistent, and a malformed publisher
could silently coerce values into wrong types.

These schemas are *opt-in*: existing nodes keep working unchanged, but new
code (and gradually migrated old code) can do:

    from pi_nodes.schemas import CmdVel, parse_payload

    def _cmd_vel_cb(self, topic, data):
        cmd = parse_payload(data, CmdVel)
        if cmd is None:
            self.log_warn('Bad cmd_vel payload: %s', data)
            return
        self._linear = cmd.linear_x
        self._angular = cmd.angular_z

`parse_payload` returns None on validation failure rather than raising —
keeps the Pi nodes' "drop-bad-message-and-keep-going" semantics. Nodes that
prefer hard failures can call CmdVel.model_validate(data) directly.
"""
from __future__ import annotations

from typing import Any, Optional

try:
    from pydantic import BaseModel, ConfigDict, Field, ValidationError
    _HAS_PYDANTIC = True
except ImportError:
    _HAS_PYDANTIC = False
    BaseModel = object  # type: ignore[assignment,misc]
    ValidationError = Exception  # type: ignore[assignment,misc]


def parse_payload(data: Any, model_cls: type[BaseModel]) -> Optional[BaseModel]:
    """Validate and convert raw MQTT payload into a typed model.

    Returns None if pydantic is unavailable, the payload isn't dict-shaped,
    or validation fails. Caller should log and skip on None.
    """
    if not _HAS_PYDANTIC or not isinstance(data, dict):
        return None
    try:
        return model_cls.model_validate(data)
    except ValidationError:
        return None


# ── Common base ─────────────────────────────────────────────────────────────
if _HAS_PYDANTIC:
    class _BaseMqttSchema(BaseModel):
        # extra='ignore' lets producers add fields (e.g. _cid for correlation
        # tracking) without breaking consumers using the schema.
        model_config = ConfigDict(extra='ignore')

    # ── Velocity / command ──────────────────────────────────────────────────
    class CmdVel(_BaseMqttSchema):
        """samurai/{id}/cmd_vel — autonomous velocity command."""
        linear_x: float = 0.0
        angular_z: float = 0.0

    # ── Sensors ─────────────────────────────────────────────────────────────
    class Range(_BaseMqttSchema):
        """samurai/{id}/range — HC-SR04 ultrasonic."""
        range: float = Field(ge=0.0, le=10.0)
        ts: float = 0.0
        age_s: float = -1.0

    class Imu(_BaseMqttSchema):
        """samurai/{id}/imu — partial schema. Real payload has many fields;
        this catches the ones consumers most commonly read."""
        ax: float = 0.0
        ay: float = 0.0
        az: float = 9.81
        gx: float = 0.0
        gy: float = 0.0
        gz: float = 0.0
        calibrated: bool = False
        ts: float = 0.0

    class Odom(_BaseMqttSchema):
        """samurai/{id}/odom — published from motor_node every 50ms.
        Note: x, y are CENTIMETRES (legacy; consumers often divide by 100).

        x/y is the *primary* fused position; the source is named in `source`.
        Per-source x_wheel/x_imu are diagnostic — published every tick so the
        dashboard can plot all estimators side-by-side and compare error.
        """
        x: float = 0.0
        y: float = 0.0
        theta: float = 0.0
        vx: float = 0.0
        vz: float = 0.0
        speed: float = 0.0
        accel_x: float = 0.0
        accel_y: float = 0.0
        stationary: bool = True
        ts: float = 0.0
        # ── Diagnostic per-source positions (centimetres, m/s) ──
        x_wheel: float = 0.0
        y_wheel: float = 0.0
        x_imu: float = 0.0
        y_imu: float = 0.0
        vx_imu: float = 0.0
        vy_imu: float = 0.0
        stationary_imu: bool = True
        source: str = 'wheel'

    class Battery(_BaseMqttSchema):
        """samurai/{id}/battery — voltage + percent."""
        voltage: float = Field(ge=0.0, le=30.0)
        percent: float = Field(default=0.0, ge=0.0, le=100.0)

    # ── Calibration ─────────────────────────────────────────────────────────
    class CalibrationSet(_BaseMqttSchema):
        """samurai/{id}/calibration/set — wheel scale + motor trim."""
        scale_fwd: Optional[float] = Field(default=None, ge=0.5, le=3.0)
        scale_bwd: Optional[float] = Field(default=None, ge=0.5, le=3.0)
        motor_trim: Optional[float] = Field(default=None, ge=-50.0, le=50.0)

    # ── Path recorder commands ──────────────────────────────────────────────
    class PathRecorderCommand(_BaseMqttSchema):
        """samurai/{id}/path_recorder/command — record/stop/replay/save/etc."""
        command: str = Field(min_length=1, max_length=32)
        # Path filename — already validated downstream by path_recorder_node
        # via a regex (#17), but bound length here too for defence-in-depth.
        name: Optional[str] = Field(default=None, max_length=64)

    # ── FSM / commands ──────────────────────────────────────────────────────
    class FsmCommand(_BaseMqttSchema):
        """samurai/{id}/fsm_cmd — FSM control."""
        command: str = Field(min_length=1, max_length=32)
        target: Optional[str] = None

    # Public interface (only export if pydantic is available)
    __all__ = [
        'parse_payload',
        'CmdVel', 'Range', 'Imu', 'Odom', 'Battery',
        'CalibrationSet', 'PathRecorderCommand', 'FsmCommand',
    ]
else:
    __all__ = ['parse_payload']
