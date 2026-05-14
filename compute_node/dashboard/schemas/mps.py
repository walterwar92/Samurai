"""
MPS — Модель Пространства Состояний (state-space) — Pydantic схемы.

Используются:
  • REST роутером compute_node/dashboard/routers/mps.py;
  • mqtt_handlers.py (для MQTT-телеметрии mps/telemetry, mps/scenario/finished);
  • compute_node/mps_runner.py (идеальный sim).

TS-зеркало: compute_node/frontend/src/types/mps.ts (строго синхронизировано
по структуре, schema_version совпадает).

Глоссарий см. docs/mps/api.md §1; полная спека —
docs/superpowers/specs/2026-05-05-mps-state-space-design.md.
"""
from __future__ import annotations

from datetime import datetime
from typing import Literal, Optional

from pydantic import BaseModel, Field, field_validator, model_validator

from .common import OkResponse


# ── Версия схемы (bump при breaking changes контракта) ─────────────────
MPS_SCHEMA_VERSION = '1.0'

# ── Размерности (фиксированы для курсовой Козлова) ─────────────────────
N_STATES = 5            # x = [s, v, θ, ω, e_int]
N_CONTROLS = 2          # u = [v_cmd, ω_cmd]


# ── Базовые типы ───────────────────────────────────────────────────────
class ComplexNumber(BaseModel):
    """JSON-сериализуемое комплексное число (для eigenvalues)."""
    re: float
    im: float

    @classmethod
    def from_complex(cls, z: complex) -> 'ComplexNumber':
        return cls(re=float(z.real), im=float(z.imag))


class MpsMatrices(BaseModel):
    """Матрицы пространства состояний + параметры регулятора.

    Порядок состояний:  x = [s, v, θ, ω, e_int]
    Порядок управлений: u = [v_cmd, ω_cmd]

    A, B — НЕПРЕРЫВНЫЕ матрицы A_c, B_c канонической ОДУ-модели. Бэкенд
    (mps_runner / mps_node) ZOH-дискретизирует их при mps.plant.Ts перед
    передачей в дискретный MPCController. См. docs/mps/api.md §2.

    C, D хранятся для документации курсовой; в `step()` контроллера не
    используются. В UI — только для y(t) визуализации (`StateSpaceModel.output`).
    """
    A: list[list[float]] = Field(
        ...,
        description='5×5 — непрерывная матрица состояния A_c (бэкенд ZOH-дискретизирует)'
    )
    B: list[list[float]] = Field(
        ...,
        description='5×2 — непрерывная матрица управления B_c (бэкенд ZOH-дискретизирует)'
    )
    C: list[list[float]] = Field(
        ...,
        description='5×5 — матрица выхода (default I_5)'
    )
    D: list[list[float]] = Field(
        ...,
        description='5×2 — матрица прямой связи (default 0)'
    )
    Q_diag: list[float] = Field(
        ...,
        description='Диагональ Q (5) — веса состояния'
    )
    R_diag: list[float] = Field(
        ...,
        description='Диагональ R (2) — веса управления'
    )
    horizon_N: int = Field(
        ...,
        ge=1,
        le=200,
        description='Горизонт MPC, шагов'
    )
    u_min: list[float] = Field(
        ...,
        description='Нижние ограничения u (2)'
    )
    u_max: list[float] = Field(
        ...,
        description='Верхние ограничения u (2)'
    )
    schema_version: str = MPS_SCHEMA_VERSION

    @field_validator('A')
    @classmethod
    def _check_a(cls, v: list[list[float]]) -> list[list[float]]:
        if len(v) != N_STATES or any(len(row) != N_STATES for row in v):
            raise ValueError(f'A must be {N_STATES}x{N_STATES}')
        return v

    @field_validator('B')
    @classmethod
    def _check_b(cls, v: list[list[float]]) -> list[list[float]]:
        if len(v) != N_STATES or any(len(row) != N_CONTROLS for row in v):
            raise ValueError(f'B must be {N_STATES}x{N_CONTROLS}')
        return v

    @field_validator('C')
    @classmethod
    def _check_c(cls, v: list[list[float]]) -> list[list[float]]:
        # k×n, k=n по умолчанию. Допускаем любой k>=1, но колонок ровно n.
        if not v or any(len(row) != N_STATES for row in v):
            raise ValueError(f'C must be k×{N_STATES}, k>=1')
        return v

    @field_validator('D')
    @classmethod
    def _check_d(cls, v: list[list[float]]) -> list[list[float]]:
        if not v or any(len(row) != N_CONTROLS for row in v):
            raise ValueError(f'D must be k×{N_CONTROLS}, k>=1')
        return v

    @field_validator('Q_diag')
    @classmethod
    def _check_q(cls, v: list[float]) -> list[float]:
        if len(v) != N_STATES:
            raise ValueError(f'Q_diag must have {N_STATES} elements')
        if any(x < 0 for x in v):
            raise ValueError('Q_diag must be non-negative')
        return v

    @field_validator('R_diag')
    @classmethod
    def _check_r(cls, v: list[float]) -> list[float]:
        if len(v) != N_CONTROLS:
            raise ValueError(f'R_diag must have {N_CONTROLS} elements')
        if any(x <= 0 for x in v):
            raise ValueError('R_diag must be strictly positive')
        return v

    @field_validator('u_min', 'u_max')
    @classmethod
    def _check_u_bounds(cls, v: list[float]) -> list[float]:
        if len(v) != N_CONTROLS:
            raise ValueError(f'u bounds must have {N_CONTROLS} elements')
        return v

    @model_validator(mode='after')
    def _check_u_min_lt_max(self) -> 'MpsMatrices':
        if any(lo >= hi for lo, hi in zip(self.u_min, self.u_max)):
            raise ValueError('u_min must be strictly less than u_max element-wise')
        # Согласованность размерностей C/D (одно k).
        if len(self.C) != len(self.D):
            raise ValueError('C and D must share the same number of rows (output dim k)')
        return self


# ── Сценарий ───────────────────────────────────────────────────────────
ScenarioSource = Literal['sim', 'robot']


class MpsScenarioRequest(BaseModel):
    """POST /api/v1/mps/scenario/run."""
    distance: float = Field(
        ...,
        gt=0,
        le=5.0,
        description='D — дистанция в метрах (safety cap 5.0)'
    )
    v_target: float = Field(
        ...,
        gt=0,
        le=0.30,
        description='Целевая продольная скорость (м/с, cap 0.30)'
    )
    source: ScenarioSource = Field(
        ...,
        description='sim — на ноуте, robot — реальный Pi через MQTT'
    )
    schema_version: str = MPS_SCHEMA_VERSION


# ── Телеметрия ─────────────────────────────────────────────────────────
class MpsTelemetryPoint(BaseModel):
    """Одна точка телеметрии прогона. Публикуется 50 Гц (sim — собирается
    в массив; robot — стримится через MQTT/WS)."""
    t: float = Field(..., description='Секунды от старта')
    x: list[float] = Field(..., description='Состояние, длина 5')
    u: list[float] = Field(..., description='Управление, длина 2')
    y: list[float] = Field(..., description='y = Cx + Du, длина k (default 5)')
    s_remaining: float = Field(..., description='D − s, метры')


# ── Метрики ────────────────────────────────────────────────────────────
class MpsMetrics(BaseModel):
    overshoot: float = Field(..., description='max(s) − D, м (≥0; <0 если не дошёл)')
    settling_time: float = Field(..., description='Первое t где |s−D|<0.02 устойчиво, с')
    control_energy: float = Field(..., description='Σ uᵀRu·dt')
    ss_error: float = Field(..., description='|s(t_end) − D|, м')
    peak_v: float = Field(..., description='max|v|, м/с')
    peak_omega: float = Field(..., description='max|ω|, рад/с')


# ── Результат прогона ──────────────────────────────────────────────────
ScenarioStatus = Literal['running', 'reached', 'timeout', 'aborted', 'error']


class MpsScenarioResult(BaseModel):
    run_id: str
    started_at: datetime
    finished_at: Optional[datetime] = None
    status: ScenarioStatus
    request: MpsScenarioRequest
    matrices_snapshot: MpsMatrices
    telemetry: list[MpsTelemetryPoint] = Field(default_factory=list)
    metrics: Optional[MpsMetrics] = None
    error_message: Optional[str] = None
    schema_version: str = MPS_SCHEMA_VERSION


# ── Validate ───────────────────────────────────────────────────────────
class MpsValidateResult(BaseModel):
    """Ответ на POST /api/v1/mps/validate.

    Считается:
      • λ(Ad) — устойчивость объекта;
      • λ(Ad − Bd·K_first) — устойчивость замкнутой системы;
      • короткий 2-сек step-response в идеальном симуляторе.
    """
    eigenvalues_ad: list[ComplexNumber]
    eigenvalues_closed: list[ComplexNumber]
    is_plant_stable: bool
    is_closed_loop_stable: bool
    step_response: list[MpsTelemetryPoint] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)
    schema_version: str = MPS_SCHEMA_VERSION


# ── REST envelopes ─────────────────────────────────────────────────────
class MpsMatricesGetResponse(OkResponse):
    applied: MpsMatrices
    draft: Optional[MpsMatrices] = None


class MpsMatricesSetResponse(OkResponse):
    status: Literal['draft', 'applied'] = 'draft'
    matrices: MpsMatrices


class MpsScenarioRunResponse(OkResponse):
    """Sync (sim): result отдаётся сразу. Async (robot): только run_id."""
    run_id: str
    result: Optional[MpsScenarioResult] = None


class MpsScenarioAbortResponse(OkResponse):
    aborted: bool = True
    run_id: Optional[str] = None


class MpsHistoryResponse(OkResponse):
    history: list[MpsScenarioResult] = Field(default_factory=list)


class MpsConfigSaveResponse(OkResponse):
    written: bool = True
    path: str


# ── MQTT payload типы (используются mqtt_handlers) ─────────────────────
class MpsMqttScenarioRun(BaseModel):
    """compute → Pi: запуск сценария на реальном роботе."""
    run_id: str
    request: MpsScenarioRequest
    schema_version: str = MPS_SCHEMA_VERSION


class MpsMqttApplied(BaseModel):
    """Pi → compute: ack применения матриц."""
    matrices: MpsMatrices
    applied_at: datetime
    schema_version: str = MPS_SCHEMA_VERSION


class MpsMqttError(BaseModel):
    """Pi → compute: ошибка во время прогона (NaN, instability, watchdog)."""
    run_id: Optional[str] = None
    error_type: Literal['nan', 'instability', 'watchdog', 'precondition', 'other']
    message: str
    schema_version: str = MPS_SCHEMA_VERSION


class MpsMqttTelemetry(BaseModel):
    """Pi → compute: одна точка телеметрии (QoS 0)."""
    run_id: str
    point: MpsTelemetryPoint
    schema_version: str = MPS_SCHEMA_VERSION
