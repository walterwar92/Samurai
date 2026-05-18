// MPS — Модель Пространства Состояний — TS-зеркало Pydantic-схем.
// Источник правды: compute_node/dashboard/schemas/mps.py
// Контракт зафиксирован в docs/mps/api.md.
//
// Размерности: x ∈ ℝ^5, u ∈ ℝ^2. C/D могут быть k×n с произвольным k≥1,
// но по умолчанию k=n=5 (Cd=I, Dd=0).

// 1.1: добавлены опциональные e_y/theta_err/delta_theta в MpsTelemetryPoint
// (outer LQR-петля коррекции бокового сноса). Backwards-compatible.
// 1.2: добавлены опциональные r/x_local/y_local в MpsTelemetryPoint;
// статус 'timeout_settle'; семантика target_heading = final heading
// (pose-tracking refactor 2026-05-17).
export const MPS_SCHEMA_VERSION = '1.2' as const

export const N_STATES = 5
export const N_CONTROLS = 2

export interface ComplexNumber {
  re: number
  im: number
}

export interface MpsMatrices {
  /** 5×5 — дискретная матрица состояния */
  A: number[][]
  /** 5×2 — дискретная матрица управления */
  B: number[][]
  /** k×5 — матрица выхода (default I_5) */
  C: number[][]
  /** k×2 — матрица прямой связи (default 0) */
  D: number[][]
  /** Диагональ Q (5) — веса состояния */
  Q_diag: number[]
  /** Диагональ R (2) — веса управления */
  R_diag: number[]
  /** Горизонт MPC, шагов */
  horizon_N: number
  /** Нижние ограничения u (2) */
  u_min: number[]
  /** Верхние ограничения u (2) */
  u_max: number[]
  schema_version: string
}

export type ScenarioSource = 'sim' | 'robot'

export interface MpsScenarioRequest {
  /** D — дистанция в метрах (0 < D ≤ 5.0) */
  distance: number
  /** Целевая продольная скорость (0 < v ≤ 0.30) */
  v_target: number
  source: ScenarioSource
  /** Финальный курс φ (рад, −π…π) после прибытия в (D, 0) локального
   *  фрейма старта. 0.0 = не разворачивается; π = разворот на 180°
   *  после доезда. Используется только при source='robot'. */
  target_heading?: number
  schema_version?: string
}

export interface MpsTelemetryPoint {
  /** Секунды от старта */
  t: number
  /** Состояние, длина 5 */
  x: number[]
  /** Управление, длина 2 */
  u: number[]
  /** y = C·x + D·u, длина k (default 5) */
  y: number[]
  /** D − s, метры */
  s_remaining: number
  /** Латеральная ошибка от ideal-line (м). Заполнено только на роботе в
   *  фазе DRIVE с включённой outer LQR-петлёй; в sim или с
   *  lateral.enabled=false — undefined. */
  e_y?: number
  /** Ошибка курса θ − φ (рад). Тот же контекст, что и e_y. */
  theta_err?: number
  /** Коррекция курсовой ссылки δθ_ref = -K_lat·[e_y,θ_err], клипнута до
   *  ±delta_theta_max (рад). Тот же контекст, что и e_y. */
  delta_theta?: number
  /** Опорный 5-вектор r(t) = [s_ref, v_ref, θ_ref, ω_ref, e_int_ref] на
   *  этом тике (pose-tracking, schema 1.2). null/undefined для старой
   *  телеметрии (pre-2026-05-17). */
  r?: number[]
  /** Позиция робота (м) в локальном фрейме старта; X-ось локального
   *  фрейма направлена вдоль курса робота на момент _on_scenario_run.
   *  null/undefined для старой телеметрии. */
  x_local?: number
  /** Позиция робота (м) в локальном фрейме старта; Y-ось локального
   *  фрейма — налево от X (правая система координат). null/undefined
   *  для старой телеметрии. */
  y_local?: number
}

export interface MpsMetrics {
  /** max(s) − D, м (≥0; <0 если не дошёл) */
  overshoot: number
  /** Первое t где |s−D|<0.02 устойчиво, с */
  settling_time: number
  /** Σ uᵀRu·dt */
  control_energy: number
  /** |s(t_end) − D|, м */
  ss_error: number
  /** max|v|, м/с */
  peak_v: number
  /** max|ω|, рад/с */
  peak_omega: number
}

export type ScenarioStatus = 'running' | 'reached' | 'timeout' | 'timeout_settle' | 'aborted' | 'error'

export interface MpsScenarioResult {
  run_id: string
  /** ISO timestamp */
  started_at: string
  finished_at: string | null
  status: ScenarioStatus
  request: MpsScenarioRequest
  matrices_snapshot: MpsMatrices
  telemetry: MpsTelemetryPoint[]
  metrics: MpsMetrics | null
  error_message?: string | null
  schema_version: string
}

export interface MpsValidateResult {
  eigenvalues_ad: ComplexNumber[]
  eigenvalues_closed: ComplexNumber[]
  is_plant_stable: boolean
  is_closed_loop_stable: boolean
  step_response: MpsTelemetryPoint[]
  warnings: string[]
  schema_version: string
}

// ── REST envelopes ────────────────────────────────────────────────────
export interface MpsMatricesGetResponse {
  ok: true
  applied: MpsMatrices
  draft: MpsMatrices | null
}

export interface MpsMatricesSetResponse {
  ok: true
  status: 'draft' | 'applied'
  matrices: MpsMatrices
}

export interface MpsScenarioRunResponse {
  ok: true
  run_id: string
  /** Заполнено только когда source='sim' (sync). Для 'robot' — null, дождись WebSocket. */
  result: MpsScenarioResult | null
}

export interface MpsScenarioAbortResponse {
  ok: true
  aborted: boolean
  run_id: string | null
}

export interface MpsHistoryResponse {
  ok: true
  history: MpsScenarioResult[]
}

export interface MpsConfigSaveResponse {
  ok: true
  written: boolean
  path: string
}

// ── WebSocket /ws/mps/telemetry ───────────────────────────────────────
export interface MpsWsSubscribe {
  action: 'subscribe'
  run_id?: string
}

export interface MpsWsTelemetryFrame {
  type: 'telemetry'
  run_id: string
  point: MpsTelemetryPoint
}

export interface MpsWsFinishedFrame {
  type: 'finished'
  run_id: string
  result: MpsScenarioResult
}

export interface MpsWsErrorFrame {
  type: 'error'
  run_id?: string | null
  error_type: 'nan' | 'instability' | 'watchdog' | 'precondition' | 'other'
  message: string
}

export type MpsWsFrame =
  | MpsWsTelemetryFrame
  | MpsWsFinishedFrame
  | MpsWsErrorFrame

// ── /ws/mps/live_state ────────────────────────────────────────────────
// Постоянный поток вектора состояния x ∈ ℝ⁵ и управления u ∈ ℝ²
// (10 Hz), независимо от прогона. Контракт: docs/superpowers/specs/
// 2026-05-18-mps-live-state-vector-design.md §2.
export const MPS_LIVE_STATE_SCHEMA = '1.0' as const

export interface MpsLiveStatePoint {
  /** Pi-clock unix seconds */
  ts: number
  /** Состояние [s, v, θ, ω, e_int]; длина 5 */
  x: number[]
  /** Управление [v_cmd, ω_cmd]; длина 2 */
  u: number[]
  /** true когда mps_node в DRIVE_FORWARD_MPS */
  scenario_active: boolean
  /** UUID активного прогона, если scenario_active=true */
  run_id: string | null
  schema_version: string
}

export interface MpsLiveStateWsFrame {
  type: 'live_state'
  point: MpsLiveStatePoint
}
