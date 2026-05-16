// MPS — Модель Пространства Состояний — TS-зеркало Pydantic-схем.
// Источник правды: compute_node/dashboard/schemas/mps.py
// Контракт зафиксирован в docs/mps/api.md.
//
// Размерности: x ∈ ℝ^5, u ∈ ℝ^2. C/D могут быть k×n с произвольным k≥1,
// но по умолчанию k=n=5 (Cd=I, Dd=0).

// 1.1: добавлены опциональные e_y/theta_err/delta_theta в MpsTelemetryPoint
// (outer LQR-петля коррекции бокового сноса). Backwards-compatible.
export const MPS_SCHEMA_VERSION = '1.1' as const

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
  /** Относительный целевой курс (рад, −π…π) от курса на старте сценария.
   *  0 = ехать прямо. Используется только при source='robot'. */
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

export type ScenarioStatus = 'running' | 'reached' | 'timeout' | 'aborted' | 'error'

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
