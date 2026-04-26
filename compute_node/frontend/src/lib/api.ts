/**
 * Legacy API wrapper — fetch к /api/* (без /v1/).
 *
 * DEPRECATED. Backend (#7, 2026-04) ввёл /api/v1/ + автогенерированный
 * TypeScript-клиент в `src/api/`. Старые `/api/*` URL'ы продолжают
 * работать через deprecated-alias middleware (HTTP header
 * `Deprecation: true; sunset="2026-12-31"`), но новые компоненты
 * должны использовать сервисы:
 *
 *   import { RobotService, MapsService } from '@/api'
 *   await RobotService.setVelocityApiV1RobotVelocityPost({ requestBody: { linear, angular } })
 *
 * Эта обёртка будет удалена в #6 (Zustand миграция фронта).
 */
const BASE = ''

const DEFAULT_TIMEOUT_MS = 5000
const DEFAULT_RETRIES = 2          // total attempts = 1 + DEFAULT_RETRIES
const RETRY_BACKOFF_MS = [200, 500, 1200]
// Methods safe to retry without risking duplicate side effects.
const IDEMPOTENT_METHODS = new Set(['GET', 'HEAD', 'PUT', 'DELETE', 'OPTIONS'])

interface RequestOpts {
  method?: string
  body?: unknown
  timeoutMs?: number
  retries?: number
  // Override the safe-to-retry default. Use sparingly: only when the caller
  // knows the POST is idempotent on the server (e.g. setLed, setSpeedProfile).
  retryOn5xx?: boolean
  signal?: AbortSignal
  cache?: RequestCache
}

/**
 * Single fetch wrapper with timeout + retry. Used by every helper below.
 *
 * Retry policy:
 *  - Network errors (fetch throws) and 5xx responses are retried.
 *  - 4xx responses are NOT retried — the server explicitly rejected the
 *    request, retrying won't help.
 *  - POST is retried only when retryOn5xx=true (caller asserts idempotency).
 *  - Backoff: 200ms, 500ms, 1200ms.
 *  - Each attempt has its own timeout via AbortController.
 */
async function request(url: string, opts: RequestOpts = {}): Promise<Response> {
  const method = (opts.method ?? 'GET').toUpperCase()
  const retries = opts.retries ?? DEFAULT_RETRIES
  const timeoutMs = opts.timeoutMs ?? DEFAULT_TIMEOUT_MS
  const isIdempotent = IDEMPOTENT_METHODS.has(method) || opts.retryOn5xx === true
  const maxAttempts = isIdempotent ? retries + 1 : 1

  const headers: Record<string, string> = {}
  let body: BodyInit | undefined
  if (opts.body !== undefined) {
    headers['Content-Type'] = 'application/json'
    body = JSON.stringify(opts.body)
  }

  let lastError: unknown
  for (let attempt = 0; attempt < maxAttempts; attempt++) {
    const ctrl = new AbortController()
    const timer = setTimeout(() => ctrl.abort(), timeoutMs)

    // Daisy-chain caller's external signal so they can cancel this attempt.
    const externalAbort = () => ctrl.abort()
    opts.signal?.addEventListener('abort', externalAbort, { once: true })

    try {
      const res = await fetch(BASE + url, {
        method,
        headers,
        body,
        cache: opts.cache,
        signal: ctrl.signal,
      })

      // 5xx + idempotent → retry. 4xx → return as-is, caller decides.
      if (res.status >= 500 && res.status < 600 && attempt < maxAttempts - 1) {
        await sleep(RETRY_BACKOFF_MS[Math.min(attempt, RETRY_BACKOFF_MS.length - 1)])
        continue
      }
      return res
    } catch (err) {
      lastError = err
      // Caller cancelled — don't swallow into a retry.
      if (opts.signal?.aborted) throw err
      if (attempt >= maxAttempts - 1) break
      await sleep(RETRY_BACKOFF_MS[Math.min(attempt, RETRY_BACKOFF_MS.length - 1)])
    } finally {
      clearTimeout(timer)
      opts.signal?.removeEventListener('abort', externalAbort)
    }
  }
  throw lastError ?? new Error('request failed')
}

function sleep(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms))
}

async function post(url: string, body?: object) {
  return request(url, { method: 'POST', body })
}

/** POST that parses the JSON body — use when you need the response payload. */
async function postJson(url: string, body?: object) {
  const res = await request(url, { method: 'POST', body })
  try {
    return await res.json()
  } catch {
    return { ok: res.ok }
  }
}

async function get(url: string) {
  // GET is idempotent → retried automatically by request()
  const res = await request(url, { method: 'GET', cache: 'no-store' })
  return res.json()
}

async function del(url: string) {
  return request(url, { method: 'DELETE' })
}

export const api = {
  createZone: (x1: number, y1: number, x2: number, y2: number) =>
    post('/api/zones', { x1, y1, x2, y2 }),

  deleteZone: (id: number | string) =>
    del(`/api/zones/${id}`),

  clearZones: () =>
    post('/api/zones/clear'),

  forceTransition: (state: string) =>
    post('/api/fsm/transition', { state }),

  emergencyStop: () =>
    post('/api/robot/stop'),

  sendVelocity: (linear: number, angular: number) =>
    post('/api/robot/velocity', { linear, angular }),

  setClaw: (open: boolean) =>
    post('/api/actuators/claw', { open }),

  // Head (single servo camera)
  setHeadAngle: (angle: number) =>
    post('/api/actuators/head', { angle }),

  centerHead: () =>
    post('/api/actuators/head', { command: 'center' }),

  headCommand: (command: string) =>
    post('/api/actuators/head', { command }),

  headSavePreset: (name: string) =>
    post('/api/actuators/head', { command: 'save_preset', name }),

  headLoadPreset: (name: string) =>
    post('/api/actuators/head', { command: 'load_preset', name }),

  headDeletePreset: (name: string) =>
    post('/api/actuators/head', { command: 'delete_preset', name }),

  headListPresets: () =>
    get('/api/actuators/head/presets'),

  // Arm (4 joints)
  setArmJoint: (joint: number, angle: number) =>
    post('/api/actuators/arm', { joint, angle }),

  setArmAll: (joints: number[]) =>
    post('/api/actuators/arm', { joints }),

  homeArm: () =>
    post('/api/actuators/arm', { command: 'home' }),

  armCommand: (command: string, extra?: object) =>
    post('/api/actuators/arm', { command, ...extra }),

  armFreezeJoint: (joint: number) =>
    post('/api/actuators/arm', { command: 'freeze', joint }),

  armUnfreezeJoint: (joint: number) =>
    post('/api/actuators/arm', { command: 'unfreeze', joint }),

  armSavePreset: (name: string) =>
    post('/api/actuators/arm', { command: 'save_preset', name }),

  armLoadPreset: (name: string) =>
    post('/api/actuators/arm', { command: 'load_preset', name }),

  armDeletePreset: (name: string) =>
    post('/api/actuators/arm', { command: 'delete_preset', name }),

  armListPresets: () =>
    get('/api/actuators/arm/presets'),

  // Speed profiles
  setSpeedProfile: (profile: string) =>
    post('/api/speed_profile', { profile }),

  // Patrol
  setPatrolWaypoints: (waypoints: { x: number; y: number; yaw?: number }[]) =>
    post('/api/patrol/waypoints', { waypoints }),

  patrolCommand: (command: string) =>
    post('/api/patrol/command', { command }),

  // Map management
  saveMap: (name: string) =>
    post('/api/map/save', { name }),

  loadMap: (name: string) =>
    post('/api/map/load', { name }),

  listMaps: () =>
    get('/api/map/list'),

  // Follow me
  followMeCommand: (command: string) =>
    post('/api/follow_me', { command }),

  // Position reset
  resetPosition: () =>
    post('/api/robot/reset_position'),

  // Path recorder
  pathRecorderCommand: (command: string) =>
    post('/api/path_recorder/command', { command }),

  listPaths: () =>
    get('/api/path_recorder/list'),

  // Detection toggle
  setDetectionEnabled: (enabled: boolean) =>
    post('/api/detection/toggle', { enabled }),

  // Obstacle avoidance toggle
  setObstacleAvoidance: (enabled: boolean) =>
    post('/api/obstacle_avoidance/toggle', { enabled }),

  // Collision guard toggle
  setCollisionGuard: (enabled: boolean) =>
    post('/api/collision_guard/toggle', { enabled }),

  // Precision drive
  precisionDriveCommand: (body: object) =>
    post('/api/precision_drive/command', body),

  // LED
  setLed: (mode: string) =>
    post('/api/led/command', { mode }),

  // Calibration coefficients & profiles
  setCalibration: (scale_fwd: number, scale_bwd: number, motor_trim: number) =>
    post('/api/calibration/set', { scale_fwd, scale_bwd, motor_trim }),

  loadCalibrationProfile: (name: string) =>
    post('/api/calibration/profile/load', { name }),

  saveCalibrationProfile: (name: string, description?: string) =>
    post('/api/calibration/profile/save', { name, description: description ?? '' }),

  deleteCalibrationProfile: (name: string) =>
    post('/api/calibration/profile/delete', { name }),

  listCalibrationProfiles: () =>
    get('/api/calibration/profile/list'),

  // Hardware presets
  listHardwarePresets: () =>
    get('/api/hardware/presets'),

  getHardwarePreset: (name: string) =>
    get(`/api/hardware/presets/${encodeURIComponent(name)}`),

  saveHardwarePreset: (preset: object) =>
    postJson('/api/hardware/presets', preset),

  deleteHardwarePreset: (name: string) =>
    del(`/api/hardware/presets/${encodeURIComponent(name)}`),

  applyHardwarePreset: (name: string) =>
    post('/api/hardware/apply', { name }),

  getActiveHardwarePreset: () =>
    get('/api/hardware/active'),
}
