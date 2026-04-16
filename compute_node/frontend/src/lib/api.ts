const BASE = ''

async function post(url: string, body?: object) {
  const res = await fetch(BASE + url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  })
  return res
}

/** POST that parses the JSON body — use when you need the response payload. */
async function postJson(url: string, body?: object) {
  const res = await fetch(BASE + url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  })
  try {
    return await res.json()
  } catch {
    return { ok: res.ok }
  }
}

async function get(url: string) {
  const res = await fetch(BASE + url, { cache: 'no-store' })
  return res.json()
}

async function del(url: string) {
  const res = await fetch(BASE + url, { method: 'DELETE' })
  return res
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
