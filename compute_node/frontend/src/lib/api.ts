const BASE = ''

async function post(url: string, body?: object) {
  const res = await fetch(BASE + url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  })
  return res
}

async function get(url: string) {
  const res = await fetch(BASE + url)
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

  // Arm (4 joints)
  setArmJoint: (joint: number, angle: number) =>
    post('/api/actuators/arm', { joint, angle }),

  setArmAll: (joints: number[]) =>
    post('/api/actuators/arm', { joints }),

  homeArm: () =>
    post('/api/actuators/arm', { command: 'home' }),

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
}
