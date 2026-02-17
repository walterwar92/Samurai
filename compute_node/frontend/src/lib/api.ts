const BASE = ''

async function post(url: string, body?: object) {
  const res = await fetch(BASE + url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  })
  return res
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

  setLaser: (on: boolean) =>
    post('/api/actuators/laser', { on }),
}
