# Samurai Simulator — REST API Reference

## Overview

Samurai Simulator provides a complete REST API for controlling the robot, reading sensors, managing the FSM state machine, and querying arena state. Any developer can integrate their code via standard HTTP requests.

**Base URL:** `http://localhost:5000`

**Start the simulator:**
```bash
cd Samurai
python compute_node/simulator.py
```

---

## Response Format

All JSON endpoints return a consistent envelope:

```json
// Success
{"ok": true, "data": { ... }}

// Error
{"ok": false, "error": "Human-readable error message"}
```

**POST requests** must send JSON body with `Content-Type: application/json`.

**Image endpoints** (`/api/camera/frame`, `/api/map/image`) return raw binary data (JPEG/PNG) directly, not wrapped in JSON.

### HTTP Status Codes

| Code | Meaning |
|------|---------|
| 200 | OK |
| 400 | Bad request (invalid parameters) |
| 404 | Endpoint not found |
| 405 | Method not allowed |
| 503 | Resource not ready (no frame/map yet) |

---

## Quick Start

### 1. Get system status
```bash
curl http://localhost:5000/api/status
```

### 2. Send a command (find red ball)
```bash
curl -X POST http://localhost:5000/api/fsm/command \
  -H "Content-Type: application/json" \
  -d "{\"text\": \"найди красный мяч\"}"
```

### 3. Read ultrasonic sensor
```bash
curl http://localhost:5000/api/sensors/ultrasonic
```

---

## Endpoints

### 1. System Status

#### `GET /api/status`

Full system snapshot — robot, sensors, FSM, arena in one call.

**Response:**
```json
{
  "ok": true,
  "data": {
    "sim_time": 12.35,
    "sim_dt": 0.05,
    "sim_hz": 20,
    "robot": {
      "x": 1.500, "y": 1.500, "theta": 0.0,
      "v_linear": 0.0, "v_angular": 0.0,
      "claw_open": false, "laser_on": false
    },
    "fsm": {
      "state": "IDLE",
      "target_colour": "",
      "target_action": ""
    },
    "sensors": {
      "ultrasonic_m": 1.234,
      "imu": {
        "yaw": 0.0, "pitch": 0.1, "roll": -0.1,
        "gyro_z": 0.0, "accel_x": 0.0
      }
    },
    "arena": {
      "width": 3.0, "height": 3.0,
      "balls_total": 5, "balls_remaining": 4
    },
    "detections_count": 2
  }
}
```

---

### 2. Robot Control

#### `GET /api/robot/pose`

Current robot position and heading.

**Response:**
```json
{
  "ok": true,
  "data": {
    "x": 1.500,
    "y": 1.500,
    "theta": 0.785,
    "theta_deg": 45.0
  }
}
```

| Field | Type | Description |
|-------|------|-------------|
| x | float | X position in metres |
| y | float | Y position in metres |
| theta | float | Heading in radians |
| theta_deg | float | Heading in degrees |

---

#### `GET /api/robot/velocity`

Current velocity values and limits.

**Response:**
```json
{
  "ok": true,
  "data": {
    "linear": 0.12,
    "angular": -0.4,
    "max_linear": 0.3,
    "max_angular": 2.0
  }
}
```

---

#### `POST /api/robot/velocity`

Set robot velocity directly (equivalent to ROS2 `/cmd_vel`).

**Request body:**
```json
{
  "linear": 0.15,
  "angular": 0.5
}
```

Both fields are optional. Omitted fields keep their current value. Values are clamped to `[-max, max]`.

| Parameter | Type | Range | Description |
|-----------|------|-------|-------------|
| linear | float | [-0.3, 0.3] | Forward/backward speed (m/s) |
| angular | float | [-2.0, 2.0] | Rotation speed (rad/s, positive = counter-clockwise) |

**Response:**
```json
{
  "ok": true,
  "data": {"linear": 0.15, "angular": 0.5}
}
```

**Example:**
```bash
# Drive forward at 0.1 m/s
curl -X POST http://localhost:5000/api/robot/velocity \
  -H "Content-Type: application/json" \
  -d "{\"linear\": 0.1, \"angular\": 0.0}"
```

---

#### `POST /api/robot/stop`

Emergency stop — sets both velocities to zero.

**Request body:** None needed (empty or `{}`).

**Response:**
```json
{"ok": true, "data": {"linear": 0.0, "angular": 0.0}}
```

---

#### `POST /api/robot/reset`

Full simulation reset: robot returns to center, all balls respawn, FSM goes to IDLE.

**Request body:** None needed.

**Response:**
```json
{"ok": true, "data": {"message": "Simulation reset"}}
```

---

### 3. Sensors

#### `GET /api/sensors`

All sensor readings in one call.

**Response:**
```json
{
  "ok": true,
  "data": {
    "ultrasonic": {
      "range_m": 1.234,
      "min_range": 0.02,
      "max_range": 2.0,
      "field_of_view_rad": 0.26
    },
    "imu": {
      "yaw_deg": 45.3,
      "pitch_deg": 0.1,
      "roll_deg": -0.2,
      "gyro_z_rad_s": 0.5,
      "accel_x_m_s2": 0.01
    }
  }
}
```

---

#### `GET /api/sensors/ultrasonic`

Ultrasonic distance sensor only.

**Response:**
```json
{
  "ok": true,
  "data": {
    "range_m": 1.234,
    "min_range": 0.02,
    "max_range": 2.0,
    "field_of_view_rad": 0.26
  }
}
```

| Field | Type | Description |
|-------|------|-------------|
| range_m | float | Distance to nearest obstacle (metres) |
| min_range | float | Minimum detectable distance |
| max_range | float | Maximum detectable distance |
| field_of_view_rad | float | Sensor cone half-angle (radians) |

---

#### `GET /api/sensors/imu`

IMU (Inertial Measurement Unit) data.

**Response:**
```json
{
  "ok": true,
  "data": {
    "yaw_deg": 45.3,
    "pitch_deg": 0.1,
    "roll_deg": -0.2,
    "gyro_z_rad_s": 0.5,
    "accel_x_m_s2": 0.01
  }
}
```

| Field | Type | Description |
|-------|------|-------------|
| yaw_deg | float | Heading angle (degrees) |
| pitch_deg | float | Pitch angle (degrees) |
| roll_deg | float | Roll angle (degrees) |
| gyro_z_rad_s | float | Angular velocity around Z (rad/s) |
| accel_x_m_s2 | float | Forward acceleration (m/s^2) |

---

### 4. Ball Detection

#### `GET /api/detection`

All balls currently visible in the camera.

**Response:**
```json
{
  "ok": true,
  "data": {
    "count": 2,
    "detections": [
      {
        "colour": "red",
        "class": "sports ball",
        "x": 280, "y": 200, "w": 40, "h": 40,
        "conf": 0.87,
        "distance": 0.95
      },
      {
        "colour": "blue",
        "class": "sports ball",
        "x": 120, "y": 210, "w": 30, "h": 30,
        "conf": 0.72,
        "distance": 1.45
      }
    ]
  }
}
```

| Field | Type | Description |
|-------|------|-------------|
| colour | string | Ball colour (red, blue, green, yellow, orange) |
| class | string | Object class (always "sports ball") |
| x, y | int | Bounding box top-left corner (pixels, 640x480) |
| w, h | int | Bounding box size (pixels) |
| conf | float | Detection confidence (0.0 - 1.0) |
| distance | float | Estimated distance to ball (metres) |

---

#### `GET /api/detection/closest`

Closest visible ball, optionally filtered by colour.

**Query parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| colour | string | "" | Filter by colour (e.g., "red", "blue"). Empty = any colour. |

**Examples:**
```bash
# Closest ball of any colour
curl http://localhost:5000/api/detection/closest

# Closest red ball
curl http://localhost:5000/api/detection/closest?colour=red
```

**Response (found):**
```json
{
  "ok": true,
  "data": {
    "found": true,
    "detection": {
      "colour": "red",
      "class": "sports ball",
      "x": 280, "y": 200, "w": 40, "h": 40,
      "conf": 0.87,
      "distance": 0.95
    }
  }
}
```

**Response (not found):**
```json
{"ok": true, "data": {"found": false, "detection": null}}
```

---

### 5. Actuators

#### `GET /api/actuators`

Current state of claw and laser.

**Response:**
```json
{
  "ok": true,
  "data": {
    "claw_open": false,
    "laser_on": false
  }
}
```

---

#### `POST /api/actuators/claw`

Open or close the claw.

**Request body:**
```json
{"open": true}
```

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| open | boolean | Yes | `true` = open claw, `false` = close claw |

**Response:**
```json
{"ok": true, "data": {"claw_open": true}}
```

---

#### `POST /api/actuators/laser`

Turn laser on or off.

**Request body:**
```json
{"on": true}
```

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| on | boolean | Yes | `true` = laser on, `false` = laser off |

**Response:**
```json
{"ok": true, "data": {"laser_on": true}}
```

---

### 6. FSM (Finite State Machine)

#### `GET /api/fsm`

Current FSM state, target, and action.

**Response:**
```json
{
  "ok": true,
  "data": {
    "state": "SEARCHING",
    "target_colour": "red",
    "target_action": "grab",
    "all_states": ["IDLE","SEARCHING","TARGETING","APPROACHING","GRABBING","BURNING","CALLING","RETURNING"]
  }
}
```

---

#### `GET /api/fsm/states`

List of all valid FSM states.

**Response:**
```json
{
  "ok": true,
  "data": {
    "states": ["IDLE","SEARCHING","TARGETING","APPROACHING","GRABBING","BURNING","CALLING","RETURNING"],
    "current": "IDLE"
  }
}
```

#### FSM State Diagram

```
IDLE ──(command)──> SEARCHING ──(ball found)──> TARGETING ──(centered)──> APPROACHING
  ^                     |                          |                         |
  |                     |                          |                         |
  |              (360° no ball)             (lost > 1.5s)              (range < 0.1m)
  |                     |                          |                     /       \
  |                     v                          v                    v         v
  |                   IDLE                    SEARCHING             GRABBING   BURNING
  |                                                                    |         |
  |                                                                    v         v
  +<────────────────────────────────────────────────────────────── RETURNING  SEARCHING
```

---

#### `POST /api/fsm/command`

Send a voice command (text) to the FSM. This is the primary way to control the robot.

**Request body:**
```json
{"text": "найди красный мяч"}
```

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| text | string | Yes | Command text (Russian) |

**Response:**
```json
{
  "ok": true,
  "data": {
    "command": "найди красный мяч",
    "new_state": "SEARCHING",
    "target_colour": "red",
    "target_action": "grab"
  }
}
```

**Available commands:**

| Command (Russian) | Action | FSM Transition |
|---|---|---|
| `найди [цвет] мяч` | Find and grab ball | IDLE -> SEARCHING |
| `получи [цвет] мяч` | Find and grab ball | IDLE -> SEARCHING |
| `возьми [цвет] мяч` | Find and grab ball | IDLE -> SEARCHING |
| `сожги [цвет] мяч` | Find and burn ball with laser | IDLE -> SEARCHING (action=burn) |
| `прожги [цвет] мяч` | Find and burn ball with laser | IDLE -> SEARCHING (action=burn) |
| `лазер [цвет]` | Burn with laser | IDLE -> SEARCHING (action=burn) |
| `стоп` | Emergency stop | Any -> IDLE |
| `остановись` | Emergency stop | Any -> IDLE |
| `домой` | Return to home position | Any -> RETURNING |
| `вернись` | Return to home position | Any -> RETURNING |
| `вызови вторую машину` | Call second robot | Any -> CALLING -> IDLE |

**Colour words (Russian -> English):**

| Russian | English |
|---------|---------|
| красный / красн | red |
| синий / синего | blue |
| зелёный / зелен | green |
| жёлтый / желт | yellow |
| оранжевый / оранж | orange |
| белый / бел | white |
| чёрный / черн | black |

---

#### `POST /api/fsm/transition`

Force a direct FSM state transition (for debugging/testing).

**Request body:**
```json
{"state": "IDLE"}
```

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| state | string | Yes | Target state (must be one of the valid states) |

**Response:**
```json
{
  "ok": true,
  "data": {
    "previous_state": "SEARCHING",
    "new_state": "IDLE"
  }
}
```

---

### 7. Arena

#### `GET /api/arena`

Arena dimensions and ball summary.

**Response:**
```json
{
  "ok": true,
  "data": {
    "width": 3.0,
    "height": 3.0,
    "robot_radius": 0.12,
    "ball_radius": 0.02,
    "balls_total": 5,
    "balls_remaining": 4,
    "balls_grabbed": 1
  }
}
```

---

#### `GET /api/arena/balls`

All balls with positions and grabbed status.

**Response:**
```json
{
  "ok": true,
  "data": {
    "balls": [
      {"id": 0, "colour": "red",    "x": 0.800, "y": 0.600, "radius": 0.02, "grabbed": false},
      {"id": 1, "colour": "blue",   "x": 2.200, "y": 0.800, "radius": 0.02, "grabbed": false},
      {"id": 2, "colour": "green",  "x": 1.500, "y": 2.000, "radius": 0.02, "grabbed": true},
      {"id": 3, "colour": "yellow", "x": 0.500, "y": 2.300, "radius": 0.02, "grabbed": false},
      {"id": 4, "colour": "orange", "x": 2.500, "y": 1.800, "radius": 0.02, "grabbed": false}
    ]
  }
}
```

---

### 8. Map and Camera

#### `GET /api/map/image`

Top-down arena map as PNG image (binary response, not JSON).

**Response:** Raw PNG bytes, `Content-Type: image/png`

**Example:**
```bash
curl http://localhost:5000/api/map/image -o map.png
```

---

#### `GET /api/map/info`

Map metadata (JSON).

**Response:**
```json
{
  "ok": true,
  "data": {
    "width_px": 300,
    "height_px": 300,
    "resolution_m_per_px": 0.01,
    "origin_x": 0.0,
    "origin_y": 0.0,
    "arena_width_m": 3.0,
    "arena_height_m": 3.0
  }
}
```

---

#### `GET /api/camera/frame`

Latest camera frame as JPEG image (binary response, not JSON).

**Response:** Raw JPEG bytes, `Content-Type: image/jpeg`

**Example:**
```bash
curl http://localhost:5000/api/camera/frame -o frame.jpg
```

---

#### `GET /api/camera/frame.json`

Camera frame encoded as base64 in JSON (for clients that can't handle binary).

**Response:**
```json
{
  "ok": true,
  "data": {
    "format": "jpeg",
    "width": 640,
    "height": 480,
    "base64": "/9j/4AAQSkZJRg..."
  }
}
```

---

### 9. Event Log

#### `GET /api/log`

FSM event and transition log.

**Query parameters:**

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| limit | int | 20 | 1-50 | Number of recent entries to return |

**Example:**
```bash
curl http://localhost:5000/api/log?limit=10
```

**Response:**
```json
{
  "ok": true,
  "data": {
    "count": 5,
    "entries": [
      {"time": "14:32:01", "text": "Команда: \"найди красный мяч\""},
      {"time": "14:32:01", "text": "Цель: захватить red мяч"},
      {"time": "14:32:01", "text": "FSM: IDLE -> SEARCHING"},
      {"time": "14:32:03", "text": "Обнаружен red мяч — наведение"},
      {"time": "14:32:03", "text": "FSM: SEARCHING -> TARGETING"}
    ]
  }
}
```

---

## WebSocket API (SocketIO)

The simulator also provides real-time updates via Socket.IO (used by the built-in dashboard).

**Connect:**
```javascript
const socket = io('http://localhost:5000');
```

### Events: Server -> Client

#### `state_update`

Emitted every 250ms (4 Hz) with full system state.

```javascript
socket.on('state_update', (data) => {
  // data.status    — {state, target_colour, target_action, range_m}
  // data.detection — closest ball detection or {}
  // data.range_m   — ultrasonic distance
  // data.imu_ypr   — [yaw, pitch, roll]
  // data.pose      — {x, y, yaw}
  // data.map_info  — {width, height, resolution, origin_x, origin_y}
  // data.voice_log — [{text, time}, ...]
});
```

### Events: Client -> Server

#### `send_command`

Send a voice command.

```javascript
socket.emit('send_command', { text: 'найди красный мяч' });
```

#### `reset_sim`

Reset the simulation.

```javascript
socket.emit('reset_sim', {});
```

### Streaming: MJPEG

The camera feed is also available as an MJPEG stream:

```
GET /video_feed
```

Can be embedded directly in an `<img>` tag:
```html
<img src="http://localhost:5000/video_feed" />
```

---

## Integration Examples

### Python (requests)

```python
import requests

BASE = 'http://localhost:5000'

# Get system status
r = requests.get(f'{BASE}/api/status')
status = r.json()['data']
print(f"Robot at ({status['robot']['x']}, {status['robot']['y']})")
print(f"FSM state: {status['fsm']['state']}")

# Send command to find red ball
r = requests.post(f'{BASE}/api/fsm/command', json={
    'text': 'найди красный мяч'
})
result = r.json()['data']
print(f"New state: {result['new_state']}")

# Read ultrasonic sensor
r = requests.get(f'{BASE}/api/sensors/ultrasonic')
dist = r.json()['data']['range_m']
print(f"Distance: {dist}m")

# Drive forward manually
requests.post(f'{BASE}/api/robot/velocity', json={
    'linear': 0.1, 'angular': 0.0
})

# Stop
requests.post(f'{BASE}/api/robot/stop')

# Open claw
requests.post(f'{BASE}/api/actuators/claw', json={'open': True})

# Turn on laser
requests.post(f'{BASE}/api/actuators/laser', json={'on': True})

# Get all ball positions
r = requests.get(f'{BASE}/api/arena/balls')
for ball in r.json()['data']['balls']:
    status = 'grabbed' if ball['grabbed'] else 'available'
    print(f"  {ball['colour']} ball at ({ball['x']}, {ball['y']}) — {status}")

# Save camera frame
r = requests.get(f'{BASE}/api/camera/frame')
with open('frame.jpg', 'wb') as f:
    f.write(r.content)

# Save map
r = requests.get(f'{BASE}/api/map/image')
with open('map.png', 'wb') as f:
    f.write(r.content)
```

### Python — Autonomous Control Loop

```python
import requests
import time

BASE = 'http://localhost:5000'

def get(path):
    return requests.get(f'{BASE}{path}').json()['data']

def post(path, data=None):
    return requests.post(f'{BASE}{path}', json=data or {}).json()['data']

# Custom autonomous behaviour: find and grab all balls
colours = ['red', 'blue', 'green', 'yellow', 'orange']

for colour in colours:
    # Command: find ball
    post('/api/fsm/command', {'text': f'найди {colour} мяч'})

    # Wait for FSM to finish (return to IDLE)
    while True:
        state = get('/api/fsm')['state']
        if state == 'IDLE':
            break
        time.sleep(0.5)

    # Check how many balls remain
    arena = get('/api/arena')
    print(f"Balls remaining: {arena['balls_remaining']}")

print("All balls collected!")
```

### JavaScript (fetch)

```javascript
const BASE = 'http://localhost:5000';

// Get status
async function getStatus() {
  const res = await fetch(`${BASE}/api/status`);
  const json = await res.json();
  if (json.ok) {
    console.log('Robot:', json.data.robot);
    console.log('FSM:', json.data.fsm);
  }
}

// Send command
async function sendCommand(text) {
  const res = await fetch(`${BASE}/api/fsm/command`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ text })
  });
  const json = await res.json();
  console.log('New state:', json.data.new_state);
}

// Set velocity
async function drive(linear, angular) {
  await fetch(`${BASE}/api/robot/velocity`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ linear, angular })
  });
}

// Usage
await sendCommand('найди красный мяч');
```

### curl

```bash
# Status
curl -s http://localhost:5000/api/status | python -m json.tool

# Find red ball
curl -s -X POST http://localhost:5000/api/fsm/command \
  -H "Content-Type: application/json" \
  -d '{"text": "найди красный мяч"}' | python -m json.tool

# Drive forward
curl -s -X POST http://localhost:5000/api/robot/velocity \
  -H "Content-Type: application/json" \
  -d '{"linear": 0.1, "angular": 0.0}' | python -m json.tool

# Stop
curl -s -X POST http://localhost:5000/api/robot/stop | python -m json.tool

# Reset simulation
curl -s -X POST http://localhost:5000/api/robot/reset | python -m json.tool

# Get all detections
curl -s http://localhost:5000/api/detection | python -m json.tool

# Get arena balls
curl -s http://localhost:5000/api/arena/balls | python -m json.tool

# Open claw
curl -s -X POST http://localhost:5000/api/actuators/claw \
  -H "Content-Type: application/json" \
  -d '{"open": true}' | python -m json.tool

# Laser on
curl -s -X POST http://localhost:5000/api/actuators/laser \
  -H "Content-Type: application/json" \
  -d '{"on": true}' | python -m json.tool

# Force FSM to IDLE
curl -s -X POST http://localhost:5000/api/fsm/transition \
  -H "Content-Type: application/json" \
  -d '{"state": "IDLE"}' | python -m json.tool

# Event log (last 10)
curl -s "http://localhost:5000/api/log?limit=10" | python -m json.tool

# Save camera frame
curl http://localhost:5000/api/camera/frame -o frame.jpg

# Save map
curl http://localhost:5000/api/map/image -o map.png
```

---

## Endpoint Summary

| # | Method | Path | Returns | Description |
|---|--------|------|---------|-------------|
| 1 | GET | `/api/status` | JSON | Full system snapshot |
| 2 | GET | `/api/robot/pose` | JSON | Robot x, y, theta |
| 3 | GET | `/api/robot/velocity` | JSON | Current velocities |
| 4 | POST | `/api/robot/velocity` | JSON | Set velocities (cmd_vel) |
| 5 | POST | `/api/robot/stop` | JSON | Emergency stop |
| 6 | POST | `/api/robot/reset` | JSON | Reset simulation |
| 7 | GET | `/api/sensors` | JSON | All sensor readings |
| 8 | GET | `/api/sensors/ultrasonic` | JSON | Ultrasonic range |
| 9 | GET | `/api/sensors/imu` | JSON | IMU orientation |
| 10 | GET | `/api/detection` | JSON | All visible detections |
| 11 | GET | `/api/detection/closest` | JSON | Closest ball (optional colour filter) |
| 12 | GET | `/api/actuators` | JSON | Claw and laser state |
| 13 | POST | `/api/actuators/claw` | JSON | Open/close claw |
| 14 | POST | `/api/actuators/laser` | JSON | Laser on/off |
| 15 | GET | `/api/fsm` | JSON | FSM state |
| 16 | GET | `/api/fsm/states` | JSON | List of valid states |
| 17 | POST | `/api/fsm/command` | JSON | Send voice command |
| 18 | POST | `/api/fsm/transition` | JSON | Force state transition |
| 19 | GET | `/api/arena` | JSON | Arena info |
| 20 | GET | `/api/arena/balls` | JSON | All balls with positions |
| 21 | GET | `/api/map/image` | PNG | Map as image |
| 22 | GET | `/api/map/info` | JSON | Map metadata |
| 23 | GET | `/api/camera/frame` | JPEG | Camera frame as image |
| 24 | GET | `/api/camera/frame.json` | JSON | Camera frame as base64 |
| 25 | GET | `/api/log` | JSON | Event log |

---

## Architecture

```
HTTP Client (Python/JS/curl/etc.)
        |
        v
  Flask REST API (/api/*)
        |
        v
  +-----+-----+-----+-----+-----+-----+
  |     |     |     |     |     |     |
SimRobot SimSensors SimDetector SimFSM SimArena MapRenderer
  |     |     |     |     |     |
  +-----+-----+-----+-----+-----+
        |
   Main Sim Loop (20 Hz)
        |
        v
  SocketIO (state_update @ 4 Hz)
        |
        v
  Dashboard (browser)
```

All API calls and the simulation loop share a single `threading.Lock()` for thread safety. API calls are fast (no blocking I/O) and do not affect simulation performance.
