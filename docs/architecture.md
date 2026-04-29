# Architecture

Samurai uses a three-tier architecture: **Pi** (sensors / actuators) ↔ **Compute** (heavy CV / nav / dashboard) ↔ **Android / browser** (UI).

## Communication topology

| Tier | Tier | Protocol | Purpose |
|------|------|----------|---------|
| Pi   | Compute | MQTT (1883) | Telemetry, commands, FSM events |
| Pi   | Compute | TCP (8554)  | H.264 video stream from Pi camera |
| Compute | Browser | HTTP/WebSocket (5000) | Dashboard REST + SocketIO + WS |
| Compute | Samcan-Arduino | USB-Serial | Second robot bridge |
| Compute | Browser/Mobile | SSE (`/api/samcan/stream`) | Samcan telemetry push |

## MQTT topic prefix

All topics are namespaced as `samurai/{robot_id}/...` (default `robot_id=robot1`).

Selected high-rate topics:

| Topic | Rate | Direction | Schema |
|-------|------|-----------|--------|
| `odom`        | 20 Hz | Pi → All | `pi_nodes.schemas.Odom` |
| `imu`         | 50 Hz | Pi → All | `pi_nodes.schemas.Imu` |
| `range`       | 20 Hz | Pi → All | `pi_nodes.schemas.Range` (with `age_s`) |
| `cmd_vel`     | event | All → Pi | `pi_nodes.schemas.CmdVel` |
| `voice_command` | event | All → Pi | string |

## FSM states

Behaviour Tree (post-#1 transformation). See `pi_nodes/bt/` for tree definitions.

## Cross-references

- Detailed file maps: see `memory/files_pi.md`, `memory/files_compute.md`,
  `memory/frontend.md` in the repo's memory bundle.
- Per-improvement implementation notes: `memory/improvements_backlog.md`.
- LaTeX academic chapters: `latex_doc/chapters/01_velocity_ekf.tex` etc.
