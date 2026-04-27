# Samurai Robot

A two-tier autonomous tracked robot built around a Raspberry Pi 4 and an Adeept HAT V3.1, with optional support for a second robot (Samcan / Arduino Uno) over USB-Serial.

## Architecture at a glance

```
+----------------------+        MQTT 1883       +-------------------+
|   Raspberry Pi 4     |  <-------------------> |  Compute laptop   |
|   pi_nodes/  Python  |        H.264 TCP       |  Docker + ROS2    |
|   GPIO / I2C / cam   |  <-------------------> |  Nav2 / SLAM /    |
|                      |                        |  YOLO / Dashboard |
+----------------------+                        +-------------------+
        ^                                                ^
        | MQTT bridge                                    | REST + WS
        v                                                v
+----------------------+                        +-------------------+
| Samcan (Arduino Uno) |                        | React frontend    |
| USB-Serial           |                        | :5173 dev / SPA   |
+----------------------+                        +-------------------+
```

- **Pi side**: Pure Python + paho-mqtt (no ROS2). 22 nodes, hardware drivers, filters.
  Started via `./samurai.sh robot`.
- **Compute laptop**: Docker + ROS2 Humble + FastAPI + React. SLAM Toolbox, Nav2, YOLO.
  Started via `./samurai.sh compute`.
- **Android**: Kotlin + Compose, REST + SocketIO + MQTT, offline Vosk.
- **Firmware**: Samcan Arduino Uno (Serial) + ESP32 skeleton.

## Quick links

- [First-time setup](first-run.md) — installation walk-through for Pi + laptop
- [Architecture](architecture.md) — FSM states, MQTT topology, REST API
- [USB on Windows](usb-connect-windows.md) — Samcan COM-port troubleshooting

## Running locally

```bash
# Pi (over SSH):
./samurai.sh robot                    # default: Pure Python + MQTT
./samurai.sh status                   # what's running

# Laptop:
./samurai.sh compute                  # Docker + ROS2 stack
./samurai.sh sim                      # Flask simulator (no hardware)
./samurai.sh bridge                   # USB bridge for Samcan
```

See `./samurai.sh --help` for the full command set.

## Documentation generation

This site is built with [MkDocs Material](https://squidfunk.github.io/mkdocs-material/):

```bash
pip install mkdocs mkdocs-material
mkdocs serve   # http://localhost:8000
mkdocs build   # static site in ./site/
```

The deeper academic content (state estimation, EKF derivation, pure-pursuit math) lives in `latex_doc/` as a 15-chapter LaTeX report compiled to PDF. Browsable HTML versions are migrated chapter-by-chapter into this MkDocs site as effort allows.
