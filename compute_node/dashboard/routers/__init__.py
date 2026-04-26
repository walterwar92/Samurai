"""
FastAPI routers по доменам. Каждый router монтируется в app.py через include_router.

Структура (создаётся incrementally в C5-C10):
  - robot.py     — pose, velocity, stop, reset
  - sensors.py   — sensors, ultrasonic, imu, battery, temperature
  - actuators.py — claw, head, arm, led
  - detection.py — yolo detections, balls, qr
  - fsm.py       — fsm/command, fsm/transition
  - maps.py      — slam_map, zones, save/load
  - control.py   — patrol, follow_me, path_recorder, precision, calibration, mission
  - system.py    — status, logs, hardware presets, multi-robot
  - camera.py    — H.264 endpoint, /ws/h264 proxy
  - samcan.py    — proxy на :5005

Все routers получают prefix `/api/v1/` от app.py (старые без префикса —
deprecated alias через C11).
"""
