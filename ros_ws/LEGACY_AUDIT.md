# ros_ws Legacy Audit (#71)

**Date:** 2026-04-26
**Context:** Pi-side moved from ROS2 to pure Python + MQTT (`pi_nodes/`)
in 2026-04. The laptop ROS2 compute stack (Nav2, SLAM Toolbox, EKF) is
still active and uses several nodes from `robot_pkg`.

This document classifies every `ros_ws/src/robot_pkg/robot_pkg/*.py` so
future cleanup PRs can delete the dead weight without breaking the
compute stack.

## TL;DR

- **13 files** are duplicates of `pi_nodes/nodes/` equivalents — Pi
  replaced them, ros_ws copies have not been used since 2026-04. **Safe
  to delete** in a follow-up PR after confirming nothing in
  `robot_bringup.launch.py` (already DEPRECATED) is being invoked
  outside development.
- **8 files** are still launched by `compute_bringup.launch.py` on the
  laptop. **Keep** until that launch file is rewritten or retired.
- **2 files** are reference-only / utility — recommendation per file.

## Per-file classification

### ✅ ACTIVE — laptop ROS2 stack

Referenced from `ros_ws/src/robot_pkg/launch/compute_bringup.launch.py`:

| File | Purpose |
|---|---|
| `depth_to_scan_node.py` | Camera depth → /scan for SLAM Toolbox |
| `patrol_node.py` | Waypoint patrol behaviour |
| `map_manager_node.py` | SLAM map save/load |
| `follow_me_node.py` | Person-follow behaviour |
| `path_recorder_node.py` | ROS2 path record/replay (laptop side) |
| `qr_detector_node.py` | QR code detection |
| `gesture_node.py` | Hand-gesture recognition |
| `mqtt_bridge_compute.py` | MQTT ↔ ROS2 translator (laptop side) |

⚠️ `mqtt_bridge_compute.py` exists in TWO locations:
   - `ros_ws/src/robot_pkg/robot_pkg/mqtt_bridge_compute.py` (ROS2 stack)
   - `compute_node/mqtt_bridge_compute.py` (recently modified for #24)

Need to check which one `compute_bringup.launch.py` actually invokes
when colcon-built — likely the ros_ws copy. The compute_node version
might be a leftover from a refactoring split. Action: confirm and unify.

### 🗑 LEGACY — replaced by pi_nodes/

Pi-side nodes moved to `pi_nodes/nodes/<name>.py` in the 2026-04
Pure-Python migration. ros_ws copies are no longer launched (only
`robot_bringup.launch.py` references them, and that launch file's own
header is marked DEPRECATED). Each pair below has a stub-vs-fleshed-out
size delta showing the pi_nodes equivalent has continued evolving:

| ros_ws file | LoC | pi_nodes/ equivalent LoC | Action |
|---|---|---|---|
| `battery_node.py` | 114 | 148 | DELETE |
| `camera_node.py` | 109 | 293 | DELETE |
| `fallback_nav_node.py` | 171 | 175 | DELETE |
| `fsm_node.py` | 486 | 546 | DELETE |
| `imu_node.py` | 117 | 357 | DELETE |
| `motor_node.py` | 155 | 639 | DELETE |
| `path_recorder_node.py` (Pi side) | 180 | 555 | DELETE (laptop copy still in use, see Active) |
| `servo_node.py` | 69 | 69 | DELETE (sizes match — likely identical or near) |
| `temperature_node.py` | 66 | 63 | DELETE |
| `ultrasonic_node.py` | 92 | 147 | DELETE |
| `voice_node.py` | 145 | 155 | DELETE (note: voice now Android-side too) |
| `watchdog_node.py` | 100 | 151 | DELETE |
| `mqtt_bridge_node.py` | (Pi side) | — | DELETE (replaced by pi_nodes.mqtt_node) |

### 📋 LAUNCH FILES

| File | Status |
|---|---|
| `launch/robot_bringup.launch.py` | **DEPRECATED** (header says so) — DELETE candidate after the legacy nodes above are removed |
| `launch/compute_bringup.launch.py` | **ACTIVE** on laptop |

### 🔧 SUPPORT

| File | Action |
|---|---|
| `robot_pkg/__init__.py` | KEEP (ROS2 package marker) |
| `robot_pkg/hardware/*.py` | Pi-side; replaced by `pi_nodes/hardware/`. DELETE candidate. |
| `setup.py` | Update entry_points after deletions — drop legacy nodes from `console_scripts` |
| `package.xml` | KEEP (ROS2 manifest) |

## Recommended cleanup PR sequence

1. **PR-1**: delete the 13 legacy Pi-side files + remove entries from
   `setup.py` `console_scripts`. ros_ws still builds, compute launch
   still works, robot_bringup.launch.py becomes unbuildable but it's
   already deprecated.
2. **PR-2**: delete `launch/robot_bringup.launch.py` and the
   `hardware/` subdir.
3. **PR-3**: investigate the dual `mqtt_bridge_compute.py` situation —
   pick one canonical source, delete the other, point launch file at
   the survivor.

## Why this PR doesn't perform deletions

Deletes spanning ~2000 LoC across 13+ files would obscure the audit
itself in the diff. Splitting into the three follow-up PRs above keeps
each step reviewable and reverible.
