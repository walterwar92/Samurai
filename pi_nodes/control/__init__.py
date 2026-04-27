"""Pure-Python control policies extracted from motor_node for testability.

These classes hold their own state but never touch MQTT, GPIO, or hardware
directly — call sites pass in observations and receive decisions back. That
keeps the safety-critical loop in a single process (MQTT round-trip would
add 10-50 ms to collision response, unacceptable) while letting each policy
be unit-tested in isolation.

Submodules
----------
collision_policy   — collision avoidance state machine
imu_push_detector  — passive movement detection from accelerometer
drive_state        — cmd_vel multiplexer state

State-space control (offline-designed, online-applied):
state_space_model  — discrete plant x[k+1] = Ad x[k] + Bd u[k]
lqr_controller     — LQR feedback u = -K(x - x_ref)
mpc_controller     — MPC with explicit gain + clip projection
controller_factory — picks (off|lqr|mpc|modal) from config.yaml

The matrices A, B, K, Pf are computed offline by `matlab/main.m` and
exported into `config.yaml` (`control:` section). See
`latex_doc/control_theory/` for the math.
"""
