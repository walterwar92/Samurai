# Follow-me — person tracking + dual P-controller

!!! info "Source"
    `ros_ws/src/robot_pkg/robot_pkg/follow_me_node.py` (~140 lines).
    Consumes YOLO `person` detections; emits `cmd_vel` to maintain a
    target distance while keeping the person centred in frame.

## Distance from bounding-box height

```python title="follow_me_node.py — distance estimation"
bbox_h = best.get('h', 1)
if bbox_h > 10:
    self._person_distance = (PERSON_HEIGHT_M * FOCAL_LENGTH_PX) / bbox_h
else:
    self._person_distance = TARGET_DISTANCE
```

Pinhole-camera model — same as the simulator's
[ball-detection projection](../algorithms/pwm-motor.md):

$$
h_{\text{px}} = \frac{f \cdot H_{\text{real}}}{d}
\;\;\Rightarrow\;\;
\boxed{d = \frac{H_{\text{person}} \cdot f}{h_{\text{bbox}}}}
$$

with $H_{\text{person}} = 1.7\text{ m}$ (average human height),
$f = 500\text{ px}$, and $h_{\text{bbox}}$ the YOLO bounding-box
height in pixels. Bounding boxes shorter than 10 px are rejected as
noise — fall back to the target distance so the robot doesn't react
to a flicker.

## Steering — angular P-controller

```python title="follow_me_node.py — angular control"
person_cx = best.get('x', IMG_CX) + best.get('w', 0) / 2.0
error_x = (person_cx - IMG_CX) / IMG_CX     # normalized -1..+1
twist.angular.z = -error_x * 0.8            # K_w = 0.8
```

Centring error normalised to $[-1, +1]$:

$$
e_x = \frac{c_{\text{person}} - c_{\text{image}}}{c_{\text{image}}}
$$

with $c_{\text{image}} = 320\text{ px}$ (image centre).

Angular command:

$$
\omega = -K_\omega \cdot e_x, \quad K_\omega = 0.8
$$

The negative sign rotates the robot toward the person: person to the
right ($e_x > 0$) → $\omega < 0$ (rotate right).

## Distance — P-controller with dead-band

```python title="follow_me_node.py — linear control"
dist_error = self._person_distance - TARGET_DISTANCE

if dist_error > 0.15:
    twist.linear.x = min(0.20, dist_error * 0.3)
elif dist_error < -0.15:
    twist.linear.x = max(-0.10, dist_error * 0.2)
```

Distance error vs $d_{\text{target}} = 1.0\text{ m}$:

$$
e_d = d_{\text{measured}} - d_{\text{target}}
$$

Speed:

$$
v = \begin{cases}
\min(0.20,\;0.3\,e_d) & e_d > 0.15\text{ m (too far — chase)}\\
\max(-0.10,\;0.2\,e_d) & e_d < -0.15\text{ m (too close — back off)}\\
0 & \text{otherwise (dead band)}
\end{cases}
$$

The dead band prevents oscillation around the target distance.

## Closed-loop transfer functions

Robot is an integrator on each axis — $\dot\theta = \omega$,
$\dot d = -v$.

**Angular loop:**

$$
W_\omega(p) = \frac{K_\omega}{p + K_\omega} = \frac{0.8}{p + 0.8},
\quad \tau_\omega = 1 / K_\omega = 1.25\text{ s}
$$

**Distance loop:**

$$
W_v(p) = \frac{K_v}{p + K_v} = \frac{0.3}{p + 0.3},
\quad \tau_v = 1 / K_v = 3.33\text{ s}
$$

The angular loop is faster than the distance loop ($\tau_\omega <
\tau_v$) — robot rotates toward the person first, *then* drives in.

## Dead band as hysteresis

The $\pm 0.15\text{ m}$ dead band prevents limit-cycle oscillation
(autoperiod) from YOLO-detection latency. Same role as a Schmitt
trigger in analogue electronics.

## Lost-target timeout

```python title="follow_me_node.py — _tick()"
def _tick(self):
    if self._tracking and (time.time() - self._last_person_time) > LOST_TIMEOUT:
        self._tracking = False
        self._cmd_pub.publish(Twist())  # stop
```

Person not seen for $t_{\text{timeout}} = 2\text{ s}$ → robot stops.
Safety net against runaway when YOLO drops the target.

## Parameter reference

| Parameter | Value | Description |
|---|---|---|
| `TARGET_DISTANCE`  | 1.0 m | Target follow distance |
| `PERSON_HEIGHT_M`  | 1.7 m | Average human height (pinhole assumption) |
| `FOCAL_LENGTH_PX`  | 500 px | Camera focal length |
| `IMG_CX`           | 320 px | Image x-centre |
| $K_\omega$         | 0.8   | Angular P gain |
| $K_v^+$            | 0.3   | Speed gain when far |
| $K_v^-$            | 0.2   | Speed gain when close |
| `LOST_TIMEOUT`     | 2.0 s | Timeout before stop |
