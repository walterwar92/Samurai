# Patrol — waypoint sequence with Nav2 goal publishing

!!! info "Source"
    `ros_ws/src/robot_pkg/robot_pkg/patrol_node.py` (~145 lines).
    Cyclically issues PoseStamped goals to Nav2 and watches the
    robot's reported position to advance to the next waypoint.

## Reaching a waypoint

```python title="patrol_node.py — _tick()"
# Check if reached
dist = math.hypot(tx - self._robot_x, ty - self._robot_y)
if dist < GOAL_TOLERANCE:   # 0.20 m
    self._current_idx = (self._current_idx + 1) % len(self._waypoints)
    self._goal_sent = False
```

Euclidean distance to the current target:

$$
d = \sqrt{(t_x - x_r)^2 + (t_y - y_r)^2} = \mathrm{hypot}(\Delta x, \Delta y)
$$

A waypoint is "reached" once $d < 0.20\text{ m}$.

Cyclic indexing wraps after the last waypoint:

$$
i_{\text{next}} = (i_{\text{curr}} + 1) \bmod N_{\text{waypoints}}
$$

## Sending a Nav2 goal (yaw → quaternion)

```python title="patrol_node.py — _send_goal()"
def _send_goal(self, x, y, yaw):
    goal = PoseStamped()
    goal.header.stamp = self.get_clock().now().to_msg()
    goal.header.frame_id = 'map'
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)
    self._goal_pub.publish(goal)
```

For a planar robot the goal pose has only a yaw component. Pure
rotation around $Z$:

$$
\boxed{
q_z = \sin\!\left(\tfrac{\psi}{2}\right), \quad
q_w = \cos\!\left(\tfrac{\psi}{2}\right), \quad
q_x = q_y = 0
}
$$

This follows from Rodrigues' formula with rotation axis
$\hat{\vect{n}} = (0, 0, 1)$:

$$
\vect{q} = \cos\frac{\psi}{2} + \hat{\vect{n}}\sin\frac{\psi}{2}
        = \bigl(0,\;0,\;\sin\tfrac{\psi}{2},\;\cos\tfrac{\psi}{2}\bigr)
$$

See the [Quaternions chapter](quaternions.md) for the full rotation
matrix and the inverse `atan2` extraction used when receiving Nav2
status messages.

!!! note "ROS2 quaternion field order"
    `geometry_msgs/Quaternion` orders the fields as
    $(q_x, q_y, q_z, q_w)$. For a 2D robot only $q_z$ and $q_w$ need
    setting; the others stay zero by default.

## Patrol geometry

```
        WP₃                    WP₂
          ●─────────────────────●
          │                     │
          │      ●  Robot       │
          │ d < 0.20 m? ●→  ●   │
          │                     │
          ●─────────────────────●
        WP₀                    WP₁

        Tolerance circles (green dashed) at each waypoint;
        next waypoint advanced once robot enters a circle.
```

## Parameters

| Parameter | Value | Description |
|---|---|---|
| `GOAL_TOLERANCE` | 0.20 m | Distance under which a waypoint is considered reached |
| Update rate      | 2 Hz   | Frequency of goal-checking tick |
| Goal orientation | $(0, 0, \sin\tfrac{\psi}{2}, \cos\tfrac{\psi}{2})$ | Yaw-only quaternion |
