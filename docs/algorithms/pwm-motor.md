# PWM motor control & PCA9685

!!! info "Source"
    `pi_nodes/hardware/motor_driver.py` (Pure-Python).
    `ros_ws/src/robot_pkg_cpp/src/motor_node.cpp` (legacy C++ ROS2 reference).
    Hardware: PCA9685 16-channel PWM controller on I2C at `0x5F`.

The differential-drive robot has two DC motors driven through the
PCA9685's PWM channels in **slow-decay** mode. This chapter documents
how a normalised throttle command translates into the four PWM
register writes per motor.

## PCA9685 controller

16 channels, 12-bit duty cycle ($0 \dots 4095$), I2C interface,
configurable PWM frequency via the prescaler register.

### Prescale calculation

```cpp title="motor_node.cpp — setPwmFreq()"
// prescale = round(osc / (4096 * freq)) - 1
uint8_t prescale = static_cast<uint8_t>(
  std::round(static_cast<double>(OSC_FREQ_HZ) /
             (4096.0 * static_cast<double>(PWM_FREQ_HZ))) - 1.0);
```

Derived from the chip's reference clock divider:

$$
\boxed{\text{prescale} = \left\lfloor \frac{f_{\text{osc}}}{4096 \cdot f_{\text{PWM}}} + 0.5 \right\rfloor - 1}
$$

For $f_{\text{osc}} = 25\text{ MHz}$ and $f_{\text{PWM}} = 50\text{ Hz}$:

$$
\text{prescale} = \left\lfloor \frac{25{,}000{,}000}{4096 \times 50} + 0.5 \right\rfloor - 1
                = \left\lfloor 122.07 + 0.5 \right\rfloor - 1 = 121
$$

50 Hz is the standard servo frequency — same chip drives both the
motor pair and the arm/head servos, so they share the prescaler
setting.

## Slow-decay drive mode

Each motor has two PCA9685 channels — call them IN1 and IN2 — wired
to a half-bridge driver. Slow-decay (synchronous-rectification-like)
keeps both pins switching, which gives smooth braking and lower
ripple than fast-decay.

```cpp title="motor_node.cpp — setMotor()"
void setMotor(int idx, double throttle)
{
  throttle = std::clamp(throttle, -1.0, 1.0);
  int in1, in2;
  if (throttle > 0.001) {
    // Forward: IN1 = 4095, IN2 = 4095 * (1 - t)
    in1 = PWM_MAX;
    in2 = static_cast<int>(PWM_MAX * (1.0 - throttle));
  } else if (throttle < -0.001) {
    // Reverse: IN1 = 4095 * (1 + t), IN2 = 4095
    in1 = static_cast<int>(PWM_MAX * (1.0 + throttle));
    in2 = PWM_MAX;
  } else {
    // Active brake: IN1 = IN2 = 4095
    in1 = PWM_MAX;
    in2 = PWM_MAX;
  }
  pca_->setPwm(MOTORS[idx].in1, in1);
  pca_->setPwm(MOTORS[idx].in2, in2);
}
```

### Forward ($t > 0$, throttle ∈ (0, 1])

$$
\text{IN1} = 4095, \quad \text{IN2} = 4095 \cdot (1 - t)
$$

IN1 is at full duty (always high during the period), IN2 modulates
the inverse — net voltage across the motor is $V_{\text{cc}} \cdot t$.

### Reverse ($t < 0$, throttle ∈ [−1, 0))

$$
\text{IN1} = 4095 \cdot (1 + t), \quad \text{IN2} = 4095
$$

Symmetric to forward — IN2 is fixed high, IN1 modulates. Net voltage
$-V_{\text{cc}} \cdot |t|$.

### Active brake ($t = 0$)

$$
\text{IN1} = \text{IN2} = 4095
$$

Both pins held high — the H-bridge shorts the motor windings through
the upper transistors. Back-EMF drives a braking current proportional
to angular velocity (see derivation below). This is **active**
braking, distinct from coasting.

### Channel layout

| Motor | Position    | IN1 channel | IN2 channel |
|-------|-------------|-------------|-------------|
| M1    | Left rear   | ch 11       | ch 10       |
| M2    | Right rear  | ch 8        | ch 9        |

Two motors only — the chassis has rear-tracked drive and no front
wheels.

## Differential-drive mixer

```cpp title="motor_node.cpp — controlLoop()"
double lin_t = (max_lin_ > 0.0) ? (linear_  / max_lin_) : 0.0;
double ang_t = (max_ang_ > 0.0) ? (angular_ / max_ang_) : 0.0;

double left  = lin_t + ang_t;
double right = lin_t - ang_t;

double max_val = std::max({std::abs(left), std::abs(right), 1.0});
if (max_val > 1.0) { left /= max_val; right /= max_val; }
```

Normalised commands ($\ell \in [-1, 1]$):

$$
\ell_{\text{lin}} = \frac{v}{v_{\max}}, \quad
\ell_{\text{ang}} = \frac{\omega}{\omega_{\max}}
$$

$$
\text{left} = \ell_{\text{lin}} + \ell_{\text{ang}}, \quad
\text{right} = \ell_{\text{lin}} - \ell_{\text{ang}}
$$

Saturation-preserving normalisation:

$$
m = \max(1, |\text{left}|, |\text{right}|), \quad
\text{left} \leftarrow \frac{\text{left}}{m}, \quad
\text{right} \leftarrow \frac{\text{right}}{m}
$$

When sum exceeds the actuator bound, **both tracks scale by the same
factor** — yaw rate stays correct even when full forward speed isn't
achievable. See [Odometry chapter](odometry.md) for the same mixer
described at the kinematic level.

## DC-motor model

Robot motors obey the classical DC-motor differential equations:

$$
L_a \frac{dI}{dt} + R_a I = U - C_e \omega
$$

$$
J \frac{d\omega}{dt} = C_m I - M_{\text{load}}
$$

with $L_a$, $R_a$ as armature inductance / resistance, $C_e$ the
back-EMF constant, $C_m$ the torque constant, $J$ the rotor inertia,
and $M_{\text{load}}$ the load torque.

Eliminating current gives a second-order transfer function:

$$
W(p) = \frac{\omega(p)}{U(p)} = \frac{K_m}{T_m T_e p^2 + T_m p + 1}
$$

where $K_m = 1 / C_e$, $T_e = L_a / R_a$, $T_m = J R_a / (C_e C_m)$.

In our regime $T_e \ll T_m \ll T_{\text{control}}$ (electrical time
constant ≪ mechanical ≪ control period), so the dynamics collapse to
a static gain — exactly why the [Odometry chapter](odometry.md) is
content with the kinematic model.

## Active-brake torque

In active-brake mode the windings are shorted by holding both PCA
pins high. The back-EMF $C_e\omega$ drives a current

$$
I_{\text{brake}} = \frac{C_e \omega}{R_a}
$$

which in turn produces a braking torque

$$
M_{\text{brake}} = C_m I_{\text{brake}} = \frac{C_e C_m}{R_a}\omega
$$

That's a viscous-damping term: braking force is proportional to
velocity. Smooth deceleration without overshoot — the right behaviour
when the controller commands "stop".

## Closed-loop mixer + motor

Stacking the mixer onto the motor model:

$$
\begin{pmatrix} V_L(p) \\ V_R(p) \end{pmatrix}
= \frac{K_m}{T_m p + 1}
  \begin{pmatrix} 1 & 1 \\ 1 & -1 \end{pmatrix}
  \begin{pmatrix} \ell_{\text{lin}}(p) \\ \ell_{\text{ang}}(p) \end{pmatrix}
$$

When $T_m \ll T_{\text{control}}$:

$$
V_L \approx K_m (\ell_{\text{lin}} + \ell_{\text{ang}})
$$

That static-gain approximation is what the kinematic odometry assumes
when integrating commanded velocity into pose.

## Parameter reference

| Parameter | Value | Description |
|---|---|---|
| `PCA9685_ADDR` | `0x5F` | I2C address |
| `PWM_FREQ_HZ`  | 50 Hz  | PWM carrier frequency |
| `PWM_MAX`      | 4095   | 12-bit duty maximum |
| `WHEEL_BASE`   | 0.17 m | Distance between tracks |

After #46 these defaults are also reachable through `config.yaml`
under `wheel_calibration.*` and `motor.*` keys.
