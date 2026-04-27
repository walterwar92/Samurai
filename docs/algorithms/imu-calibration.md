# IMU calibration & filtering

!!! info "Source"
    `pi_nodes/nodes/imu_node.py` (~290 lines).
    Talks I2C to an MPU-6050 at address `0x68` via the `I2CDevice` wrapper
    introduced in #13. Pre-EKF pipeline; downstream consumer is the
    [IMU EKF](imu-ekf.md) (#51 tests).

The IMU pipeline does four things between raw I2C bytes and the
filter input:

1. **Read** 14 bytes per sample at 100 Hz.
2. **Scale** to physical units (m/s² and rad/s).
3. **Smooth** via per-channel exponential moving average.
4. **Subtract** static biases learned at startup.

## Hardware: MPU-6050

A 6-axis IMU (3-axis accelerometer + 3-axis gyroscope) on I2C at
`0x68`. Default sensitivity bands:

```python title="imu_node.py — scale constants"
MPU6050_ADDR = 0x68
ACCEL_SCALE = 16384.0    # LSB/g at ±2g
GYRO_SCALE  = 131.0      # LSB/(deg/s) at ±250 deg/s
DEG2RAD = math.pi / 180.0
```

Conversion from raw 16-bit signed integer to physical units:

$$
a_{\text{m/s}^2} = \frac{\text{raw}_{16}}{16384} \times 9.81
$$

$$
\omega_{\text{rad/s}} = \frac{\text{raw}_{16}}{131} \times \frac{\pi}{180}
$$

The 14-byte register read at offset `0x3B` returns
`accel(6) + temp(2) + gyro(6)` in big-endian — temperature is
ignored, so the gyro starts at byte 8:

```python title="imu_node.py — _read_raw()"
raw = self._bus.read_i2c_block_data(MPU6050_ADDR, ACCEL_XOUT_H, 14)
ax = struct.unpack('>h', bytes(raw[0:2]))[0]  / ACCEL_SCALE * 9.81
ay = struct.unpack('>h', bytes(raw[2:4]))[0]  / ACCEL_SCALE * 9.81
az = struct.unpack('>h', bytes(raw[4:6]))[0]  / ACCEL_SCALE * 9.81
gx = struct.unpack('>h', bytes(raw[8:10]))[0] / GYRO_SCALE  * DEG2RAD
gy = struct.unpack('>h', bytes(raw[10:12]))[0] / GYRO_SCALE * DEG2RAD
gz = struct.unpack('>h', bytes(raw[12:14]))[0] / GYRO_SCALE * DEG2RAD
```

The `_bus.read_i2c_block_data()` is wrapped by the `I2CDevice` from
#13 so a stuck I2C bus produces an `I2CTimeout` instead of hanging
the whole node.

## Hardware DLPF + sample rate

```python title="imu_node.py — _init_i2c_bus()"
# DLPF mode 3: accel 44Hz, gyro 42Hz
bus.write_byte_data(MPU6050_ADDR, DLPF_CFG, 0x03)
bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 9)  # 1kHz/(1+9)=100Hz
```

`DLPF_CFG = 3` enables the chip's built-in low-pass: 44 Hz for
accelerometer, 42 Hz for gyroscope. This kills high-frequency
mechanical vibration from the tracks before it ever reaches our code.

Sample rate:

$$
f_{\text{sample}} = \frac{f_{\text{osc}}}{1 + \text{SMPLRT\_DIV}}
                  = \frac{1000}{1 + 9} = 100\text{ Hz}
$$

## Software EMA (per-channel)

```python title="imu_node.py — alpha constants"
ACCEL_EMA_ALPHA = 0.2     # alpha for accel (~2 Hz cutoff)
GYRO_EMA_ALPHA  = 0.5     # alpha for gyro  (~8 Hz cutoff)
```

Exponential moving average — a first-order IIR LPF:

$$
\boxed{\hat x_k = \alpha\,x_k + (1 - \alpha)\,\hat x_{k-1}}
$$

Approximate cutoff frequency:

$$
f_{\text{cutoff}} \approx \frac{\alpha\,f_s}{2\pi(1-\alpha)}
$$

For $\alpha = 0.2$ and $f_s = 50\text{ Hz}$:

$$
f_{\text{cutoff}} \approx \frac{0.2 \times 50}{2\pi \times 0.8}
                  \approx 2.0\text{ Hz}
$$

| Channel | $\alpha$ | Cutoff | Purpose |
|---|---|---|---|
| Accelerometer | 0.2 | ~2 Hz | Aggressive vibration suppression |
| Gyroscope     | 0.5 | ~8 Hz | Mild smoothing — preserves yaw transients |

## Startup calibration

```python title="imu_node.py — _finish_calibration()"
n = len(self._cal_samples)        # 100 samples ≈ 2s @ 50 Hz
sum_ax = sum_ay = sum_az = 0.0
sum_gx = sum_gy = sum_gz = 0.0
for (ax, ay, az, gx, gy, gz) in self._cal_samples:
    sum_ax += ax; sum_ay += ay; sum_az += az
    sum_gx += gx; sum_gy += gy; sum_gz += gz

# Gyro offset: average at rest should be zero
self._gyro_offset = (sum_gx / n, sum_gy / n, sum_gz / n)

# Gravity vector in body frame at rest
avg_ax, avg_ay, avg_az = sum_ax / n, sum_ay / n, sum_az / n
self._gravity_body = (avg_ax, avg_ay, avg_az)

# Initial orientation from gravity
self._home_roll  = math.atan2(avg_ay, avg_az)
self._home_pitch = math.atan2(-avg_ax,
                              math.sqrt(avg_ay*avg_ay + avg_az*avg_az))
```

### Gyro bias

At rest an ideal gyro reads zero, but real sensors have a static DC
offset. Average $N$ samples and subtract from every later reading:

$$
\vect{b}_{\text{gyro}} = \frac{1}{N}\sum_{k=1}^{N} \vect{\omega}_k^{\text{raw}}
\;,\quad N = 100
$$

$$
\vect{\omega}_{\text{corr}} = \vect{\omega}_{\text{raw}} - \vect{b}_{\text{gyro}}
$$

The EKF tracks the residual drift on top of this — see
[IMU EKF chapter](imu-ekf.md), `gz_bias` state.

### Body-frame gravity

The accelerometer's resting average gives the gravity vector in
the IMU's body frame:

$$
\vect{g}_{\text{body}} = \frac{1}{N}\sum_{k=1}^{N} \vect{a}_k^{\text{raw}}
                       = \bigl(\bar a_x, \bar a_y, \bar a_z\bigr)
$$

This vector is later subtracted from every accel reading in the
[IMU push detector](../algorithms/pure-pursuit.md) (#11) to recover
gravity-free linear acceleration.

### Home orientation

From the same averaged gravity vector:

$$
\phi_{\text{home}} = \mathrm{atan2}(\bar a_y, \bar a_z)
$$

$$
\theta_{\text{home}} = \mathrm{atan2}\!\left(-\bar a_x,
                                              \sqrt{\bar a_y^2 + \bar a_z^2}\right)
$$

The EKF's `init_from_calibration()` consumes this so subsequent
roll/pitch outputs are reported relative to the boot-time pose
(rather than absolute IMU mounting angle).

Sanity check on $|\vect g|$:

$$
|\vect g| = \sqrt{\bar a_x^2 + \bar a_y^2 + \bar a_z^2} \approx 9.81\text{ m/s}^2
$$

Wide deviation here means the robot wasn't actually still during
calibration — the user should re-run after the chassis settles.

## Pipeline diagram

```
raw 14 bytes (I2C, 100 Hz)
       │
       ▼
scaling   raw / 16384 × 9.81  (accel)
       │  raw / 131    × π/180 (gyro)
       ▼
EMA LPF   x̂_k = α x_k + (1-α) x̂_{k-1}
       │
       ▼
bias subtract  ω - b_gyro
       │
       ▼
EKF (yaw + bias) ──► (φ, θ, ψ) + linear accel
```

## EMA as a discrete LPF — control-theory view

EMA is a first-order IIR low-pass:

$$
y_k = \alpha\,x_k + (1-\alpha)\,y_{k-1}
$$

Z-transform:

$$
H(z) = \frac{\alpha}{1 - (1-\alpha)\,z^{-1}}
$$

Single pole at $z_p = 1 - \alpha$. Stability requires $|z_p| < 1$,
which holds for any $0 < \alpha < 2$.

Cutoff at the −3 dB point:

$$
f_c = \frac{f_s}{2\pi}\arccos\!\left(\frac{2-\alpha}{2(1-\alpha)}\right)
$$

The EMA is equivalent to a discretised analog RC LPF with

$$
W(p) = \frac{1}{\tau p + 1}, \quad
\tau = \frac{(1-\alpha)\,\Delta t}{\alpha}
$$

For $\alpha = 0.2$ and $\Delta t = 0.02\text{ s}$:
$\tau = 0.8 \cdot 0.02 / 0.2 = 0.08\text{ s}$.

Combined with the chip's hardware DLPF the pipeline rolls off at
−12 dB/oct, comfortably suppressing motor-driven vibration.

## Parameter reference

| Parameter | Value | Description |
|---|---|---|
| `ACCEL_SCALE` | 16384 | LSB/g at ±2g |
| `GYRO_SCALE`  | 131   | LSB/(°/s) at ±250 °/s |
| `DLPF_CFG`    | 3     | Hardware LPF (44/42 Hz) |
| `SMPLRT_DIV`  | 9     | 100 Hz sample rate |
| `ACCEL_EMA_ALPHA` | 0.2 | Accelerometer smoothing |
| `GYRO_EMA_ALPHA`  | 0.5 | Gyroscope smoothing |
| `CALIBRATION_SAMPLES` | 100 | 2 s of static samples |
