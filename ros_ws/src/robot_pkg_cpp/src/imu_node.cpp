/**
 * imu_node — MPU6050 6-axis IMU via direct Linux I2C ioctl.
 *
 * C++ replaces Python imu_node (smbus2):
 *   - No GIL, no Python overhead — timer callback: ~10 µs vs ~1 ms
 *   - Same topic, same message format, same QoS as Python version
 *
 * I2C:      /dev/i2c-1, address 0x68
 * Reads:    14 bytes from ACCEL_XOUT_H (0x3B): ax ay az temp gx gy gz
 * Publishes: /imu/data  (sensor_msgs/Imu)  @ 50 Hz, BEST_EFFORT QoS
 */

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// ── MPU6050 constants ─────────────────────────────────────────────────────────
static constexpr int     MPU6050_ADDR   = 0x68;
static constexpr int     I2C_BUS        = 1;        // /dev/i2c-1
static constexpr double  ACCEL_SCALE    = 16384.0;  // ±2 g → LSB/g
static constexpr double  GYRO_SCALE     = 131.0;    // ±250 °/s → LSB/(°/s)
static constexpr double  DEG2RAD        = M_PI / 180.0;
static constexpr double  GRAVITY        = 9.81;

static constexpr uint8_t REG_PWR_MGMT_1  = 0x6B;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

// ─────────────────────────────────────────────────────────────────────────────
class IMUNode : public rclcpp::Node
{
public:
  IMUNode() : Node("imu_node"), fd_(-1), simulated_(false)
  {
    std::string dev = "/dev/i2c-" + std::to_string(I2C_BUS);
    fd_ = open(dev.c_str(), O_RDWR);

    if (fd_ < 0) {
      RCLCPP_WARN(get_logger(), "Cannot open %s — IMU simulated", dev.c_str());
      simulated_ = true;
    } else if (ioctl(fd_, I2C_SLAVE, MPU6050_ADDR) < 0) {
      RCLCPP_WARN(get_logger(),
                  "I2C_SLAVE 0x%02X failed — IMU simulated", MPU6050_ADDR);
      close(fd_);
      fd_ = -1;
      simulated_ = true;
    } else {
      // Wake MPU6050: clear sleep bit in PWR_MGMT_1
      uint8_t buf[2] = {REG_PWR_MGMT_1, 0x00};
      if (write(fd_, buf, 2) != 2) {
        RCLCPP_WARN(get_logger(), "MPU6050 wake-up failed — IMU simulated");
        close(fd_);
        fd_ = -1;
        simulated_ = true;
      } else {
        RCLCPP_INFO(get_logger(), "MPU6050 initialised on /dev/i2c-%d", I2C_BUS);
      }
    }

    // BEST_EFFORT + KEEP_LAST(1): only the latest sample matters at 50 Hz
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                  .best_effort()
                  .durability_volatile();
    pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", qos);

    // 50 Hz timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&IMUNode::readAndPublish, this));
  }

  ~IMUNode()
  {
    if (fd_ >= 0) { close(fd_); }
  }

private:
  struct RawImu {
    double ax, ay, az;  // m/s²
    double gx, gy, gz;  // rad/s
  };

  RawImu readRaw()
  {
    if (simulated_) {
      return {0.0, 0.0, GRAVITY, 0.0, 0.0, 0.0};
    }

    // Point MPU6050 register cursor to ACCEL_XOUT_H
    uint8_t reg = REG_ACCEL_XOUT_H;
    if (write(fd_, &reg, 1) != 1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "MPU6050: register write failed");
      return {0.0, 0.0, GRAVITY, 0.0, 0.0, 0.0};
    }

    // Read 14 bytes: AX_H AX_L AY_H AY_L AZ_H AZ_L TEMP_H TEMP_L GX_H GX_L GY_H GY_L GZ_H GZ_L
    uint8_t raw[14] = {};
    if (read(fd_, raw, 14) != 14) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "MPU6050: read 14 bytes failed");
      return {0.0, 0.0, GRAVITY, 0.0, 0.0, 0.0};
    }

    auto to_int16 = [](uint8_t hi, uint8_t lo) -> int16_t {
      return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
    };

    // Bytes 0-5: accelerometer (X,Y,Z)
    double ax = to_int16(raw[0],  raw[1])  / ACCEL_SCALE * GRAVITY;
    double ay = to_int16(raw[2],  raw[3])  / ACCEL_SCALE * GRAVITY;
    double az = to_int16(raw[4],  raw[5])  / ACCEL_SCALE * GRAVITY;
    // Bytes 6-7: temperature (skipped)
    // Bytes 8-13: gyroscope (X,Y,Z)
    double gx = to_int16(raw[8],  raw[9])  / GYRO_SCALE * DEG2RAD;
    double gy = to_int16(raw[10], raw[11]) / GYRO_SCALE * DEG2RAD;
    double gz = to_int16(raw[12], raw[13]) / GYRO_SCALE * DEG2RAD;

    return {ax, ay, az, gx, gy, gz};
  }

  void readAndPublish()
  {
    auto raw = readRaw();

    sensor_msgs::msg::Imu msg;
    msg.header.stamp    = now();
    msg.header.frame_id = "imu_link";

    // Orientation: unknown → signal with covariance[0] = -1
    msg.orientation_covariance[0] = -1.0;

    msg.angular_velocity.x = raw.gx;
    msg.angular_velocity.y = raw.gy;
    msg.angular_velocity.z = raw.gz;
    msg.angular_velocity_covariance[0] = 0.01;
    msg.angular_velocity_covariance[4] = 0.01;
    msg.angular_velocity_covariance[8] = 0.01;

    msg.linear_acceleration.x = raw.ax;
    msg.linear_acceleration.y = raw.ay;
    msg.linear_acceleration.z = raw.az;
    msg.linear_acceleration_covariance[0] = 0.1;
    msg.linear_acceleration_covariance[4] = 0.1;
    msg.linear_acceleration_covariance[8] = 0.1;

    pub_->publish(msg);
  }

  int  fd_;
  bool simulated_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr                        timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUNode>());
  rclcpp::shutdown();
  return 0;
}
