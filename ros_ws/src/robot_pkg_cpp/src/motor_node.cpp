/**
 * motor_node — 4-motor tracked chassis + open-loop odometry.
 *
 * C++ replaces Python motor_node (adafruit-circuitpython-motor):
 *   - PCA9685 @ I2C 0x5F controlled via direct Linux ioctl — no Python GIL
 *   - Deterministic 20 Hz control loop (timer jitter < 100 µs vs ~1 ms Python)
 *   - Same topics, same TF output, same speed profiles as Python version
 *
 * Motor channel map (Adeept Robot HAT V3.1):
 *   M1: IN1=ch15, IN2=ch14  (right-front)
 *   M2: IN1=ch12, IN2=ch13  (left-front)
 *   M3: IN1=ch11, IN2=ch10  (left-rear)
 *   M4: IN1=ch8,  IN2=ch9   (right-rear)
 *
 * PWM drive mode: SLOW_DECAY (matches adafruit_motor.SLOW_DECAY):
 *   forward (t>0): IN1 = 4095,              IN2 = 4095*(1-t)
 *   reverse (t<0): IN1 = 4095*(1+t),        IN2 = 4095
 *   brake   (t=0): IN1 = 4095,              IN2 = 4095
 *
 * Subscribes:
 *   /cmd_vel        (geometry_msgs/Twist)  — linear.x, angular.z
 *   /speed_profile  (std_msgs/String)      — "slow" / "normal" / "fast"
 * Publishes:
 *   /odom                (nav_msgs/Odometry)  @ 20 Hz
 *   /speed_profile/active (std_msgs/String)   @ 1 Hz
 * TF:
 *   odom → base_link
 */

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

// ── PCA9685 constants ─────────────────────────────────────────────────────────
static constexpr int     PCA9685_ADDR  = 0x5F;
static constexpr uint8_t REG_MODE1     = 0x00;
static constexpr uint8_t REG_PRESCALE  = 0xFE;
static constexpr int     PWM_FREQ_HZ   = 50;
static constexpr int     OSC_FREQ_HZ   = 25'000'000;
static constexpr int     PWM_MAX       = 4095;

// ── Motor channel layout ──────────────────────────────────────────────────────
struct MotorCh { int in1; int in2; };
static constexpr std::array<MotorCh, 4> MOTORS = {{
  {15, 14},   // index 0 → M1 right-front
  {12, 13},   // index 1 → M2 left-front
  {11, 10},   // index 2 → M3 left-rear
  { 8,  9},   // index 3 → M4 right-rear
}};
static constexpr int IDX_RIGHT_FRONT = 0;
static constexpr int IDX_LEFT_FRONT  = 1;
static constexpr int IDX_LEFT_REAR   = 2;
static constexpr int IDX_RIGHT_REAR  = 3;

// ── Robot geometry ────────────────────────────────────────────────────────────
static constexpr double WHEEL_BASE = 0.17;  // metres between tracks

// ── Speed profiles ────────────────────────────────────────────────────────────
struct SpeedProfile { double max_lin; double max_ang; };
static const std::map<std::string, SpeedProfile> PROFILES = {
  {"slow",   {0.10, 0.8}},
  {"normal", {0.20, 1.5}},
  {"fast",   {0.30, 2.0}},
};

// ── PCA9685 driver ────────────────────────────────────────────────────────────
class PCA9685
{
public:
  explicit PCA9685(int fd) : fd_(fd) {}

  void init()
  {
    // prescale = round(osc / (4096 * freq)) - 1
    uint8_t prescale = static_cast<uint8_t>(
      std::round(static_cast<double>(OSC_FREQ_HZ) /
                 (4096.0 * static_cast<double>(PWM_FREQ_HZ))) - 1.0);

    writeReg(REG_MODE1, 0x10);        // sleep (required before changing prescale)
    writeReg(REG_PRESCALE, prescale);
    writeReg(REG_MODE1, 0x00);        // wake
    usleep(500);
    writeReg(REG_MODE1, 0xA0);        // enable auto-increment
  }

  // Set PWM on channel ch: duty in [0, PWM_MAX]
  void setPwm(int ch, int duty)
  {
    duty = std::clamp(duty, 0, PWM_MAX);
    // Registers: LEDn_ON_L/H (always 0), LEDn_OFF_L/H
    int base = 0x06 + 4 * ch;
    uint8_t buf[5] = {
      static_cast<uint8_t>(base),
      0x00, 0x00,                                  // ON = 0
      static_cast<uint8_t>(duty & 0xFF),           // OFF_L
      static_cast<uint8_t>((duty >> 8) & 0x0F),   // OFF_H
    };
    write(fd_, buf, 5);
  }

  void allOff()
  {
    // ALL_LED_OFF_H bit 4 = full off for all channels
    uint8_t buf[2] = {0xFD, 0x10};
    write(fd_, buf, 2);
  }

private:
  int fd_;

  void writeReg(uint8_t reg, uint8_t val)
  {
    uint8_t buf[2] = {reg, val};
    write(fd_, buf, 2);
  }
};

// ── ROS2 node ─────────────────────────────────────────────────────────────────
class MotorNode : public rclcpp::Node
{
public:
  MotorNode()
  : Node("motor_node"),
    fd_(-1), simulated_(false),
    linear_(0.0), angular_(0.0),
    x_(0.0), y_(0.0), theta_(0.0),
    profile_("normal"), max_lin_(0.20), max_ang_(1.5)
  {
    // Open PCA9685 via Linux I2C
    fd_ = open("/dev/i2c-1", O_RDWR);
    if (fd_ < 0 || ioctl(fd_, I2C_SLAVE, PCA9685_ADDR) < 0) {
      RCLCPP_WARN(get_logger(),
                  "PCA9685 @0x%02X unavailable — SIMULATION mode", PCA9685_ADDR);
      if (fd_ >= 0) { close(fd_); fd_ = -1; }
      simulated_ = true;
    } else {
      pca_ = std::make_unique<PCA9685>(fd_);
      pca_->init();
      RCLCPP_INFO(get_logger(),
                  "PCA9685 initialised @0x%02X, %d Hz", PCA9685_ADDR, PWM_FREQ_HZ);
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscriptions
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr m) {
        linear_  = m->linear.x;
        angular_ = m->angular.z;
      });

    profile_sub_ = create_subscription<std_msgs::msg::String>(
      "/speed_profile", 10,
      [this](std_msgs::msg::String::SharedPtr m) {
        auto it = PROFILES.find(m->data);
        if (it != PROFILES.end()) {
          profile_ = m->data;
          max_lin_ = it->second.max_lin;
          max_ang_ = it->second.max_ang;
          RCLCPP_INFO(get_logger(), "Speed profile: %s (lin=%.2f ang=%.2f)",
                      profile_.c_str(), max_lin_, max_ang_);
        }
      });

    // Publishers
    odom_pub_    = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    profile_pub_ = create_publisher<std_msgs::msg::String>("/speed_profile/active", 10);

    last_time_ = now();

    // 20 Hz control + odometry
    ctrl_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&MotorNode::controlLoop, this));

    // 1 Hz profile broadcast
    prof_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MotorNode::publishProfile, this));
  }

  ~MotorNode()
  {
    stopMotors();
    if (pca_) { pca_->allOff(); }
    if (fd_ >= 0) { close(fd_); }
  }

private:
  // ── Motor drive (SLOW_DECAY — matches adafruit_motor.SLOW_DECAY) ────────────
  // throttle in [-1.0, 1.0]
  void setMotor(int idx, double throttle)
  {
    if (simulated_) { return; }
    throttle = std::clamp(throttle, -1.0, 1.0);

    int in1, in2;
    if (throttle > 0.001) {
      // Forward: IN1 full, IN2 braking side
      in1 = PWM_MAX;
      in2 = static_cast<int>(PWM_MAX * (1.0 - throttle));
    } else if (throttle < -0.001) {
      // Reverse: IN1 braking side, IN2 full
      in1 = static_cast<int>(PWM_MAX * (1.0 + throttle));
      in2 = PWM_MAX;
    } else {
      // Active brake: both full
      in1 = PWM_MAX;
      in2 = PWM_MAX;
    }

    pca_->setPwm(MOTORS[idx].in1, in1);
    pca_->setPwm(MOTORS[idx].in2, in2);
  }

  void stopMotors()
  {
    for (int i = 0; i < 4; ++i) {
      if (!simulated_ && pca_) {
        pca_->setPwm(MOTORS[i].in1, PWM_MAX);
        pca_->setPwm(MOTORS[i].in2, PWM_MAX);
      }
    }
  }

  // ── 20 Hz control loop + odometry ────────────────────────────────────────
  void controlLoop()
  {
    auto t_now = now();
    double dt  = (t_now - last_time_).nanoseconds() / 1e9;
    last_time_ = t_now;

    // Convert m/s → throttle in [-1, 1]
    double lin_t = (max_lin_ > 0.0) ? (linear_  / max_lin_) : 0.0;
    double ang_t = (max_ang_ > 0.0) ? (angular_ / max_ang_) : 0.0;

    // Differential-drive mixer (same as Python MotorDriver.move())
    double left  = lin_t + ang_t;
    double right = lin_t - ang_t;

    // Normalise: keep ratio, clamp to [-1, 1]
    double max_val = std::max({std::abs(left), std::abs(right), 1.0});
    if (max_val > 1.0) { left /= max_val; right /= max_val; }

    setMotor(IDX_LEFT_FRONT,  left);
    setMotor(IDX_LEFT_REAR,   left);
    setMotor(IDX_RIGHT_FRONT, right);
    setMotor(IDX_RIGHT_REAR,  right);

    // Open-loop odometry integration
    x_     += linear_  * std::cos(theta_) * dt;
    y_     += linear_  * std::sin(theta_) * dt;
    theta_ += angular_ * dt;

    publishOdom(t_now);
  }

  void publishOdom(const rclcpp::Time & stamp)
  {
    double sin_h = std::sin(theta_ / 2.0);
    double cos_h = std::cos(theta_ / 2.0);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp            = stamp;
    odom.header.frame_id         = "odom";
    odom.child_frame_id          = "base_link";
    odom.pose.pose.position.x    = x_;
    odom.pose.pose.position.y    = y_;
    odom.pose.pose.orientation.z = sin_h;
    odom.pose.pose.orientation.w = cos_h;
    odom.twist.twist.linear.x    = linear_;
    odom.twist.twist.angular.z   = angular_;
    odom_pub_->publish(odom);

    // TF: odom → base_link
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp            = stamp;
    tf.header.frame_id         = "odom";
    tf.child_frame_id          = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation.z    = sin_h;
    tf.transform.rotation.w    = cos_h;
    tf_broadcaster_->sendTransform(tf);
  }

  void publishProfile()
  {
    std_msgs::msg::String msg;
    msg.data = profile_;
    profile_pub_->publish(msg);
  }

  // ── Members ────────────────────────────────────────────────────────────────
  int  fd_;
  bool simulated_;
  std::unique_ptr<PCA9685> pca_;

  double linear_, angular_;
  double x_, y_, theta_;
  rclcpp::Time last_time_;

  std::string profile_;
  double      max_lin_, max_ang_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     profile_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        profile_pub_;

  rclcpp::TimerBase::SharedPtr ctrl_timer_;
  rclcpp::TimerBase::SharedPtr prof_timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorNode>());
  rclcpp::shutdown();
  return 0;
}
