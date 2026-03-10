cmake_minimum_required(VERSION 3.8)
project(robot_pkg_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── imu_node ─────────────────────────────────────────────────────────────────
# Direct Linux I2C ioctl → MPU6050 @ 0x68
# Replaces Python imu_node (smbus2) — no GIL, ~10× lower latency
add_executable(imu_node src/imu_node.cpp)
ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
)

# ── motor_node ───────────────────────────────────────────────────────────────
# Direct Linux I2C ioctl → PCA9685 @ 0x5F → 4 DC motors + open-loop odometry
# Replaces Python motor_node (adafruit libs) — deterministic 20 Hz loop
add_executable(motor_node src/motor_node.cpp)
ament_target_dependencies(motor_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  imu_node
  motor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
