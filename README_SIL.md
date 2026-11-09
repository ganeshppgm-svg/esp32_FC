# esp32_FC native Software-in-the-Loop (SIL)

Repository-specific starter implementation for `ganeshppgm-svg/esp32_FC`, targeting **ROS 2 Jazzy + Gazebo Harmonic**. It uses your existing six-motor mixer ordering and separates portable flight-control algorithms from Arduino / ESP32 I/O.

## What is included

```text
repository-root additions/
├── lib/flight_core/                 # Shared, hardware-independent C++ core
│   ├── include/flight_core/
│   ├── src/
│   └── library.json                 # PlatformIO library metadata
├── sil_ws/src/fc_sil_ros2/          # ROS 2 Jazzy adapter package
│   ├── src/fc_sil_node.cpp
│   ├── config/                      # Gazebo bridge + controller params
│   ├── launch/sil.launch.py
│   ├── models/hexacopter/model.sdf   # Six motors + IMU + magnetometer
│   └── worlds/hexacopter_world.sdf
├── tests/                            # Standalone C++ smoke test
└── docs/ESP32_CORE_ADAPTER.md       # How to call this same core from PlatformIO
```

The ROS node compiles the exact source files in `lib/flight_core/src`. PlatformIO discovers the same library through `library.json` when the ESP32 application includes `flight_core/fc_controller.hpp`.

## SIL data flow

```text
Gazebo Harmonic hexacopter
  ├── /fc/imu  (400 Hz, sensor_msgs/Imu)
  └── /fc/mag  (100 Hz, sensor_msgs/MagneticField)
            │
            ▼
      ros_gz_bridge
            │
            ▼
       fc_sil_node  ◄──── /fc/rc (sensor_msgs/Joy)
            │
            ├── flight_core::FlightController::update()
            │     ├── complementary attitude estimator
            │     ├── roll / pitch angle PID
            │     ├── yaw-rate PID
            │     └── original six-motor matrix + desaturation
            │
            ├── /fc/motor_cmd (actuator_msgs/Actuators)
            └── /fc/telemetry (Float32MultiArray)
                         │
                         ▼
               Gazebo motor plugins 0..5
```

PX4 is not part of this control loop. Gazebo produces the sensor measurements and models rotor thrust/torque; your own FC core calculates motor commands.

## 1. Prerequisites

Recommended host: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic. Jazzy + Harmonic is an officially supported pairing. See [Gazebo's ROS installation guidance](https://gazebosim.org/docs/harmonic/ros2_integration/) and [ROS/Gazebo compatibility](https://gazebosim.org/docs/harmonic/).

Install the ROS/Gazebo integration and message dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-actuator-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-rosgraph-msgs \
  python3-colcon-common-extensions
```

Check the environment:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg list | grep -E 'ros_gz_bridge|ros_gz_sim|actuator_msgs'
gz sim --versions
```

## 2. Build the ROS 2 workspace

Keep `lib/flight_core` and `sil_ws` as siblings at the repository root. The package CMake file resolves the repository root three levels above `fc_sil_ros2` and compiles `lib/flight_core` from there. Do not overwrite your existing `README.md` or `.gitignore`; this bundle uses `README_SIL.md` and adds no root `.gitignore`.

```bash
cd /path/to/esp32_fc_sil_bundle/sil_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If `rosdep` is not initialized on the machine, initialize it once using the standard ROS 2 instructions, or install the listed apt dependencies manually.

## 3. Launch Gazebo + bridge + your controller

```bash
cd /path/to/esp32_fc_sil_bundle/sil_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fc_sil_ros2 sil.launch.py
```

The launch file starts:

1. Gazebo Harmonic with `hexacopter_world.sdf`.
2. `ros_gz_bridge` with `config/bridge.yaml`.
3. `fc_sil_node` with `config/sil_params.yaml` and simulation time enabled.

The launch file adds the installed `models` directory to `GZ_SIM_RESOURCE_PATH`, so the world can resolve `model://hexacopter`.

## 4. Verify sensors and motor command topics

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/esp32_fc_sil_bundle/sil_ws/install/setup.bash

ros2 topic list | grep -E '^/clock|/fc/'
ros2 topic echo /fc/imu --once
ros2 topic echo /fc/mag --once
```

Expected topics include:

```text
/clock
/fc/imu
/fc/mag
/fc/rc
/fc/motor_cmd
/fc/telemetry
```

`/fc/rc` is a ROS input topic and will only appear once a publisher is started. The sensor topics originate in Gazebo and are bridged to ROS 2.

## 5. Send RC input and arm in simulation

RC message contract for `/fc/rc` (`sensor_msgs/msg/Joy`):

| Field | Meaning | Range |
|---|---|---|
| `axes[0]` | Roll stick | -1 to +1 |
| `axes[1]` | Pitch stick | -1 to +1 |
| `axes[2]` | Yaw stick / requested yaw rate | -1 to +1 |
| `axes[3]` | Throttle stick | -1 to +1, converted to 0..1 |
| `buttons[0]` | Arm switch | 1 = armed, 0 = disarmed |

Start with neutral roll/pitch/yaw and a throttle axis of `0.10` (mapped to 0.55), and explicitly arm with button 0:

```bash
ros2 topic pub -r 50 /fc/rc sensor_msgs/msg/Joy \
  "{axes: [0.0, 0.0, 0.0, 0.10], buttons: [1]}"
```

Observe the motor commands and telemetry. `/fc/telemetry` is a `Float32MultiArray` with data indices `[roll_deg, pitch_deg, yaw_deg, motor0_norm, motor1_norm, motor2_norm, motor3_norm, motor4_norm, motor5_norm]`.

Observe the motor commands:

```bash
ros2 topic echo /fc/motor_cmd
ros2 topic echo /fc/telemetry
```

Stop the RC publisher or publish the same axes with `buttons: [0]` to disarm. The node also sets all motor speeds to zero if RC or IMU messages time out.

For testing a small roll input, change the first axis, e.g. `axes: [0.10, 0.0, 0.0, 0.10]`. Change one axis at a time during initial testing.

## 6. Motor model and output scaling

The ROS node publishes six `actuator_msgs/msg/Actuators.velocity[]` entries in **rad/s**. Motor index `i` in the message maps to `<motorNumber>i</motorNumber>` in the SDF. The node converts normalized mixer output `u_i` to:

```text
rotor_speed_rad_s[i] = u_i * max_motor_speed_rad_s
```

Both the ROS parameter `max_motor_speed_rad_s` and each SDF `<maxRotVelocity>` are set to `800.0` in this starter configuration. Keep them identical.

The motor model uses the Gazebo example's motor constants as an initial placeholder. The hexacopter body is also a nominal 1.0 kg model. These are simulation assumptions, **not measurements of your eventual aircraft**. Update mass, inertia, arm length, motor constant, moment constant, rotor drag, and maximum speed to match your actual airframe and motor/propeller test data.

## 7. Configuration and tuning

Edit:

```text
sil_ws/src/fc_sil_ros2/config/sil_params.yaml
```

The controller currently uses:

- Roll and pitch: angle setpoint PID, with derivative damping from gyro rate.
- Yaw: yaw-rate PID, with stick deflection mapped to a desired yaw rate.
- Attitude: complementary filter on accelerometer + gyro, with magnetometer correction when a valid magnetic vector is available.
- Motor allocation: the six-row mixer retained from your `src/Application/motor_mixing.cpp`, with desaturation to keep each output between 0 and 1.

The gain values are conservative **starting values for SIL experiments only**, not a validated tuning set. Begin with low correction limits. Verify control signs and motor order using small stick inputs and telemetry. The yaw sign may need inversion depending on how your actual rotor spin directions map to Gazebo reaction torque. Do not treat a successful SIL build as approval for real flight.

## 8. Standalone core smoke test (no ROS required)

This verifies that the reusable library compiles with a normal Linux C++ compiler and that the mixer output remains within [0, 1]:

```bash
cd /path/to/esp32_fc_sil_bundle/tests
cmake -S . -B build
cmake --build build -j
./build/core_smoke_test
```

Expected output:

```text
flight_core smoke test passed
```

## 9. Reuse the core in PlatformIO / ESP32-S3

The bundle includes `lib/flight_core/library.json`, so PlatformIO can discover it when the existing firmware is updated to include:

```cpp
#include <flight_core/fc_controller.hpp>
```

Your hardware adapter must translate MPU6050 values (m/s², rad/s), calibrated HMC5883L values, and RC commands into `flight_core::SensorData` / `flight_core::RcInput`, call `FlightController::update()`, then map the six normalized values to ESC pulse widths. See [`docs/ESP32_CORE_ADAPTER.md`](docs/ESP32_CORE_ADAPTER.md).

The old `src/Application/attitude_control.cpp`, `src/Application/motor_mixing.cpp`, and `src/abstraction/angle_estimation.cpp` are not compiled by this ROS package; the ROS package uses the shared library. Migrate the ESP32 `main.cpp` to call the same core before excluding those old translation units from the PlatformIO build. Do not compile both old and new algorithm implementations into the MCU application.

The shared types use C++17 language features. Add this to the PlatformIO environment (ESP32 GCC toolchains used by current PlatformIO Espressif32 platforms support this mode):

```ini
build_flags = -std=gnu++17
```

## 10. Important source-specific fixes identified in the current repository

Before using the same code on the ESP32, also fix these existing issues:

1. In `src/main.cpp`, `void updateRCCMD();` is a declaration, not a call; use `updateRCCMD();`.
2. `main.cpp` currently reads six mixer outputs but does not call `sendESC(esc1, ..., esc6)` in the normal armed path.
3. `getRollCMD()`, `getPitchCMD()`, and `getYawCMD()` are already scaled to degrees by `RCreciever_driver.cpp`; `main.cpp` multiplies by `MAX_*` again.
4. Current `FC_config.h` contains zero placeholders for RC pulse min/max and ESC pulse min/max.
5. Current `control_params.h` has all gains and PID limits set to zero.
6. `angle_estimation.cpp` calls `micros()`, `updateSensorData()`, and `getHeading()` internally; the shared estimator in this bundle instead receives sensor data and `dt` as inputs.
7. The old mixer only clamps high outputs. The shared mixer performs desaturation while keeping all six outputs inside [0,1].

## 11. Limitations of this first SIL increment

- The existing BMP280 driver is not used in the current main control path, and this first increment does not implement altitude hold.
- GPS / position control is not included.
- The world defines an approximate 45 µT magnetic field and the magnetometer is bridged. Check `/fc/mag` and verify its axis/sign behavior before relying on magnetic yaw correction.
- This is a native Linux build of the portable control core, not cycle-accurate ESP32-S3 emulation. It does not validate FreeRTOS scheduling, ESP32 interrupts, I2C drivers, PWM peripherals, or real-time deadlines on the MCU.
- A successful compile or stable simulation is not evidence of flight readiness. The motor constants, inertia, frame geometry, coordinate signs, sensor biases, and gains all require validation.
