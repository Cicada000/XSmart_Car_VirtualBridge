# XSmart_Car_VirtualBridge

`VirtualBridge` 是 `XSmart_Car_LineFollower` 的虚拟下位机与虚拟定位桥接程序。它用于在没有真实小车、真实串口下位机和真实 ArUco 相机定位系统时，闭环测试上位机循线控制程序。

程序做两件事：

1. 监听 `XSmart_Car_LineFollower` 原本发给 `icar_socket_bridge` 的 TCP 控制帧，解析速度和舵机 PWM。
2. 使用简化车辆运动学模型积分出虚拟车辆位置，并按 `ArucoCalibCpp` 的 `robot_position` UDP 格式发送给板卡上的定位接收程序。

因此虚拟链路是：

```text
XSmart_Car_LineFollower
  -> TCP 127.0.0.1:8899 speed/servo control frame
  -> VirtualBridge
  -> UDP robot_position
  -> setupUI / xverse_ar_engine / existing pose bridge
  -> XSmart_Car_LineFollower receives sync pose
```

使用虚拟桥时，不要同时启动真实的 `icar_socket_bridge`，因为两个程序都会占用 `127.0.0.1:8899`，而且真实桥会把控制指令发到物理小车。

## Project Layout

```text
VirtualBridge/
├── CMakeLists.txt
├── README.md
├── include/virtual_bridge/
│   ├── ControlFrame.hpp
│   ├── RobotPositionJson.hpp
│   └── VehicleModel.hpp
├── src/
│   ├── ControlFrame.cpp
│   ├── RobotPositionJson.cpp
│   ├── VehicleModel.cpp
│   └── virtual_aruco_bridge.cpp
└── tests/
    └── virtual_bridge_tests.cpp
```

### Core Files

- `include/virtual_bridge/ControlFrame.hpp` and `src/ControlFrame.cpp`

  Decode the 11-byte binary control frame sent by `XSmart_Car_LineFollower`.
  The frame contains `float speed_mps` and `uint16 servo_pulse_us`.

- `include/virtual_bridge/VehicleModel.hpp` and `src/VehicleModel.cpp`

  Maintain the virtual vehicle state. The model integrates the rear axle center
  and publishes the configured localization point in front of the rear axle.

- `include/virtual_bridge/RobotPositionJson.hpp` and `src/RobotPositionJson.cpp`

  Build UDP payloads compatible with `ArucoCalibCpp`:

  ```json
  {"type":"robot_position","pos":[x,0.160000,z],"euler":[0.0,yaw,0.0],"t_aruco_emit_ns":123}
  ```

- `src/virtual_aruco_bridge.cpp`

  CLI entry point. It starts the TCP control server, runs the model loop, and
  sends UDP `robot_position` packets.

- `tests/virtual_bridge_tests.cpp`

  Regression tests for control-frame decoding, servo mapping, bicycle-model
  integration, pose-point offset, and JSON output.

## Vehicle Model

The simulator uses a low-speed kinematic bicycle model.

Default parameters:

- rear axle center is the integration reference point
- front/rear wheelbase: `0.20 m`
- rear wheel track: `0.155 m`
- localization point: `0.075 m` in front of rear axle center
- servo neutral pulse: `1500 us`
- servo pulse span: `2000 us`, matching `500..2500 us` for `0..180 deg`
- max front-wheel steering angle: `36 deg`
- servo speed model: `0.16 s / 60 deg`
- default UDP pose height: `0.16 m`

This is suitable for:

- testing the full control and coordinate feedback loop
- checking whether path following logic converges
- checking steering polarity and heading conventions
- rough low-speed route simulation

It does not model:

- tire slip
- motor dead zone
- battery voltage effects
- servo linkage nonlinearity
- backlash
- wheel friction and ground contact details

For closer behavior, tune these parameters with real telemetry:

- `--speed-scale`
- `--speed-tau-s`
- `--max-accel-mps2`
- `--max-steering-deg`
- `--steering-sign`

## Build

On the board:

```bash
cd ~/Desktop/VirtualBridge
cmake -S . -B build
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

Build outputs:

- `build/virtual_aruco_bridge`: main executable
- `build/virtual_bridge_tests`: test executable

## Startup Order

### 1. Start setupUI / pose receiver

Start the board-side program that normally receives ArUco positioning data.
Confirm the receiver port. In the current tested setup, the working port is
`9005`:

```bash
ss -lunp | grep -E '9000|9003|9005|9991'
```

If a process is listening on `0.0.0.0:9005`, use `--udp-port 9005`.

### 2. Start VirtualBridge

Example that matched the tested board chain:

```bash
cd ~/Desktop/VirtualBridge

./build/virtual_aruco_bridge \
  --control-bind 127.0.0.1 \
  --control-port 8899 \
  --udp-host 127.0.0.1 \
  --udp-port 9005 \
  --initial-world-x-mm 315 \
  --initial-world-y-mm 1077 \
  --initial-heading-deg 350
```

Important:

- `--control-port 8899` must match `XSmart_Car_LineFollower`'s `control_bridge.port`.
- `--udp-host` and `--udp-port` must point to the board-side positioning receiver.
- Do not start `icar_socket_bridge` at the same time.

### 3. Start XSmart_Car_LineFollower

Normal control mode:

```bash
cd ~/Desktop/XSmart_Car_LineFollower
./build/xsmart_car -d
```

Debug view while still sending control frames:

```bash
cd ~/Desktop/XSmart_Car_LineFollower
./build/xsmart_car -d --debug --debug-control -w 8082
```

`--debug-control` is required when using `--debug`; otherwise the XSmart program disables bridge control output and VirtualBridge will receive no speed/servo commands.

## Initial Pose

Initial pose is configured with:

```bash
--initial-world-x-mm <mm>
--initial-world-y-mm <mm>
--initial-heading-deg <deg>
```

These values describe the localization point, not the rear axle center. The
localization point is the point `0.075 m` in front of the rear axle center.

Example:

```bash
--initial-world-x-mm 315 \
--initial-world-y-mm 1077 \
--initial-heading-deg 350
```

The first two values use ArUco field coordinates in millimeters. The heading is
the virtual vehicle front direction in the ArUco world XY plane.

If you copy `yaw_deg` from old `ArucoCalibCpp` raw pose logs, add `180 deg`
before passing it to `VirtualBridge`, because that raw yaw was computed from a
marker edge rather than from the virtual vehicle front direction.

Example:

```text
old ArUco raw yaw_deg = 170
VirtualBridge initial heading = 170 + 180 = 350
```

## Coordinate Output

By default, output mapping follows the tested `ArucoCalibCpp/config.yaml`
convention:

```text
robot_position.pos[0] = world_y_m
robot_position.pos[1] = height_m
robot_position.pos[2] = world_x_m
robot_position.euler[1] = heading_deg
```

Equivalent defaults:

```bash
--pos-x-source world_y
--pos-z-source world_x
--height-m 0.16
--yaw-offset-deg 0
--yaw-sign 1
```

If a receiver expects a different coordinate convention, override these at
startup.

## Control Input

VirtualBridge listens for the same 11-byte control frame used by
`icar_socket_bridge`:

```text
byte 0       0x42
byte 1       address = 1
byte 2       length = 10
byte 3..6    float32 little-endian speed_mps
byte 7..8    uint16 little-endian servo_pulse_us
byte 9       checksum over bytes 0..8
byte 10      unused / reserved by original frame layout
```

When the link is working, VirtualBridge prints lines like:

```text
[control] client connected
[control] speed=0.000 servo=1684
[control] speed=1.026 servo=1542
```

`command_frames=0` in status output means no control frame has arrived yet.

## Common Commands

Show CLI options:

```bash
./build/virtual_aruco_bridge --help
```

Run with reversed steering polarity:

```bash
./build/virtual_aruco_bridge \
  --control-bind 127.0.0.1 \
  --control-port 8899 \
  --udp-host 127.0.0.1 \
  --udp-port 9005 \
  --initial-world-x-mm 315 \
  --initial-world-y-mm 1077 \
  --initial-heading-deg 350 \
  --steering-sign -1
```

Run with slower simulated speed:

```bash
./build/virtual_aruco_bridge \
  --control-bind 127.0.0.1 \
  --control-port 8899 \
  --udp-host 127.0.0.1 \
  --udp-port 9005 \
  --initial-world-x-mm 315 \
  --initial-world-y-mm 1077 \
  --initial-heading-deg 350 \
  --speed-scale 0.7
```

Run quietly:

```bash
./build/virtual_aruco_bridge \
  --control-bind 127.0.0.1 \
  --control-port 8899 \
  --udp-host 127.0.0.1 \
  --udp-port 9005 \
  --initial-world-x-mm 315 \
  --initial-world-y-mm 1077 \
  --initial-heading-deg 350 \
  --quiet
```

## Troubleshooting

### VirtualBridge says `command_frames=0`

`XSmart_Car_LineFollower` has not connected to `127.0.0.1:8899`.

Check:

```bash
ss -ltnp | grep 8899
```

Make sure the real `icar_socket_bridge` is not already using the same port.

If XSmart is running with `--debug`, start it with `--debug-control`.

### setupUI receives nothing

Check the UDP receiver port:

```bash
ss -lunp | grep -E '9000|9003|9005|9991'
```

Then make `--udp-port` match the actual receiver. In the tested chain, `9005`
is the working port.

### Virtual car moves backward

Most likely the initial heading is 180 degrees off.

If you copied a raw ArUco `yaw_deg`, add `180` before passing it as
`--initial-heading-deg`.

Example:

```text
wrong: --initial-heading-deg 170
right: --initial-heading-deg 350
```

### Virtual car turns opposite direction

Use:

```bash
--steering-sign -1
```

### Route tracking is directionally correct but not close enough

Tune:

```bash
--speed-scale
--speed-tau-s
--max-accel-mps2
--max-steering-deg
```

The current model is a low-speed kinematic model, not a full physical tire and
motor simulator.
