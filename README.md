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
├── config/
│   └── virtual_bridge.json
├── include/virtual_bridge/
│   ├── AppConfig.hpp
│   ├── ControlFrame.hpp
│   ├── RobotPositionJson.hpp
│   └── VehicleModel.hpp
├── src/
│   ├── AppConfig.cpp
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

- `include/virtual_bridge/AppConfig.hpp` and `src/AppConfig.cpp`

  Load runtime settings from `config/virtual_bridge.json`. The format is
  commented JSON, matching `XSmart_Car_LineFollower/config/xsmart_car.json`.

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
  integration, pose-point offset, JSON output, and config loading.

## Vehicle Model

The simulator uses a low-speed kinematic bicycle model.

Default parameters are stored in `config/virtual_bridge.json`:

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

- `vehicle.speed_scale`
- `vehicle.speed_tau_s`
- `vehicle.max_accel_mps2`
- `vehicle.max_steering_deg`
- `vehicle.steering_sign`

## Configuration

By default, `virtual_aruco_bridge` reads:

```text
config/virtual_bridge.json
```

The file is JSON with `//` comments, the same style as
`XSmart_Car_LineFollower/config/xsmart_car.json`.

Important sections:

- `control`: TCP endpoint used by `XSmart_Car_LineFollower` control frames.
- `udp`: UDP target that receives ArucoCalibCpp-compatible `robot_position`.
- `initial_pose`: initial localization point and vehicle heading.
- `vehicle`: wheelbase, rear track, pose offset, servo PWM mapping, speed model.
- `robot_position`: output coordinate mapping and yaw convention.
- `runtime`: logging behavior.

Current tested defaults:

```jsonc
"control": {
  "bind_ip": "127.0.0.1",
  "port": 8899
},
"udp": {
  "host": "127.0.0.1",
  "port": 9005,
  "send_hz": 30.0
},
"initial_pose": {
  "world_x_mm": 315.0,
  "world_y_mm": 1077.0,
  "heading_deg": 350.0
}
```

To switch hardware, edit the `vehicle` section:

```jsonc
"vehicle": {
  "wheelbase_m": 0.20,
  "rear_track_m": 0.155,
  "pose_offset_m": 0.075,
  "servo_mid_us": 1500,
  "servo_span_us": 2000.0,
  "max_steering_deg": 36.0
}
```

You can still override frequently changed values from the command line. CLI
arguments are applied after the config file is loaded.

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

If a process is listening on `0.0.0.0:9005`, set `udp.port` to `9005`.

### 2. Start VirtualBridge

With the default config, start VirtualBridge without long startup arguments:

```bash
cd ~/Desktop/VirtualBridge
./build/virtual_aruco_bridge
```

Important:

- `control.port` must match `XSmart_Car_LineFollower`'s `control_bridge.port`.
- `udp.host` and `udp.port` must point to the board-side positioning receiver.
- Do not start `icar_socket_bridge` at the same time.

For temporary changes, either pass a different config:

```bash
./build/virtual_aruco_bridge --config config/my_car.json
```

or override a few values:

```bash
./build/virtual_aruco_bridge --udp-port 9005 --initial-heading-deg 350
```

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

Initial pose is configured in `config/virtual_bridge.json`:

```jsonc
"initial_pose": {
  "world_x_mm": 315.0,
  "world_y_mm": 1077.0,
  "heading_deg": 350.0
}
```

These values describe the localization point, not the rear axle center. The
localization point is the point `vehicle.pose_offset_m` in front of the rear
axle center.

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

The same correction in config is:

```jsonc
"heading_deg": 350.0
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

```jsonc
"robot_position": {
  "height_m": 0.16,
  "pos_x_source": "world_y",
  "pos_x_sign": 1.0,
  "pos_z_source": "world_x",
  "pos_z_sign": 1.0,
  "yaw_offset_deg": 0.0,
  "yaw_sign": 1.0
}
```

If a receiver expects a different coordinate convention, edit these values or
override them at startup.

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
./build/virtual_aruco_bridge --steering-sign -1
```

Run with slower simulated speed:

```bash
./build/virtual_aruco_bridge --speed-scale 0.7
```

Run quietly:

```bash
./build/virtual_aruco_bridge --quiet
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

Then make `udp.port` match the actual receiver. In the tested chain, `9005` is
the working port.

### Virtual car moves backward

Most likely the initial heading is 180 degrees off.

If you copied a raw ArUco `yaw_deg`, add `180` before passing it as
`initial_pose.heading_deg`.

Example:

```text
wrong: "heading_deg": 170.0
right: "heading_deg": 350.0
```

### Virtual car turns opposite direction

Use:

```jsonc
"steering_sign": -1.0
```

### Route tracking is directionally correct but not close enough

Tune:

```text
vehicle.speed_scale
vehicle.speed_tau_s
vehicle.max_accel_mps2
vehicle.max_steering_deg
```

The current model is a low-speed kinematic model, not a full physical tire and
motor simulator.
