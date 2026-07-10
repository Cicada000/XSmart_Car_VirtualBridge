# VirtualBridge

`VirtualBridge` replaces the physical lower-machine bridge during virtual tests.
It accepts the same TCP control frames that `XSmart_Car_LineFollower` sends to
`icar_socket_bridge`, integrates a virtual car pose, and publishes UDP
`robot_position` messages compatible with `ArucoCalibCpp`.

## Model

The simulator uses a kinematic bicycle model:

- rear axle center is the integration reference point
- front/rear wheelbase defaults to `0.20 m`
- rear wheel track defaults to `0.155 m`
- published localization point is `0.075 m` forward from the rear axle center
- servo neutral pulse defaults to `1500 us`
- servo pulse range defaults to `500..2500 us` for `0..180 deg`
- front wheel steering angle is clamped by `--max-steering-deg`, default `36 deg`

This is accurate enough for closed-loop control, coordinate-chain testing, and
turning trend validation at low speed. It does not model tire slip, backlash,
servo linkage nonlinearity, motor dead zone, battery sag, or surface friction.
Use real telemetry later to tune `--speed-scale`, `--speed-tau-s`,
`--max-accel-mps2`, `--max-steering-deg`, and `--steering-sign`.

## Build

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## Run

Example using the original ArUco UDP target from `ArucoCalibCpp/config.yaml`:

```bash
./build/virtual_aruco_bridge \
  --control-bind 127.0.0.1 \
  --control-port 8899 \
  --udp-host 192.168.8.123 \
  --udp-port 9005 \
  --initial-world-x-mm 315 \
  --initial-world-y-mm 1077 \
  --initial-heading-deg 170
```

Then run `XSmart_Car_LineFollower` normally. Do not run the real
`icar_socket_bridge` on the same port at the same time.

## Output Protocol

Each UDP packet is:

```json
{"type":"robot_position","pos":[x,0.160000,z],"euler":[0.0,yaw,0.0],"t_aruco_emit_ns":123}
```

By default, output coordinate mapping follows `ArucoCalibCpp/config.yaml`:

- `pos[0] = world_y_mm * 0.001`
- `pos[2] = world_x_mm * 0.001`
- `euler[1] = 180 + heading_deg`

The simulator stores coordinates internally in meters. Initial pose arguments
are in millimeters to match ArUco field coordinates.

Run `./build/virtual_aruco_bridge --help` for all model and protocol options.
