# Sensors → ROS 2 (Docker) → Foxglove

| Sensor | Container | Topics |
|---|---|---|
| Yahboom 10-axis IMU | `imu` | `/imu/data` (Imu), `/imu/rpy` (deg), `/imu/pose`, `/imu/mag_raw`, `/imu/temperature`, `/imu/pressure`, `/imu/altitude` |
| YDLidar | `ydlidar` | `/scan` (LaserScan) |
| SHARP GP2Y0A21 on NodeMCU-32S (GPIO36) | `micro_ros_agent` | `/range_finder/range` (Range, m), `/range_finder/voltage` (V) |
| HotRC DS600 receiver on NodeMCU-32S (CH1 GPIO34, CH2 GPIO35, CH3 GPIO32) | `micro_ros_agent`, `rc_teleop` | `/rc/pulses` (µs) → `/cmd_vel_rc` |
| PC teleop (Foxglove Teleop panel, keyboard) | — | `/cmd_vel_pc` |
| — | `twist_mux` | `/cmd_vel_rc` (priority) + `/cmd_vel_pc` → `/cmd_vel` |
| DDSM115 via DDSM Driver HAT (A) | `ddsm115` | subscribes `/cmd_vel`, publishes `/wheel/joint_states` |
| Orbbec Gemini | `orbbec_camera` | `/camera/color/image_raw/compressed`, `/camera/depth/image_raw`, `/camera/depth/points`, `*/camera_info` |
| — | `foxglove_bridge` | WebSocket on `ws://localhost:8765` |

TF: `base_link → imu_link`, `base_link → laser_frame`, `base_link → ir_range_link`, `base_link → camera_link → camera_{depth,color}_optical_frame`.

## Run

1. Attach the USB devices to WSL (Windows PowerShell, admin): `usbipd list`, then `usbipd attach --wsl --busid <id>` for each sensor.
2. `./start.sh` (add `-d` to run in the background). It finds the ESP32 by its CH340 USB ID, the IMU by probing for its packets, uses the remaining `/dev/ttyUSB*` for the lidar, and starts only the sensors that are connected.
3. Foxglove → *Open connection* → *Foxglove WebSocket* → `ws://localhost:8765`.
   Import [docker/foxglove_layout.json](docker/foxglove_layout.json) via *Layouts → Import from file*.

Stop with `docker compose down`.

## Settings (`.env` or environment)

| Variable | Default | Meaning |
|---|---|---|
| `LIDAR_MODEL` | `ydlidar` | params file in `ydlidar/ydlidar_ros2_driver/params/` (e.g. `X4`, `G4`, `Tmini`) |
| `IMU_PORT` / `LIDAR_PORT` | auto | force a serial port |
| `IMU_BAUD` | `0` (auto) | IMU baud rate |
| `ESP32_PORT` | auto (CH340) | NodeMCU-32S serial port for the micro-ROS agent |
| `DDSM_PORT` | auto (CH343, `/dev/ttyACM*`) | HAT "ESP32-USB" port |
| `DDSM_IDS` | `1` | motor IDs, `1,2` = differential drive [left, right] |
| `DDSM_MAX_RPM` / `WHEEL_DIAMETER` | `60` / `0.10` | speed limit (DDSM115 max 200) and wheel diameter (m) |
| `RC_MAX_LINEAR` / `RC_ENABLE_CH` | `0.3` / `-1` | full-throttle speed (m/s); RC channel index that must be on to drive (`-1` = none) |
| `RC_THROTTLE_CENTER` / `RC_STEERING_CENTER` | `1500` | receiver pulse (µs) at rest; set from `/rc/pulses` |
| `RC_INVERT_THROTTLE` / `RC_INVERT_STEERING` | `false` | flip direction if the trigger drives backwards |
| `CLOUD_STRIDE` / `CLOUD_RATE` | `2` / `10.0` | point-cloud downsampling and rate (Hz) |

Debug: `docker compose logs -f imu`, `docker exec -it foxglove_bridge bash` → `ros2 topic list`.

## ESP32 firmware (firmware/, one PlatformIO project)

[platformio.ini](platformio.ini) at the repo root, so the PlatformIO VS Code extension picks it
up when you open this folder. Sources are in `firmware/`; the RC pulse reader is shared as
`firmware/lib/rc_input`.

| Environment | Board firmware | Command |
|---|---|---|
| `car_io` (default) | range finder (GPIO36) + RC receiver (GPIO34/35/32) → micro-ROS | `pio run -e car_io -t upload` |
| `rc_test` | prints receiver pulse widths over Serial | `pio run -e rc_test -t upload && pio device monitor` |
| `pin_scan` | reports which GPIO carries RC PWM | `pio run -e pin_scan -t upload && pio device monitor` |

Stop the Docker stack before flashing (the agent holds the serial port), and flash `car_io`
again after using a diagnostic. In `car_io` the on-board LED blinks while waiting for the agent
and stays on once connected. Distance = 27.728·V^-1.2045 cm; values outside 10–80 cm are
published as ±Inf (REP 117).

## Wheels (remote_control/)

DS600 → receiver PWM → NodeMCU-32S `/rc/pulses` → `rc_teleop` → `/cmd_vel_rc` ┐
PC: Foxglove Teleop panel or keyboard → `/cmd_vel_pc` ┴→ `twist_mux` → `/cmd_vel` → `ddsm115` →
HAT JSON over USB → DDSM115. The remote has priority while its sticks are moved and hands
control back to the PC 0.3 s after release.

Keyboard: `docker compose run --rm rc_teleop ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_pc -p speed:=0.15`
 Safety layers: no pulse for 100 ms on the ESP32 → 0;
no `/rc/pulses` for 0.3 s → zero `/cmd_vel`; no `/cmd_vel` for 0.5 s → 0 rpm; HAT heartbeat
stops the motor if the PC stops sending for 500 ms.
