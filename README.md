# TAMR

| Directory | Contents |
|---|---|
| [sensor/](sensor/README.md) | Sensors and chassis: IMU, YDLidar, Orbbec depth camera, SHARP rangefinder, HotRC RC receiver, DDSM115 wheels → ROS 2 (Docker) → Foxglove; NodeMCU-32S firmware (PlatformIO) |

## Getting the code

`sensor/` depends on three upstream packages managed as git submodules, so fetch them when you clone:

```bash
git clone --recurse-submodules git@github.com:Steven0811/TAMR.git
# Already cloned without submodules:
git submodule update --init --recursive
```

| Submodule | Upstream |
|---|---|
| `sensor/ydlidar/ydlidar_ros2_driver` | [YDLIDAR/ydlidar_ros2_driver](https://github.com/YDLIDAR/ydlidar_ros2_driver) (`humble` branch) |
| `sensor/ydlidar/YDLidar-SDK` | [YDLIDAR/YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK) |
| `sensor/depth_camera/third_party/pyorbbecsdk` | [orbbec/pyorbbecsdk](https://github.com/orbbec/pyorbbecsdk) (`v1.3.4`) |

See the README in each subdirectory for how to use that subsystem.
