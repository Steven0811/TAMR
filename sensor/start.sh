#!/bin/bash
# Detect connected sensors, then start their containers + foxglove_bridge.
#   ./start.sh            build (if needed) and run in the foreground
#   ./start.sh -d         run detached
# Override detection with IMU_PORT / LIDAR_PORT / ESP32_PORT / DDSM_PORT / LIDAR_MODEL (env or .env).
set -e
cd "$(dirname "$0")"
[ -f .env ] && set -a && source .env && set +a

ports=$(ls /dev/ttyUSB* 2>/dev/null || true)
usb_id() {  # "vid:pid" of a tty
    udevadm info -q property -n "$1" 2>/dev/null |
        awk -F= '/^ID_VENDOR_ID=/{v=$2} /^ID_MODEL_ID=/{p=$2} END{print v":"p}'
}

# ESP32 (NodeMCU-32S, CH340 = 1a86:7523). Found by USB ID, never probed: opening the
# port toggles DTR/RTS and resets the board.
if [ -z "$ESP32_PORT" ]; then
    for p in $ports; do
        [ "$(usb_id "$p")" = "1a86:7523" ] && ESP32_PORT=$p && break
    done
fi
# DDSM Driver HAT (A): CH343 (1a86:55d3) shows up as /dev/ttyACM*. Use its
# "ESP32-USB" Type-C port; the other one bypasses the ESP32.
if [ -z "$DDSM_PORT" ]; then
    for p in $(ls /dev/ttyACM* 2>/dev/null); do
        [ "$(usb_id "$p")" = "1a86:55d3" ] && DDSM_PORT=$p && break
    done
fi

sensor_ports=""
for p in $ports; do [ "$p" != "$ESP32_PORT" ] && sensor_ports="$sensor_ports $p"; done

# IMU: the port that emits WitMotion packets (0x55 ...)
if [ -z "$IMU_PORT" ]; then
    for p in $sensor_ports; do
        if python3 - "$p" <<'EOF'
import sys, time, serial
sys.path.insert(0, "imu")
from imu_viewer import ImuParser
for baud in (9600, 115200, 230400, 460800, 921600):
    try:
        s = serial.Serial(sys.argv[1], baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"WARNING: cannot probe {sys.argv[1]}: {e}", file=sys.stderr)
        sys.exit(1)
    s.reset_input_buffer()
    parser, t0 = ImuParser(), time.time()
    while time.time() - t0 < 1.0:
        parser.feed(s.read(256))
        if parser.packets >= 3:
            sys.exit(0)
    s.close()
sys.exit(1)
EOF
        then IMU_PORT=$p; break; fi
    done
fi

# Lidar: first remaining serial port
if [ -z "$LIDAR_PORT" ]; then
    for p in $sensor_ports; do
        [ "$p" != "$IMU_PORT" ] && LIDAR_PORT=$p && break
    done
fi

services=(foxglove)
[ -n "$IMU_PORT" ]   && [ -e "$IMU_PORT" ]   && services+=(imu)
[ -n "$LIDAR_PORT" ] && [ -e "$LIDAR_PORT" ] && services+=(ydlidar)
lsusb 2>/dev/null | grep -qi "2bc5:" && services+=(camera)   # Orbbec VID
[ -n "$ESP32_PORT" ] && [ -e "$ESP32_PORT" ] && services+=(micro_ros_agent range_tf rc_teleop)
[ -n "$DDSM_PORT" ]  && [ -e "$DDSM_PORT" ]  && services+=(ddsm115 twist_mux)

echo "IMU   : ${IMU_PORT:-not found}"
echo "Lidar : ${LIDAR_PORT:-not found} (model: ${LIDAR_MODEL:-ydlidar})"
echo "Camera: $([[ " ${services[*]} " == *" camera "* ]] && echo found || echo 'not found')"
echo "ESP32 : ${ESP32_PORT:-not found} (micro-ROS range finder + RC receiver)"
echo "DDSM  : ${DDSM_PORT:-not found} (DDSM Driver HAT, wheels ${DDSM_IDS:-1})"
echo "Starting: ${services[*]}"
echo "Foxglove: connect to ws://localhost:8765"

export IMU_PORT LIDAR_PORT ESP32_PORT DDSM_PORT
exec docker compose up --build "$@" "${services[@]}"
