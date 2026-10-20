#!/usr/bin/env python3
"""Live viewer for the Yahboom 10-axis IMU (playrobot CSM0015, CP210x USB serial).

Protocol (WitMotion-compatible): 11-byte packets
    0x55 | type | 8 data bytes (4 x int16 little-endian) | checksum (sum of first 10 bytes & 0xFF)

Usage:
    python3 imu_viewer.py                    # terminal live display, auto-detect baud
    python3 imu_viewer.py --plot             # real-time plots (needs matplotlib)
    python3 imu_viewer.py --csv log.csv      # also log to CSV
    python3 imu_viewer.py --port /dev/ttyUSB0 --baud 9600
"""
import argparse
import csv
import struct
import sys
import threading
import time

import serial

BAUDS = [9600, 115200, 230400, 460800, 921600, 57600, 38400, 19200, 4800]
G = 9.80665


class ImuParser:
    def __init__(self):
        self.buf = bytearray()
        self.data = {
            "acc": [0.0] * 3,      # g
            "gyro": [0.0] * 3,     # deg/s
            "angle": [0.0] * 3,    # roll, pitch, yaw (deg)
            "mag": [0] * 3,        # raw LSB
            "quat": [1.0, 0.0, 0.0, 0.0],
            "temp": 0.0,           # deg C
            "pressure": 0,         # Pa
            "height": 0.0,         # m
            "chip_time": "",
        }
        self.seen = set()
        self.packets = 0
        self.bad = 0

    def feed(self, chunk):
        """Feed raw bytes; returns the list of packet types decoded (empty if none)."""
        self.buf += chunk
        kinds = []
        while len(self.buf) >= 11:
            if self.buf[0] != 0x55:
                del self.buf[0]
                continue
            pkt = bytes(self.buf[:11])
            if (sum(pkt[:10]) & 0xFF) != pkt[10]:
                self.bad += 1
                del self.buf[0]
                continue
            del self.buf[:11]
            self._decode(pkt[1], pkt[2:10])
            kinds.append(pkt[1])
        self.packets += len(kinds)
        return kinds

    def _decode(self, kind, p):
        d = self.data
        s = struct.unpack("<hhhh", p)
        self.seen.add(kind)
        if kind == 0x50:
            yy, mm, dd, hh, mi, ss = p[0], p[1], p[2], p[3], p[4], p[5]
            ms = p[6] | (p[7] << 8)
            d["chip_time"] = f"20{yy:02d}-{mm:02d}-{dd:02d} {hh:02d}:{mi:02d}:{ss:02d}.{ms:03d}"
        elif kind == 0x51:
            d["acc"] = [v / 32768.0 * 16.0 for v in s[:3]]
            d["temp"] = s[3] / 100.0
        elif kind == 0x52:
            d["gyro"] = [v / 32768.0 * 2000.0 for v in s[:3]]
        elif kind == 0x53:
            d["angle"] = [v / 32768.0 * 180.0 for v in s[:3]]
        elif kind == 0x54:
            d["mag"] = list(s[:3])
        elif kind == 0x56:
            pressure, height = struct.unpack("<ii", p)
            d["pressure"] = pressure
            d["height"] = height / 100.0
        elif kind == 0x59:
            d["quat"] = [v / 32768.0 for v in s]


def open_port(port, baud):
    try:
        return serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        if "Permission denied" in str(e):
            sys.exit(
                f"Cannot open {port}: permission denied.\n"
                f"Run: sudo usermod -aG dialout $USER  (then log in again / wsl --shutdown)\n"
                f"Or temporarily: sudo chmod 666 {port}"
            )
        sys.exit(f"Cannot open {port}: {e}")


def detect_baud(port):
    for baud in BAUDS:
        ser = open_port(port, baud)
        ser.reset_input_buffer()
        parser = ImuParser()
        t0 = time.time()
        while time.time() - t0 < 1.5:
            if parser.feed(ser.read(256)) and parser.packets >= 3:
                print(f"Detected baud rate {baud}")
                return ser
        ser.close()
        print(f"  {baud}: no valid data")
    sys.exit("No valid IMU data found. Check the connection and port.")


def reader_loop(ser, parser, lock, stop, csv_writer):
    last_log = 0.0
    while not stop.is_set():
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        with lock:
            if parser.feed(chunk) and csv_writer:
                now = time.time()
                if now - last_log >= 0.005:
                    d = parser.data
                    csv_writer.writerow(
                        [f"{now:.3f}", *d["acc"], *d["gyro"], *d["angle"], *d["mag"],
                         *d["quat"], d["temp"], d["pressure"], d["height"]]
                    )
                    last_log = now


def fmt3(v, f="{:9.3f}"):
    return " ".join(f.format(x) for x in v)


def run_terminal(parser, lock, stop):
    sys.stdout.write("\x1b[2J")
    last_count, last_t, rate = 0, time.time(), 0.0
    try:
        while True:
            time.sleep(0.1)
            with lock:
                d = {k: (list(v) if isinstance(v, list) else v) for k, v in parser.data.items()}
                count, bad, seen = parser.packets, parser.bad, sorted(parser.seen)
            now = time.time()
            if now - last_t >= 1.0:
                rate = (count - last_count) / (now - last_t)
                last_count, last_t = count, now
            lines = [
                "Yahboom 10-axis IMU  (Ctrl+C to quit)",
                "-" * 60,
                f"Accel     (g)        : {fmt3(d['acc'])}",
                f"          (m/s^2)    : {fmt3([a * G for a in d['acc']])}",
                f"Gyro      (deg/s)    : {fmt3(d['gyro'])}",
                f"Angle Roll/Pitch/Yaw: {fmt3(d['angle'])}",
                f"Mag       (raw)      : {fmt3(d['mag'], '{:9d}')}",
                f"Quat      (w,x,y,z)  : {fmt3(d['quat'], '{:8.4f}')}",
                f"Temperature          : {d['temp']:.2f} °C",
                f"Pressure / Altitude  : {d['pressure']} Pa  /  {d['height']:.2f} m",
                f"Chip time            : {d['chip_time']}",
                "-" * 60,
                f"Packets: {count}  ({rate:.0f}/s)  Checksum errors: {bad}  Types: "
                + " ".join(f"0x{k:02X}" for k in seen),
            ]
            sys.stdout.write("\x1b[H" + "\n".join(l + "\x1b[K" for l in lines) + "\n")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass


def run_plot(parser, lock, stop, window=10.0):
    import collections
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    hist = collections.deque()
    groups = [("acc", "Acceleration (g)"), ("gyro", "Angular velocity (deg/s)"),
              ("angle", "Angle (deg)"), ("mag", "Magnetic field (raw)")]
    labels = {"angle": ["roll", "pitch", "yaw"]}
    fig, axes = plt.subplots(len(groups), 1, figsize=(10, 9), sharex=True)
    lines = {}
    for ax, (key, title) in zip(axes, groups):
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.3)
        for i, name in enumerate(labels.get(key, ["x", "y", "z"])):
            (lines[(key, i)],) = ax.plot([], [], label=name)
        ax.legend(loc="upper left", fontsize=8)
    axes[-1].set_xlabel("time (s)")
    t0 = time.time()

    def update(_):
        now = time.time() - t0
        with lock:
            hist.append((now, {k: list(parser.data[k]) for k, _ in groups}))
        while hist and now - hist[0][0] > window:
            hist.popleft()
        ts = [h[0] for h in hist]
        for ax, (key, _) in zip(axes, groups):
            for i in range(3):
                lines[(key, i)].set_data(ts, [h[1][key][i] for h in hist])
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(max(0, now - window), max(window, now))
        return list(lines.values())

    _anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    fig.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(description="Yahboom 10-axis IMU viewer")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, help="auto-detect if omitted")
    ap.add_argument("--plot", action="store_true", help="real-time plot with matplotlib")
    ap.add_argument("--csv", help="log data to a CSV file")
    args = ap.parse_args()

    ser = open_port(args.port, args.baud) if args.baud else detect_baud(args.port)

    csv_file = csv_writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["time", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
                             "roll", "pitch", "yaw", "mx", "my", "mz",
                             "qw", "qx", "qy", "qz", "temp_c", "pressure_pa", "height_m"])

    parser, lock, stop = ImuParser(), threading.Lock(), threading.Event()
    th = threading.Thread(target=reader_loop, args=(ser, parser, lock, stop, csv_writer), daemon=True)
    th.start()
    try:
        (run_plot if args.plot else run_terminal)(parser, lock, stop)
    finally:
        stop.set()
        th.join(timeout=1)
        ser.close()
        if csv_file:
            csv_file.close()
        print("\nClosed.")


if __name__ == "__main__":
    main()
