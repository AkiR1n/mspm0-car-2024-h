#!/usr/bin/env python3
"""IMU real-time monitor for MSPM0 School 2026.

Connects to serial, enables IMU-only mode, and plots yaw / gyro / accel in real time.

Usage:
    python tools/imu_monitor.py [port] [baud]

Default: /dev/ttyACM0 @ 115200 (wired J-Link VCOM)
BT:      python tools/imu_monitor.py /tmp/vBT24 9600
"""

import sys
import re
import time
import argparse
from collections import deque

import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── Config ────────────────────────────────────────────────────────
WINDOW_POINTS = 300          # history points shown
PLOT_INTERVAL_MS = 50        # matplotlib update interval
DATA_TIMEOUT_S = 2.0         # warn if no data for this long
MAX_YAW_LINES = 2            # how many full rotations before yaw line wraps

# ── Parse IMU line ────────────────────────────────────────────────
_RE_IMU = re.compile(r"imu:\s+")
_RE_FIELD = re.compile(r"(\w+)=([-\d.]+)")

def parse_imu_line(line: str) -> dict | None:
    if not _RE_IMU.match(line):
        return None
    return dict(_RE_FIELD.findall(line))

# ── Data buffers ──────────────────────────────────────────────────
class ImuData:
    def __init__(self, maxlen=WINDOW_POINTS):
        self.t       = deque(maxlen=maxlen)
        self.yaw     = deque(maxlen=maxlen)
        self.gz      = deque(maxlen=maxlen)
        self.gx      = deque(maxlen=maxlen)
        self.gy      = deque(maxlen=maxlen)
        self.ax      = deque(maxlen=maxlen)
        self.ay      = deque(maxlen=maxlen)
        self.az      = deque(maxlen=maxlen)
        self.pitch   = deque(maxlen=maxlen)
        self.roll    = deque(maxlen=maxlen)
        self.rdy     = deque(maxlen=maxlen)
        self.stb     = deque(maxlen=maxlen)
        self.sign    = deque(maxlen=maxlen)
        self.sens    = deque(maxlen=maxlen)
        self.up      = deque(maxlen=maxlen)
        self.sms     = deque(maxlen=maxlen)

    def append(self, fields: dict):
        self.t.append(time.time())
        self.yaw.append(float(fields.get("yaw", 0)))
        self.gz.append(float(fields.get("gz", 0)))
        self.gx.append(float(fields.get("gx", 0)))
        self.gy.append(float(fields.get("gy", 0)))
        self.ax.append(float(fields.get("ax", 0)))
        self.ay.append(float(fields.get("ay", 0)))
        self.az.append(float(fields.get("az", 0)))
        self.pitch.append(float(fields.get("pitch", 0)))
        self.roll.append(float(fields.get("roll", 0)))
        self.rdy.append(int(fields.get("rdy", 0)))
        self.stb.append(int(fields.get("stb", 0)))
        self.sign.append(float(fields.get("sign", 0)))
        self.sens.append(float(fields.get("sens", 0)))
        self.up.append(float(fields.get("up", 0)))
        self.sms.append(float(fields.get("sms", 0)))

# ── Serial reader ─────────────────────────────────────────────────
class SerialReader:
    def __init__(self, port: str, baud: int, auto_imu: bool = True):
        self.data = ImuData()
        self._ser = serial.Serial(port, baud, timeout=0.1)
        self._buf = b""
        self._last_line_time = time.time()
        self._imu_active = False

        if auto_imu:
            # Wait for connection to settle, then send 'imu' with retries
            warmup = 3.0 if baud <= 9600 else 0.5
            print(f"Warming up ({warmup:.0f}s)...")
            time.sleep(warmup)
            self._enable_imu_mode()

    def _enable_imu_mode(self, retries: int = 5):
        """Send 'imu,<period>' command and wait for data."""
        baud = self._ser.baudrate
        period_ms = 200 if baud <= 9600 else 50
        cmd = f"imu,{period_ms}\r\n".encode()
        # Slow links need more time for ack to arrive
        wait_s = 1.5 if baud <= 9600 else 0.3
        print(f"IMU period: {period_ms}ms ({1000/period_ms:.0f} Hz)")

        for i in range(retries):
            self._ser.write(cmd)
            self._drain_buffer(check_imu_ack=True)
            # Wait and drain again for late-arriving data
            time.sleep(wait_s)
            self._drain_buffer(check_imu_ack=True)
            if self._imu_active or (len(self.data.t) > 0):
                print(f"IMU mode active ({len(self.data.t)} pts buffered)")
                return
            if i < retries - 1:
                print(f"  retry ({i+2}/{retries})...")
                time.sleep(1.0)
        print("Warning: no IMU data received (check connection)")

    def _drain_buffer(self, check_imu_ack: bool = False):
        """Read available data without blocking long."""
        try:
            chunk = self._ser.read(256)
            if chunk:
                self._buf += chunk
                while b"\n" in self._buf:
                    line_bytes, self._buf = self._buf.split(b"\n", 1)
                    line = line_bytes.decode("ascii", errors="replace").strip()
                    if check_imu_ack and "imu=on" in line:
                        self._imu_active = True
                    fields = parse_imu_line(line)
                    if fields:
                        self.data.append(fields)
                        self._last_line_time = time.time()
        except (OSError, serial.SerialException):
            pass

    def read_all(self):
        """Call from main thread. Returns number of new lines parsed."""
        count = len(self.data.t)
        self._drain_buffer()
        return len(self.data.t) - count

    def stale_sec(self) -> float:
        return time.time() - self._last_line_time

    def close(self):
        try:
            self._ser.write(b"imu\r\n")
            time.sleep(0.05)
        except Exception:
            pass
        try:
            self._ser.close()
        except Exception:
            pass

# ── Plot ──────────────────────────────────────────────────────────
class ImuPlot:
    def __init__(self, reader: SerialReader):
        self.reader = reader

        plt.rcParams.update({
            "figure.facecolor": "#1a1a2e",
            "axes.facecolor": "#16213e",
            "axes.edgecolor": "#555",
            "axes.labelcolor": "#ccc",
            "text.color": "#ccc",
            "xtick.color": "#999",
            "ytick.color": "#999",
            "grid.color": "#333",
            "legend.facecolor": "#1a1a2e",
            "legend.edgecolor": "#555",
        })

        self.fig = plt.figure("IMU Monitor — MSPM0 School 2026", figsize=(12, 9))
        self.fig.set_tight_layout(True)

        gs = self.fig.add_gridspec(3, 3, hspace=0.35, wspace=0.35)

        # ── Yaw angle ──
        self.ax_yaw = self.fig.add_subplot(gs[0, :2])
        self.ax_yaw.set_ylabel("Yaw (deg)")
        self.ax_yaw.set_title("Yaw Angle", color="#ff9800", fontweight="bold")
        self.ax_yaw.grid(True, alpha=0.4)
        self.line_yaw, = self.ax_yaw.plot([], [], color="#ff9800", linewidth=1.2)

        # ── Gyro Z ──
        self.ax_gz = self.fig.add_subplot(gs[1, :2])
        self.ax_gz.set_ylabel("Gyro Z (dps)")
        self.ax_gz.set_title("Gyroscope Z", color="#4fc3f7", fontweight="bold")
        self.ax_gz.grid(True, alpha=0.4)
        self.ax_gz.axhline(y=0, color="#555", linestyle="--", linewidth=0.8)
        self.line_gz, = self.ax_gz.plot([], [], color="#4fc3f7", linewidth=1.0)

        # ── Accel ──
        self.ax_acc = self.fig.add_subplot(gs[2, :2])
        self.ax_acc.set_ylabel("Accel (raw LSB)")
        self.ax_acc.set_xlabel("Time (s)")
        self.ax_acc.set_title("Accelerometer", color="#81c784", fontweight="bold")
        self.ax_acc.grid(True, alpha=0.4)
        self.line_ax, = self.ax_acc.plot([], [], color="#ef5350", linewidth=0.8, label="ax")
        self.line_ay, = self.ax_acc.plot([], [], color="#66bb6a", linewidth=0.8, label="ay")
        self.line_az, = self.ax_acc.plot([], [], color="#42a5f5", linewidth=0.8, label="az")
        self.ax_acc.legend(loc="upper right", fontsize=7)

        # ── Compass (polar) ──
        self.ax_compass = self.fig.add_subplot(gs[0, 2], projection="polar")
        self.ax_compass.set_title("Heading", color="#ff9800", fontweight="bold", fontsize=9)
        self.ax_compass.set_theta_zero_location("N")
        self.ax_compass.set_theta_direction(-1)
        self.ax_compass.set_xticks(np.radians([0, 45, 90, 135, 180, 225, 270, 315]))
        self.ax_compass.set_xticklabels(["N", "NE", "E", "SE", "S", "SW", "W", "NW"], fontsize=7)
        self.ax_compass.set_ylim(0, 1)
        self.ax_compass.set_yticks([])
        self.line_compass, = self.ax_compass.plot([], [], color="#ff9800", linewidth=2.5)
        self.arrow_compass, = self.ax_compass.plot([], [], color="#ff5722", linewidth=2.5, marker="o", markersize=4)

        # ── Status text ──
        self.ax_status = self.fig.add_subplot(gs[1:, 2])
        self.ax_status.axis("off")
        self.status_text = self.ax_status.text(
            0.05, 0.95, "", transform=self.ax_status.transAxes,
            fontfamily="monospace", fontsize=9, color="#ccc",
            verticalalignment="top",
        )

        # ── Gyro X/Y ──
        self.ax_gxy = self.fig.add_subplot(gs[1, 2])
        self.ax_gxy.set_title("Gyro X/Y", color="#4fc3f7", fontweight="bold", fontsize=9)
        self.ax_gxy.grid(True, alpha=0.4)
        self.ax_gxy.axhline(y=0, color="#555", linestyle="--", linewidth=0.8)
        self.line_gx, = self.ax_gxy.plot([], [], color="#ff7043", linewidth=0.8, label="gx")
        self.line_gy, = self.ax_gxy.plot([], [], color="#ab47bc", linewidth=0.8, label="gy")
        self.ax_gxy.legend(loc="upper right", fontsize=6)

        # data storage for relative time
        self._t0: float | None = None

    def _rel_time(self, data: ImuData) -> list[float]:
        if self._t0 is None and data.t:
            self._t0 = data.t[0]
        t0 = self._t0 or 0.0
        return [t - t0 for t in data.t]

    def _update_ylim(self, ax, values, margin=0.15):
        if len(values) < 2:
            return
        mn, mx = min(values), max(values)
        rng = mx - mn
        if rng < 1e-6:
            rng = 1.0
        ax.set_ylim(mn - rng * margin, mx + rng * margin)

    def animate(self, frame):
        reader = self.reader
        reader.read_all()
        data = reader.data

        t = self._rel_time(data)
        if len(t) < 1:
            return []

        # ── Yaw ──
        self.line_yaw.set_data(t, data.yaw)
        yaw_arr = np.array(data.yaw)
        if len(yaw_arr) > 1:
            self.ax_yaw.set_ylim(yaw_arr.min() - 3, yaw_arr.max() + 3)
        self.ax_yaw.set_xlim(max(0, t[-1] - 15), max(t[-1], 15) + 1)

        # ── Gyro Z ──
        self.line_gz.set_data(t, data.gz)
        gz_arr = np.array(data.gz)
        if len(gz_arr) > 1:
            gz_range = max(abs(gz_arr.min()), abs(gz_arr.max()), 1.0) * 1.3
            self.ax_gz.set_ylim(-gz_range, gz_range)
        self.ax_gz.set_xlim(max(0, t[-1] - 15), max(t[-1], 15) + 1)

        # ── Accel ──
        self.line_ax.set_data(t, data.ax)
        self.line_ay.set_data(t, data.ay)
        self.line_az.set_data(t, data.az)
        all_acc = list(data.ax) + list(data.ay) + list(data.az)
        if len(all_acc) > 1:
            acc_min, acc_max = min(all_acc), max(all_acc)
            acc_range = max(acc_max - acc_min, 1.0) * 0.6
            mid = (acc_min + acc_max) / 2
            self.ax_acc.set_ylim(mid - acc_range, mid + acc_range)
        self.ax_acc.set_xlim(max(0, t[-1] - 15), max(t[-1], 15) + 1)

        # ── Gyro X/Y ──
        self.line_gx.set_data(t, data.gx)
        self.line_gy.set_data(t, data.gy)
        all_gxy = list(data.gx) + list(data.gy)
        if len(all_gxy) > 1:
            gxy_max = max(abs(min(all_gxy)), abs(max(all_gxy)), 1.0) * 1.3
            self.ax_gxy.set_ylim(-gxy_max, gxy_max)
        self.ax_gxy.set_xlim(max(0, t[-1] - 15), max(t[-1], 15) + 1)

        # ── Compass ──
        current_yaw = data.yaw[-1] if data.yaw else 0.0
        rad = np.radians(current_yaw)
        self.line_compass.set_data([0, rad], [0, 0.85])
        self.arrow_compass.set_data([rad], [0.9])

        # ── Status text ──
        stale = reader.stale_sec()
        color = "#f44336" if stale > DATA_TIMEOUT_S else "#ccc"
        lines = [
            f"Port: {reader._ser.port}",
            f"",
            f"yaw   {current_yaw:7.1f} deg",
            f"gz    {data.gz[-1] if data.gz else 0:7.2f} dps",
            f"gx    {data.gx[-1] if data.gx else 0:7.2f} dps",
            f"gy    {data.gy[-1] if data.gy else 0:7.2f} dps",
            f"ax    {data.ax[-1] if data.ax else 0:7.0f}",
            f"ay    {data.ay[-1] if data.ay else 0:7.0f}",
            f"az    {data.az[-1] if data.az else 0:7.0f}",
            f"pitch {data.pitch[-1] if data.pitch else 0:6.1f} deg",
            f"roll  {data.roll[-1] if data.roll else 0:6.1f} deg",
            f"",
            f"ready {'OK' if data.rdy and data.rdy[-1] else 'NO'}",
            f"stable {'OK' if data.stb and data.stb[-1] else '--'}",
            f"sign  {data.sign[-1] if data.sign else 0:.0f}",
            f"sens  {data.sens[-1] if data.sens else 0:.1f}",
            f"up    {data.up[-1] if data.up else 0:.0f} ms",
            f"sms   {data.sms[-1] if data.sms else 0:.0f} ms",
            f"",
            f"data {len(data.t)} pts",
            f"stale {stale:.1f}s" if stale > 0.1 else "",
        ]
        self.status_text.set_text("\n".join(lines))
        if reader.stale_sec() > DATA_TIMEOUT_S:
            self.status_text.set_color("#f44336")
        else:
            self.status_text.set_color("#ccc")

        return (self.line_yaw, self.line_gz, self.line_ax, self.line_ay, self.line_az,
                self.line_gx, self.line_gy, self.line_compass, self.arrow_compass,
                self.status_text)

# ── Main ──────────────────────────────────────────────────────────
def connect_serial(port: str, baud: int, auto_imu: bool, retry: bool) -> SerialReader:
    """Connect to serial port, with optional retry loop."""
    while True:
        print(f"Connecting to {port} @ {baud} ...")
        try:
            return SerialReader(port, baud, auto_imu=auto_imu)
        except (serial.SerialException, FileNotFoundError) as e:
            if retry:
                print(f"  {e}")
                print(f"  Waiting for port... (Ctrl+C to cancel)")
                time.sleep(2)
            else:
                print(f"ERROR: {e}")
                sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description="IMU real-time monitor")
    parser.add_argument("port", nargs="?", default="/dev/ttyACM0",
                        help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("baud", nargs="?", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--no-send-imu", action="store_true",
                        help="Don't send 'imu' command on connect")
    parser.add_argument("--retry", action="store_true", default=None,
                        help="Keep retrying if port not available")
    args = parser.parse_args()

    # Auto-enable retry for non-ACM ports (BT virtual ports etc.)
    retry = args.retry if args.retry is not None else ("ACM" not in args.port.upper())

    reader = connect_serial(args.port, args.baud,
                            auto_imu=not args.no_send_imu,
                            retry=retry)

    print("Starting GUI (close window to exit)...")

    plot = ImuPlot(reader)
    ani = animation.FuncAnimation(
        plot.fig, plot.animate,
        interval=PLOT_INTERVAL_MS,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    finally:
        reader.close()
        print("Disconnected.")

if __name__ == "__main__":
    main()
