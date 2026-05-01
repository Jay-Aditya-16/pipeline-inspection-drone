"""
Autonomous Pipeline Inspection Drone
Raspberry Pi Dashboard — main entry point

Reads structured telemetry from ESP32 over UART, runs vision pipeline,
and renders a live 6-panel dashboard with web server for remote access.

Usage:
    python dashboard.py --port /dev/ttyUSB0 --baud 115200 --camera 0
"""

import argparse
import threading
import time
import re
import collections
import math

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial
from flask import Flask, Response

from vision import detect_pipe, detect_workers
from navigation import NavigationFusion
from safety import SafetyMonitor

# ── Constants ─────────────────────────────────────────────────────────────────
WINDOW_SAMPLES   = 30       # sliding window for trend regression
EMA_ALPHA        = 0.15     # smoothing factor (matches ESP32 firmware)
LEAK_THRESHOLDS  = {        # danger thresholds in ppm (0.5 * LEL where applicable)
    "LPG":     1000,
    "CH4":     2500,
    "H2":      2000,
    "CO":      200,
    "AMMONIA": 300,
    "ALCOHOL": 1000,
}
FRAME_SKIP       = 3        # run YOLO every Nth frame

# ── Shared state (thread-safe via locks) ──────────────────────────────────────
lock         = threading.Lock()
latest_frame = None          # annotated BGR frame for web stream
latest_dash  = None          # dashboard JPEG bytes for web stream

# ── Flask web server ──────────────────────────────────────────────────────────
app = Flask(__name__)

@app.route("/video")
def video_feed():
    def gen():
        while True:
            with lock:
                frame = latest_frame
            if frame is not None:
                _, jpg = cv2.imencode(".jpg", frame)
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + jpg.tobytes() + b"\r\n")
            time.sleep(0.05)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/dashboard")
def dashboard_feed():
    def gen():
        while True:
            with lock:
                data = latest_dash
            if data is not None:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + data + b"\r\n")
            time.sleep(0.1)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

# ── Packet parsing ────────────────────────────────────────────────────────────

def parse_packet(line: str) -> dict | None:
    """
    Parse a semicolon-delimited UART packet from the ESP32.
    Returns a dict of float values or None on parse failure.
    """
    try:
        pairs = re.findall(r"([A-Z0-9]+)=([-\d.]+)", line)
        if len(pairs) < 10:
            return None
        data = {k: float(v) for k, v in pairs}
        # Rename keys to friendly names
        return {
            "LPG":      data.get("M2",   0),
            "CH4":      data.get("M4",   0),
            "H2":       data.get("M8",   0),
            "CO":       data.get("M7",   0),
            "AMMONIA":  data.get("M135", 0),
            "ALCOHOL":  data.get("M3",   0),
            "PM25":     data.get("PM25", 0),
            "TEMP":     data.get("TEMP", 0),
            "HUM":      data.get("HUM",  0),
            "PRESS":    data.get("PRESS",0),
            "ALT":      data.get("ALT",  0),
            "AX":       data.get("AX",   0),
            "AY":       data.get("AY",   0),
            "AZ":       data.get("AZ",   0),
            "GX":       data.get("GX",   0),
            "GY":       data.get("GY",   0),
            "GZ":       data.get("GZ",   0),
        }
    except Exception:
        return None

# ── Trend forecasting ─────────────────────────────────────────────────────────

def trend_slope(distances: list[float], values: list[float]) -> float:
    """
    Linear regression slope via least squares.
    m = (N*sum(d*C) - sum(d)*sum(C)) / (N*sum(d^2) - sum(d)^2)
    """
    n = len(distances)
    if n < 2:
        return 0.0
    sd  = sum(distances)
    sc  = sum(values)
    sdc = sum(d * c for d, c in zip(distances, values))
    sd2 = sum(d * d for d in distances)
    denom = n * sd2 - sd * sd
    if abs(denom) < 1e-9:
        return 0.0
    return (n * sdc - sd * sc) / denom

# ── Dashboard renderer ────────────────────────────────────────────────────────

def render_dashboard(sensor_data: dict, map_points: list,
                     dist_history: list, leak_history: list,
                     slope: float, prediction: float,
                     status: str, camera_frame) -> bytes:
    """
    Renders a 2x3 matplotlib figure and returns JPEG bytes.
    Panels:
      [0,0] Camera feed with pipe/worker overlays
      [0,1] 3D leak map
      [1,0] Leak intensity vs distance (2D)
      [1,1] Sensor numeric dashboard
    """
    fig = plt.figure(figsize=(14, 8), facecolor="#0f0f0f")
    gs  = fig.add_gridspec(2, 2, hspace=0.4, wspace=0.3)

    STATUS_COLORS = {"SAFE": "#00cc66", "WARNING": "#ffaa00", "DANGER": "#ff3333"}
    color = STATUS_COLORS.get(status, "#ffffff")

    # ── Panel 1: Camera feed ─────────────────────────────────────────────
    ax_cam = fig.add_subplot(gs[0, 0])
    ax_cam.axis("off")
    if camera_frame is not None:
        rgb = cv2.cvtColor(camera_frame, cv2.COLOR_BGR2RGB)
        ax_cam.imshow(rgb)
    ax_cam.set_title(f"Leak Status: {status}  |  Trend: {slope:+.1f} ppm/m  |  Next 1m: {prediction:.1f}",
                     color=color, fontsize=9, pad=4)

    # ── Panel 2: 3D leak map ─────────────────────────────────────────────
    ax3d = fig.add_subplot(gs[0, 1], projection="3d")
    ax3d.set_facecolor("#0f0f0f")
    if map_points:
        xs, ys, zs = zip(*map_points)
        sc = ax3d.scatter(xs, ys, zs, c=zs, cmap="hot", s=6, alpha=0.8)
        fig.colorbar(sc, ax=ax3d, pad=0.1, shrink=0.6, label="Intensity")
    ax3d.set_xlabel("Distance (m)", fontsize=7, color="#aaaaaa")
    ax3d.set_ylabel("Altitude (cm)", fontsize=7, color="#aaaaaa")
    ax3d.set_zlabel("Leak Intensity", fontsize=7, color="#aaaaaa")
    ax3d.set_title("3D Leak Map", color="#cccccc", fontsize=9)
    ax3d.tick_params(colors="#555555", labelsize=6)

    # ── Panel 3: 2D trend chart ──────────────────────────────────────────
    ax2d = fig.add_subplot(gs[1, 0])
    ax2d.set_facecolor("#0f0f0f")
    if dist_history and leak_history:
        ax2d.plot(dist_history, leak_history, color="#4da6ff", linewidth=1.2,
                  marker="o", markersize=2)
        ax2d.fill_between(dist_history, leak_history, alpha=0.15, color="#4da6ff")
    ax2d.set_xlabel("Distance (m)", color="#aaaaaa", fontsize=8)
    ax2d.set_ylabel("Leak Intensity", color="#aaaaaa", fontsize=8)
    ax2d.set_title("Leak Intensity vs Distance", color="#cccccc", fontsize=9)
    ax2d.tick_params(colors="#555555")
    ax2d.spines[:].set_color("#333333")

    # ── Panel 4: Numeric sensor panel ───────────────────────────────────
    ax_num = fig.add_subplot(gs[1, 1])
    ax_num.axis("off")
    ax_num.set_facecolor("#0f0f0f")
    labels_left  = ["LPG", "ALCOHOL", "CH4", "CO", "H2", "AMMONIA", "PM25", "TEMP"]
    labels_right = ["HUM", "ALT", "AX", "AY", "AZ", "GX", "GY", "GZ"]
    y = 0.96
    for key in labels_left:
        val = sensor_data.get(key, 0)
        ax_num.text(0.02, y, f"{key}:", color="#888888", fontsize=8,
                    transform=ax_num.transAxes, va="top")
        ax_num.text(0.30, y, f"{val:.2f}", color="#ffffff", fontsize=8,
                    transform=ax_num.transAxes, va="top")
        y -= 0.115
    y = 0.96
    for key in labels_right:
        val = sensor_data.get(key, 0)
        ax_num.text(0.52, y, f"{key}:", color="#888888", fontsize=8,
                    transform=ax_num.transAxes, va="top")
        ax_num.text(0.78, y, f"{val:.3f}", color="#ffffff", fontsize=8,
                    transform=ax_num.transAxes, va="top")
        y -= 0.115
    ax_num.set_title("Sensor Dashboard", color="#cccccc", fontsize=9)

    fig.suptitle("Drone Leak Inspection — Camera + 3D Map + Dashboard",
                 color="#dddddd", fontsize=11)

    # Encode to JPEG bytes
    import io
    buf = io.BytesIO()
    fig.savefig(buf, format="jpeg", dpi=100, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close(fig)
    buf.seek(0)
    return buf.read()

# ── Main loop ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Pipeline Inspection Dashboard")
    parser.add_argument("--port",   default="/dev/ttyUSB0", help="ESP32 serial port")
    parser.add_argument("--baud",   default=115200, type=int)
    parser.add_argument("--camera", default=0, type=int, help="Camera device index")
    parser.add_argument("--web",    default=True,  action="store_true",
                        help="Start Flask web server on port 5000")
    args = parser.parse_args()

    # Start web server in background thread
    if args.web:
        web_thread = threading.Thread(
            target=lambda: app.run(host="0.0.0.0", port=5000, threaded=True),
            daemon=True)
        web_thread.start()
        print("Web server running at http://0.0.0.0:5000")

    # Open serial port
    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f"Connected to ESP32 on {args.port} at {args.baud} baud")

    # Open camera
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    nav     = NavigationFusion()
    safety  = SafetyMonitor(LEAK_THRESHOLDS)

    map_points    = []
    dist_history  = []
    leak_history  = []
    frame_count   = 0
    sensor_data   = {}

    print("Streaming. Press Ctrl+C to stop.")

    try:
        while True:
            # ── Read sensor packet ───────────────────────────────────────
            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
            except serial.SerialException:
                time.sleep(0.1)
                continue

            parsed = parse_packet(line)
            if parsed:
                sensor_data = parsed

            # ── Camera frame ─────────────────────────────────────────────
            ret, frame = cap.read()
            if not ret:
                continue

            frame_count += 1
            annotated = frame.copy()

            # Vision pipeline (pipe detection every frame, YOLO every Nth)
            annotated = detect_pipe(annotated)
            if frame_count % FRAME_SKIP == 0:
                annotated, worker_alert = detect_workers(annotated)
            else:
                worker_alert = False

            # ── Navigation update ────────────────────────────────────────
            imu = {k: sensor_data.get(k, 0) for k in ("AX","AY","AZ","GX","GY","GZ")}
            nav.update(frame, imu, sensor_data.get("ALT", 0))
            distance = nav.distance

            # ── Build 3D map point ───────────────────────────────────────
            leak_intensity = sensor_data.get("LPG", 0)
            altitude_cm    = sensor_data.get("ALT", 0) * 100
            map_points.append((distance, altitude_cm, leak_intensity))
            dist_history.append(distance)
            leak_history.append(leak_intensity)

            # Keep only last 200 points in history for display
            if len(dist_history) > 200:
                dist_history.pop(0)
                leak_history.pop(0)

            # ── Trend slope & prediction ─────────────────────────────────
            win_d = dist_history[-WINDOW_SAMPLES:]
            win_c = leak_history[-WINDOW_SAMPLES:]
            slope      = trend_slope(win_d, win_c)
            prediction = leak_intensity + slope * 1.0   # 1 m ahead

            # ── Safety status ────────────────────────────────────────────
            status = safety.classify(sensor_data, worker_alert)

            # ── Overlay status text on camera frame ──────────────────────
            status_colors = {"SAFE": (0,200,80), "WARNING": (0,170,255), "DANGER": (0,50,220)}
            sc = status_colors.get(status, (255,255,255))
            cv2.rectangle(annotated, (0, annotated.shape[0]-50),
                          (annotated.shape[1], annotated.shape[0]), sc, -1)
            cv2.putText(annotated,
                        f"Leak Status: {status}  (Intensity: {leak_intensity:.1f})",
                        (10, annotated.shape[0]-28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.putText(annotated,
                        f"Trend: {'rising' if slope>0 else 'falling'}  |  "
                        f"dLeakdm = {slope:.1f}  |  Next 1m: {prediction:.1f}",
                        (10, annotated.shape[0]-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220,220,220), 1)

            # ── Render dashboard ─────────────────────────────────────────
            dash_bytes = render_dashboard(
                sensor_data, map_points[-500:],
                dist_history, leak_history,
                slope, prediction, status, annotated)

            # ── Update shared state for web server ───────────────────────
            with lock:
                global latest_frame, latest_dash
                latest_frame = annotated
                latest_dash  = dash_bytes

            # ── Local display (optional) ─────────────────────────────────
            cv2.imshow("Pipeline Inspection", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
