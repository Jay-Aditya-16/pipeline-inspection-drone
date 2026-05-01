# Autonomous Pipeline Inspection & Worker Safety Monitoring System

An autonomous drone-based platform for real-time pipeline inspection, multi-gas leak detection, 3D spatial leak mapping, and confined-space worker safety monitoring. Built on a Raspberry Pi + ESP32 edge computing stack with YOLOv8 computer vision and GPS-denied navigation.

![Drone Leak Inspection Dashboard](docs/circuit/wiring_diagram.jpg)

---

## Features

- **Multi-gas detection** across 6 MQ-series sensors: LPG, methane, hydrogen, carbon monoxide, ammonia compounds, alcohol vapors
- **PM2.5 particulate**, temperature, humidity, barometric altitude, and full 6-DOF IMU
- **3D spatial leak map** — distance + altitude + gas intensity visualized in real time
- **Leak trend forecasting** — predicts gas concentration 1 meter ahead of the drone via sliding-window linear regression
- **GPS-denied navigation** — Lucas-Kanade optical flow + Mahony-filtered IMU dead reckoning + pipeline centerline alignment
- **YOLOv8 worker detection** — detects prone/unconscious workers in elevated-gas environments
- **Classical pipe detection** — Canny edge detection + probabilistic Hough transform for pipe tracking and centering
- **Live 6-panel dashboard** with annotated camera feed, 3D map, trend chart, numeric sensor panel, drone simulation, and heat-map mini-map
- **Remote web monitoring** via Flask server — any browser on the network can view the live dashboard

---

## Repository Structure

```
pipeline-inspection-drone/
├── firmware/
│   └── esp32/
│       └── main.cpp              # ESP32 sensor firmware (C++/Arduino)
├── dashboard/
│   ├── dashboard.py              # Main Raspberry Pi application
│   ├── vision.py                 # Pipe detection + YOLOv8 worker detection
│   ├── navigation.py             # GPS-denied navigation fusion + Mahony filter
│   └── safety.py                 # Rule-based safety alert classification
├── docs/
│   └── circuit/
│       └── WIRING.md             # Full pin-by-pin wiring reference
├── models/                       # Place your trained .pt files here (not tracked by git)
│   ├── worker_detection.pt
│   └── pipe_detection.pt
├── requirements.txt
├── LICENSE
└── README.md
```

---

## Hardware

| Component | Purpose |
|---|---|
| Raspberry Pi 3B | Edge compute — vision, navigation, dashboard, web server |
| ESP32 (WROOM-32) | Sensor hub — 10 Hz acquisition, UART telemetry |
| MQ-2, MQ-4, MQ-8 | Combustible gas detection (LPG, CH4, H2) |
| MQ-7, MQ-135, MQ-3 | Toxic gas detection (CO, NH3/NOx, alcohol) |
| GP2Y1010AU0F | PM2.5 particulate sensor |
| DHT11 | Temperature and humidity |
| BMP280 | Barometric pressure and altitude |
| MPU6050 | 3-axis accelerometer + gyroscope (IMU) |
| USB webcam | Downward-facing visual feed for pipe tracking and worker detection |

Total sensor payload cost: approximately $35–$45 USD.

---

## System Architecture

```
[Sensor Array]
  MQ-2/4/8/7/135/3
  GP2Y1010 · DHT11
  BMP280 · MPU6050
       |
       | Analog / I2C / Digital
       v
  [ESP32 MCU]
  10 Hz sampling
  EMA filtering
  Packet serialization
       |
       | UART 115200 baud
       v
  [Raspberry Pi 3B]
  ├── Sensor fusion + EMA
  ├── Leak trend regression
  ├── 3D map construction
  ├── Optical flow + Mahony navigation
  ├── YOLOv8 pipe + worker detection
  ├── Safety classification
  └── Dashboard rendering
       |
       | WiFi (TCP / Flask)
       v
  [Ground Station / Web Browser]
  Live dashboard, remote monitoring
```

---

## Sensor Model

MQ-series sensors follow a power-law concentration model:

$$C = a \cdot \left(\frac{R_S}{R_0}\right)^b$$

where $C$ is concentration in ppm, $R_S$ is the sensor resistance, $R_0$ is the baseline resistance in clean air, and $a$, $b$ are empirically derived constants. Perform a two-point calibration per sensor before deployment (see `firmware/esp32/main.cpp` for calibration constants).

EMA filtering applied at both firmware and software layers:

$$\hat{x}_t = \alpha \cdot x_t + (1 - \alpha)\,\hat{x}_{t-1}, \quad \alpha = 0.15$$

---

## Installation

### Raspberry Pi


```

Enable UART on the Raspberry Pi (add to `/boot/config.txt`):
```
enable_uart=1
dtoverlay=disable-bt
```

Disable the serial console:
```bash
sudo systemctl disable serial-getty@ttyS0.service
sudo reboot
```

### ESP32 Firmware

Open `firmware/esp32/main.cpp` in the Arduino IDE or PlatformIO.

Required libraries (install via Arduino Library Manager):
- `Adafruit BMP280 Library`
- `Adafruit MPU6050`
- `Adafruit Unified Sensor`
- `DHT sensor library`

Select board: **ESP32 Dev Module**, upload speed: **115200**.

---

## Usage

```bash
python dashboard/dashboard.py --port /dev/ttyS0 --baud 115200 --camera 0
```

| Argument | Default | Description |
|---|---|---|
| `--port` | `/dev/ttyUSB0` | Serial port for ESP32 |
| `--baud` | `115200` | UART baud rate |
| `--camera` | `0` | OpenCV camera device index |
| `--web` | `True` | Start Flask web server on port 5000 |

Access the remote dashboard at `http://<raspberry-pi-ip>:5000/dashboard`.

---

## YOLOv8 Models

Place your trained model weights in the `models/` directory:

```
models/
├── pipeline_tracking_yolov5s.pt   # Custom YOLOv5s trained for pipeline detection
└── worker_detection.pt            # YOLOv8n trained for human detection + posture
```

The `pipeline_tracking_yolov5s.pt` model is included in this repository (14 MB).
It was trained on a custom dataset of pipeline imagery and loaded via `torch.hub`.

To train your own worker detection model:

```bash
yolo train model=yolov8n.pt data=your_dataset.yaml epochs=100 imgsz=320
```

For INT8 quantization (recommended for Raspberry Pi deployment):

```bash
yolo export model=worker_detection.pt format=onnx int8=True
```

---

## Safety Alert Levels

| Status | Condition |
|---|---|
| SAFE | All gas concentrations below 50% of danger threshold |
| WARNING | Any gas concentration between 50% and 100% of threshold |
| DANGER | Any gas at or above threshold, OR prone/inactive worker detected |

Thresholds are configurable in `dashboard/dashboard.py` under `LEAK_THRESHOLDS`.

---

## Navigation

GPS-denied position estimation fuses three sources:

$$\vec{p}_t = w_1\,\vec{p}_{flow} + w_2\,\vec{p}_{IMU} + w_3\,\vec{p}_{pipe}$$

Weights: optical flow $w_1=0.6$, IMU $w_2=0.3$, pipe alignment $w_3=0.1$.

Attitude estimation uses the Mahony complementary filter to suppress IMU gyroscope drift. In bench testing this reduced positional error from 1.2 m to under 0.15 m over a 10-second interval compared to raw dead reckoning.

---

## Demo Results (live test)

| Gas | Reading | Status |
|---|---|---|
| LPG | 2896 ppm | DANGER |
| CH4 | 2415 ppm | DANGER |
| CO | 2895 ppm | DANGER |
| H2 | 1931 ppm | WARNING |
| NH3 | 2727 ppm | DANGER |
| PM2.5 | 1361 µg/m³ | DANGER |

Trend: rising at 91.7 ppm/m. Next-meter prediction: 2987.7 ppm.
Leak zone localized spatially to distance 85–95 m at altitude ~44,330 cm.

---
<img width="1280" height="720" alt="WhatsApp Image 2026-05-01 at 21 42 24" src="https://github.com/user-attachments/assets/b78bd902-c3ba-4695-ba2d-46b930e3a3ed" />

<img width="1600" height="905" alt="WhatsApp Image 2026-05-01 at 21 42 26 (1)" src="https://github.com/user-attachments/assets/8c3ac746-5363-43ec-b344-409b992fc1fd" />

<img width="1280" height="720" alt="WhatsApp Image 2026-05-01 at 21 42 26 (2)" src="https://github.com/user-attachments/assets/4ab781b1-9fb3-43e2-9347-ed710c92f865" />
<img width="1280" height="720" alt="WhatsApp Image 2026-05-01 at 21 42 26 (3)" src="https://github.com/user-attachments/assets/0f4c6aee-32b3-4b97-b41c-7c5969a401e2" />

<img width="1600" height="1200" alt="WhatsApp Image 2026-05-01 at 21 42 21 (1)" src="https://github.com/user-attachments/assets/b2f4afe4-1fc1-4a89-8976-173f75be9820" />
<img width="1280" height="765" alt="WhatsApp Image 2026-05-01 at 21 42 21 (2)" src="https://github.com/user-attachments/assets/ae5318ce-d821-41ff-8445-edd886557fdb" />
<img width="1600" height="1200" alt="WhatsApp Image 2026-05-01 at 21 42 21 (3)" src="https://github.com/user-attachments/assets/bdc67608-3a7c-4675-ac18-b773131957b4" />
<img width="715" height="1600" alt="WhatsApp Image 2026-05-01 at 21 42 20" src="https://github.com/user-attachments/assets/222a86e9-fb2b-4907-8a75-efcf3e662683" />


## Roadmap

- SLAM integration (ORB-SLAM3 on Jetson Nano) for globally consistent 3D mapping
- Thermal imaging (MLX90640) for hot-spot and independent human detection
- Multi-label gas classification from sensor array signatures
- LoRaWAN for long-range and multi-drone mesh communication
- ML-based structural crack detection from visual feed

---

## License

MIT License. See [LICENSE](LICENSE).

---

## Author
Jay
